#include "GP22.h"

GP22::GP22(int slaveSelectPin) {
  // Set the internal variable for the SPI slave select.
  _ssPin = slaveSelectPin;
  // Precalculate the conversion factor based on default settings.
  updateConversionFactor();
}

GP22::~GP22() {
  SPI.end();
}

void GP22::begin() {
  //Start up SPI
  SPI.begin(_ssPin);
  //Run the SPI clock at 14 MHz (GP22's max is apparently 20 MHz)
  SPI.setClockDivider(_ssPin, 6);
  //Clock polarity = 0, clock phase = 1 (MODE1?)
  SPI.setDataMode(_ssPin, SPI_MODE1);
  //The GP22 sends the most significant bit first
  SPI.setBitOrder(_ssPin, MSBFIRST);
  //Power-on-reset command
  SPI.transfer(_ssPin, 0x50);
  //Transfer the GP22 config registers across
  updateConfig();
}

//Initilise measurement
void GP22::measure() {
  SPI.transfer(_ssPin, 0x70);
}

void GP22::readStatus() {
  // Get the TDC status from it's stat register
  _status = transfer2B(0xB4, 0x00, 0x00);
}
bool GP22::timedOut() {
  return (_status & 0x0600) > 0;
}
uint8_t GP22::getMeasuredHits(Channel channel) {
  if (channel == CH1) {
    return (_status & 0x0038) >> 3;
  } else if (channel == CH2) {
    return (_status & 0x01C0) >> 6;
  }
}
uint8_t GP22::getReadPointer() {
  return _status & 0x0007;
}

//Function to read from result registers (as a signed int, as MM1 uses 2's comp)
int32_t GP22::readResult(uint8_t resultRegister) {
  // Make sure that we are only reading one of the 4 possibilities
  if (resultRegister < 4 && resultRegister >= 0) {
    // The first read code is 0xB0, so add the register to get the required read code.
    uint8_t readCode = 0xB0 + resultRegister;
    return transfer4B(readCode, 0, 0, 0, 0);
  } else {
    // No such register, return 0;
    return 0;
  }
}

// These are the functions designed to make tranfers quick enough to work
// by sending the opcode and immediatly following with data (using SPI_CONTINUE).
uint8_t GP22::transfer1B(uint8_t opcode, uint8_t byte1) {
  FourByte data = { 0 };
  SPI.transfer(_ssPin, opcode, SPI_CONTINUE);
  data.bit8[0] = SPI.transfer(_ssPin, byte1);
  return data.bit8[0];
}
uint16_t GP22::transfer2B(uint8_t opcode, uint8_t byte1, uint8_t byte2) {
  FourByte data = { 0 };
  SPI.transfer(_ssPin, opcode, SPI_CONTINUE);
  data.bit8[1] = SPI.transfer(_ssPin, byte1, SPI_CONTINUE);
  data.bit8[0] = SPI.transfer(_ssPin, byte2);
  return data.bit16[0];
}
uint32_t GP22::transfer4B(uint8_t opcode, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
  FourByte data = { 0 };
  SPI.transfer(_ssPin, opcode, SPI_CONTINUE);
  data.bit8[3] = SPI.transfer(_ssPin, byte1, SPI_CONTINUE);
  data.bit8[2] = SPI.transfer(_ssPin, byte2, SPI_CONTINUE);
  data.bit8[1] = SPI.transfer(_ssPin, byte3, SPI_CONTINUE);
  data.bit8[0] = SPI.transfer(_ssPin, byte4);
  return data.bit32;
}

bool GP22::testComms() {
  // The comms can be tested by reading read register 5, which contains the highest 8 bits of config reg 1.
  int test = transfer1B(0xB5, 0);
  // Now test the result is the same as the config register (assuming the registers have been written!).
  if (test == _config[1][0]) {
    return true;
  } else {
    return false;
  }
}

float GP22::measConv(int32_t input) {
  // Use the precalculated conversion factor.
  return ((float)input) * _conversionFactor;
}

void GP22::updateConversionFactor() {
  // This number takes cycles to calculate, so precalculate it.
  // It only needs calculating at startup and on changing the clock settings.

  // Input is a Q16.16 number representation, 
  // thus conversion is via multiplication by 2^(-16).
  // The input in also multiples of the clock (4MHz).
  // Output is in microseconds.

  float qConv = pow(2.0, -16);    //Q conversion factor
  float tRef = (1.0) / (4000000.0); //4MHz clock
  float timeBase = 1000000.0;   //Microseconds
  float N = getClkPreDiv(); // The Clock predivider correction

  _conversionFactor = tRef * qConv * timeBase * N;
}

void GP22::updateConfig() {
  //Transfer the configuration registers

  // The first config register is 0x80 and the last is 0x86
  // I know, this is a bit cheeky, but I just really wanted to try it...
  for (uint8_t i = 0; i < 7; i++)
    transfer4B((0x80 + i), _config[i][0], _config[i][1], _config[i][2], _config[i][3]);
}

void GP22::getConfig(uint32_t * arrayToFill) {
  // Fill the array with the config registers, combined into 32 bits

  for (uint8_t i = 0; i < 7; i++)
    arrayToFill[i] = (_config[i][0] << 24) + (_config[i][1] << 16) + (_config[i][2] << 8) + _config[i][3];
}

//// The config setting/getting functions

// Measurement mode selection
void GP22::setMeasurementMode(uint8_t mode) {
  uint8_t configPiece = _config[0][2];

  // Reg 0, bit 11, called MESSB2 selects which measurement mode
  // to use. MESSB2 = 0 is mode 1, MESSB2 = 1 is mode 2.

  if (mode == 1) {
    // Set MESSB2 = 0 (Measurement Mode 1)
    bitClear(configPiece, 3);

    // In measurement mode 1, only double res is available.
    // Thus, if the current settings are for quad res (from MM2), 
    // change it to double res instead.
    if (isQuadRes())
      setDoubleRes();
  } else if (mode == 2) {
    // Set MESSB2 = 1 (Measurement Mode 2)
    bitSet(configPiece, 3);
  }

  _config[0][2] = configPiece;
}
uint8_t GP22::getMeasurementMode() {
  return (_config[0][2] & B00001000) > 0;
}

// This is for the measurement mode 1 clock pre-divider
void GP22::setClkPreDiv(uint8_t div) {
  uint8_t configPiece = _config[0][1];

  // The only valid divisions are 1, 2 and 4
  if (div == 1 || div == 2 || div == 4) {
    // Start by clearing the bits (same as setting div to 1)
    bitClear(configPiece, 4);
    bitClear(configPiece, 5);

    // Now set the bits as required for div = 2 or 4
    if (div == 2)
      bitSet(configPiece, 4);
    else if (div == 4)
      bitSet(configPiece, 5);
  }

  _config[0][1] = configPiece;

  // As the clock settings have been changed...
  updateConversionFactor();
}
uint8_t GP22::getClkPreDiv() {
  uint8_t divRaw = (_config[0][1] & B00110000) >> 4;
  uint8_t div = 0;

  switch (divRaw) {
    case 0:
      div = 1;
      break;
    case 1:
      div = 2;
      break;
    case 2:
    case 3:
      div = 4;
      break;
  }

  return div;
}

// The hits of Ch1 are stored in bits 16-18 in register 1
void GP22::setExpectedHits(Channel channel, uint8_t hits) {
  // First lets get the bit of the config register we want to modify
  uint8_t configPiece = _config[1][1];

  // Now, we need to set and clear bits as necessary
  // The minimum number of hits is 0, the max is 4.
  if (hits >= 0 & hits <= 4) {
    if (channel == CH1) {
      bitClear(configPiece, 0);
      bitClear(configPiece, 1);
      bitClear(configPiece, 2);

      configPiece += hits;
    } else if (channel == CH2) {
      bitClear(configPiece, 3);
      bitClear(configPiece, 4);
      bitClear(configPiece, 5);

      configPiece += (hits << 3);
    }
  }

  // Now that the peice of the config that needed to be changed has been, lets put it back
  _config[1][1] = configPiece;
  // It is up to the user to update the GP22s registers
  // (in case they want to chain together setting modifications).
  // Also, so this can be called before the begin function is called.
}
uint8_t GP22::getExpectedHits(Channel channel) {
  if (channel == CH1) {
    return _config[1][1] & B00000111;
  } else if (channel == CH2) {
    return (_config[1][1] & B00111000) >> 3;
  }
}

void GP22::updateALUInstruction(ALUInstruction instruction) {
  // First, update the config registers
  defineHit1Op(instruction.hit1Op);
  defineHit2Op(instruction.hit2Op);
  // Now, we only want to update the relevent config register,
  // as this is quicker than doing everything...
  // The config register with the operators is Reg 1, so update that one!
  transfer4B((0x81), _config[1][0], _config[1][1], _config[1][2], _config[1][3]);
}

// Define HIT operators for ALU processing
void GP22::defineHit1Op(uint8_t op) {
  uint8_t configPiece = _config[1][0];

  // Clear the first 4 bits of the byte
  for (int i = 0; i < 4; i++)
    bitWrite(configPiece, i, 0);

  // Now write the operator into the first four bits
  configPiece += op;

  // Then write to the config
  _config[1][0] = configPiece;
}
void GP22::defineHit2Op(uint8_t op) {
  uint8_t configPiece = _config[1][0];

  // Clear the second 4 bits of the byte
  for (int i = 4; i < 8; i++)
    bitWrite(configPiece, i, 0);

  // Now write the op into the top 4 bits
  configPiece += (op << 4);

  // Then write to the config
  _config[1][0] = configPiece;
}
uint8_t GP22::getHit1Op() {
  return _config[1][0] & B00001111;
}
uint8_t GP22::getHit2Op() {
  return (_config[1][0] & B11110000) >> 4;
}

// Define the edge sensitivities of the inputs
void GP22::setEdgeSensitivity(uint8_t start, uint8_t stop1, uint8_t stop2) {
  uint8_t reg0p2 = _config[0][2];
  uint8_t reg2p0 = _config[2][0];

  // Deal with the start, which can only be rising or falling, not both.
  if (start == 0 || start == 1) {
    bitWrite(reg0p2, 0, start);
  }

  // Stop 1 and 2 can be rising, falling or both.
  if (stop1 == 0 || stop1 == 1) {
    // Deal with rising or falling
    bitWrite(reg0p2, 1, stop1);
  } else if (stop1 == 2) {
    // Deal with both, i.e. set to rising sensitivity
    // and make the stop trigger on both edges.
    bitClear(reg0p2, 1);
    bitSet(reg2p0, 3);
  }
  // Repeat for stop2
  if (stop2 == 0 || stop2 == 1) {
    bitWrite(reg0p2, 2, stop2);
  } else if (stop2 == 2) {
    bitClear(reg0p2, 2);
    bitSet(reg2p0, 4);
  }

  _config[0][2] = reg0p2;
  _config[2][0] = reg2p0;
}

void GP22::setSingleRes() {
  uint8_t configPiece = _config[6][2];

  bitClear(configPiece, 4);
  bitClear(configPiece, 5);

  _config[6][2] = configPiece;
}
bool GP22::isSingleRes() {
  return !isDoubleRes() && !isQuadRes();
}
void GP22::setDoubleRes() {
  uint8_t configPiece = _config[6][2];

  bitSet(configPiece, 4);
  bitClear(configPiece, 5);

  _config[6][2] = configPiece;
}
bool GP22::isDoubleRes() {
  return (_config[6][2] & B00010000) > 0;
}
void GP22::setQuadRes() {
  // Quad res is only available in measurement mode 2.
  if (getMeasurementMode() == 2) {
    uint8_t configPiece = _config[6][2];

    bitSet(configPiece, 5);
    bitClear(configPiece, 4);

    _config[6][2] = configPiece;
  }
}
bool GP22::isQuadRes() {
  return (_config[6][2] & B00100000) > 0;
}
void GP22::setAutoCalcOn(bool on) {
  uint8_t configPiece = _config[3][0];

  if (on)
    bitSet(configPiece, 7);
  else
    bitClear(configPiece, 7);

  _config[3][0] = configPiece;
}
bool GP22::isAutoCalcOn() {
  return (_config[3][0] & B10000000) > 0;
}

void GP22::setFirstWaveMode(bool on) {
  // First wave on/off is bit 30 of reg 3
  uint8_t configPiece = _config[3][0];

  bitSet(configPiece, 6);

  _config[3][0] = configPiece;
}
bool GP22::isFirstWaveMode() {
  return (_config[3][0] & B01000000) > 0;
}

void GP22::setFirstWaveDelays(uint8_t stop1, uint8_t stop2, uint8_t stop3) {
  // Grab the relevant bytes from Reg 3 to modify
  FourByte configPiece = { 0 };
  configPiece.bit8[3] = _config[3][0];
  configPiece.bit8[2] = _config[3][1];
  configPiece.bit8[1] = _config[3][2];
  configPiece.bit8[0] = _config[3][3];

  // DELREL1 is bits 8-13
  // DELREL2 is bits 14-19
  // DELREL3 is bits 20-25

  /// First sort out DELREL3
  // To make things easier, combine the top two bytes
  uint16_t topBytes = configPiece.bit16[1];
  // Now DELREL3 is bits 4-9 of topBytes
  // First clear bits 4-9.
  topBytes = topBytes ^ (topBytes & 0x03F0);
  // Now add the setting to those bits
  topBytes = topBytes + ((uint16_t)stop3 << 4);
  // Now we can write this back
  configPiece.bit16[1] = topBytes;

  /// Now DELREL2
  // This is tricky as it is spread accross the top and bottom half.
  // So, we grab the middle bytes.
  uint16_t middleBytes = ((uint16_t)configPiece.bit8[2] << 8) + configPiece.bit8[1];
  // Now DELREL2 is in bits 6-11 of middleBytes
  // So clear the bits that need to be modified
  middleBytes = middleBytes ^ (middleBytes & 0x0FC0);
  // Now add the new settings
  middleBytes = middleBytes + ((uint16_t)stop2 << 6);
  // Now we can put the bytes back
  configPiece.bit8[1] = (uint8_t)middleBytes;
  configPiece.bit8[2] = (uint8_t)(middleBytes >> 8);

  // Now DELREL1
  // This should be easy, as it's all in one byte
  uint8_t byte1 = configPiece.bit8[1];
  // So DELREL1 is in bits 0-5 of byte1
  byte1 = byte1 ^ (byte1 & B00111111);
  byte1 = byte1 + stop1;
  // Now we can write this back
  configPiece.bit8[1] = byte1;
}

void GP22::setPulseWidthMeasOn(bool on) {
  // DIS_PW, disable pusle width measurement is contained in bit 16, Reg 4
  uint8_t configPiece = _config[4][1];

  // bit 16 = 0 => Pulse width measurement enabled
  // bit 16 = 1 => disabled
  if (on)
    bitClear(configPiece, 0);
  else
    bitSet(configPiece, 0);

  _config[4][1] = configPiece;
}
bool GP22::isPulseWidthMeasOn() {
  return (_config[4][1] & B00000001) == 0;
}

void GP22::setFirstWaveRisingEdge(bool on) {
  uint8_t configPiece = _config[4][2];

  // bit 15 = 0 => Rising (positive) edge sensitive
  // bit 15 = 1 => Falling (negative) edge sensitive
  if (on)
    bitClear(configPiece, 7);
  else
    bitSet(configPiece, 7);

  _config[4][2] = configPiece;
}
bool GP22::isFirstWaveRisingEdge() {
  return (_config[4][2] & B10000000) > 0;
}

void GP22::setFirstWaveOffset(int8_t offset) {
  // There are three seperate settings that need to be configured.
  // First is the OFFS setting, in bits 8-12 of REG4.
  // The OFFS setting is a number between -16 and +15 in twos complement.
  // The next two settings are for adding on an addition +- 20 mV.
  /// First lets grab the byte in question
  uint8_t configPiece = _config[4][2];

  // Now we need to check if we need the extra ranges
  if (offset > 15) {
    // We are greater than the offset allows, so we need the extra +20 range
    offset -= 20;
    // Offset is now what it was, minus what the extra range gives
    bitSet(configPiece, 6);
    bitClear(configPiece, 5);
  } else if (offset < -16) {
    // The offset is less than it can be, so we need the extra -20 range
    offset += 20;
    // Now offset is what it was, add the extra range amount
    bitSet(configPiece, 5);
    bitClear(configPiece, 6);
  } else {
    // We seem to have an offset that is within range, so turn off the extra ranges
    bitClear(configPiece, 5);
    bitClear(configPiece, 6);
  }

  // Now we need to load the offset into bits 0-4 of the config byte.
  // It needs to be loaded as twos complement.
  // First lets clear the relevent bits
  configPiece = configPiece ^ (configPiece & B00011111);
  if ((offset > 0) && (offset <= 15)) {
    // If the number is positive, then we can just add it normally
    configPiece += offset;
  } else if ((offset < 0) && (offset >= -16)) {
    // This means we are in the negative part, so this is tricky.
    // Need to do a 5 bit 2s complement conversion.
    // First start with 5 bits all 1.
    uint8_t twosComp = 31;
    // Now add one to the offset, and then add it to the 1s
    twosComp = twosComp + (offset + 1);
    // Now bits 0-4 should contain the correct 5 bit 2s complement number
    configPiece += twosComp;
  }

  // So now that the config is set, put it back in place
  _config[4][2] = configPiece;
}
int8_t GP22::getFirstWaveOffset() {
  // First grab the relevant byte
  uint8_t configPiece = _config[4][2];
  // Next we need to grab the twos complement offset number
  uint8_t twosComp = configPiece & B00011111;
  // Prepare a variable to store the offset
  int8_t offset = 0;
  // Now parse the twos complement number
  if (twosComp > 15) {
    // If this number is greater than 15, then it must be negative
    offset = -1 * ((~twosComp) + 1);
  } else {
    // If it is less than that, then it is positive and nothing needs to be done
    offset = twosComp;
  }
  // Now we need to deal with any of the range additions
  if ((configPiece & B00100000) > 0) {
    // OFFSRNG1 is enabled, so take 20 from the offset
    offset -= 20;
  } else if ((configPiece & B01000000) > 0) {
    // OFFSRNG2 is enabled, so add 20 to the offset
    offset += 20;
  }

  return offset;
}