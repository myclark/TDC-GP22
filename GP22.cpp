#include "GP22.h"

GP22::GP22(int slaveSelectPin) {
  _ssPin = slaveSelectPin;
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

uint16_t GP22::readStatus() {
  // Get the TDC status from it's stat register
  uint16_t stat = transfer2B(0xB4, 0x00, 0x00);

  // It might be worth splitting up the result into more meaningful data than just a 16 bit number.
  // These numbers could go into private variables that other functions can access.

  return stat;
}

//Function to read from result registers
uint32_t GP22::readResult(uint8_t resultRegister) {
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

float GP22::measConv(uint32_t input) {
  // Input is a Q16.16 number representation, 
  // thus conversion is via multiplication by 2^(-16).
  // The input in also multiples of the clock (4MHz).
  // Output is in microseconds.

  float qConv = pow(2.0, -16);    //Q conversion factor
  float tRef = (1.0) / (4000000.0); //4MHz clock
  float timeBase = 1000000.0;   //Microseconds

  return ((float)input) * tRef * qConv * timeBase;
}

void GP22::updateConfig() {
  //Transfer the configuration registers

  // The first config register is 0x80 and the last is 0x86
  // I know, this is a bit cheeky, but I just really wanted to try it...
  for (uint8_t i = 0; i < 7; i++)
    transfer4B((0x80 + i), _config[i][0], _config[i][1], _config[i][2], _config[i][3]);
}

//// The config setting/getting functions

// The hits of Ch1 are stored in bits 16-18 in register 1
void GP22::setExpectedHits(uint8_t hits) {
  // First lets get the bit of the config register we want to modify
  uint8_t configPiece = _config[1][1];

  // Now, we need to set and clear bits as necessary
  // In measurement mode 2, the minimum number of hits is 2 (start is included), max is 4.
  switch (hits) {
    case 2:
      bitClear(configPiece, 0);
      bitSet(configPiece, 1);
      bitClear(configPiece, 2);
      break;
    case 3:
      bitSet(configPiece, 0);
      bitSet(configPiece, 1);
      bitClear(configPiece, 2);
      break;
    case 4:
      bitClear(configPiece, 0);
      bitClear(configPiece, 1);
      bitSet(configPiece, 2);
      break;
  }

  // Now that the peice of the config that needed to be changed has been, lets put it back
  _config[1][1] = configPiece;
  // It is up to the user to update the GP22s registers
  // (in case they want to chain together setting modifications).
  // Also, so this can be called before the begin function is called.
}
uint8_t GP22::getExpectedHits() {
  return _config[1][1] & B00000111;
}

void GP22::setSingleRes(bool on) {
  uint8_t configPiece = _config[6][2];

  if (on) {
    setDoubleRes(false);
    setQuadRes(false);
  }

  _config[6][2] = configPiece;
}
bool GP22::isSingleRes() {
  return !isDoubleRes() && !isQuadRes();
}
void GP22::setDoubleRes(bool on) {
  uint8_t configPiece = _config[6][2];

  if (on) {
    bitSet(configPiece, 4);
    setQuadRes(false);
  } else {
    bitClear(configPiece, 4);
  }

  _config[6][2] = configPiece;
}
bool GP22::isDoubleRes() {
  return (_config[6][2] & B00010000) > 0;
}
void GP22::setQuadRes(bool on) {
  uint8_t configPiece = _config[6][2];

  if (on) {
    bitSet(configPiece, 5);
    setDoubleRes(false);
  } else {
    bitClear(configPiece, 5);
  }

  _config[6][2] = configPiece;
}
bool GP22::isQuadRes() {
  return (_config[6][2] & B00100000) > 0;
}
void GP22::setAutoCalcOn(bool on) {
  uint8_t configPiece = _config[3][0];

  if (on)
    bitSet(configPiece, 0);
  else
    bitClear(configPiece, 0);

  _config[3][0] = configPiece;
}
bool GP22::isAutoCalcOn() {
  return (_config[3][0] & B10000000) > 0;
}

void GP22::setFirstWaveMode(bool on) {
  // First wave on/off is bit 30 of reg 3
  uint8_t configPiece = _config[3][0];

  bitSet(configPiece, 6);

  _config[4][0] = configPiece;
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