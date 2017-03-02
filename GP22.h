#ifndef GP22_h
#define GP22_h

#include "stdlib.h"
#include "stdint.h"
#include "SPI.h"

// Make it easy to mention the channels
enum Channel: uint8_t {
  CH1, CH2
};

//Setup the 32 to 8 bit variable type
union FourByte {
  uint32_t bit32;
  uint16_t bit16[2];
  uint8_t bit8[4];
};

struct ALUInstruction {
  int id;
  uint8_t hit1Op;
  uint8_t hit2Op;
};

class GP22
{
public:
  GP22(int slaveSelectPin);
  ~GP22();

  // Start communicating. Transfers the config over as well so call this
  // after configuring the settings as required.
  void begin();

  // Initialise the GP22, then it waits for an event to measure.
  void measure();

  /// Status related functions
  // Read the GP22s status register into memory.
  // This must be called first to update the status from the TDC.
  void readStatus();
  // Was there a timeout?
  bool timedOut();
  // How many hits were there for each channel?
  uint8_t getMeasuredHits(Channel channel);
  // What is the current read register pointer?
  uint8_t getReadPointer();

  // The measurement reading command
  uint32_t readResult(uint8_t resultRegister);

  // Test to make sure that the communication is working
  bool testComms();

  // This is the conversion function which takes a raw input
  // and converts it to microseconds
  float measConv(uint32_t input);

  //// These are the config setting/getting functions
  /// This is for the number of expected hits, can be 2-4
  void setExpectedHits(Channel channel, uint8_t hits);
  uint8_t getExpectedHits(Channel channel);

  /// These are for the resolution mode of the measurement
  void setSingleRes();
  bool isSingleRes();
  void setDoubleRes();
  bool isDoubleRes();
  void setQuadRes(); // Default on
  bool isQuadRes();

  /// Measurement mode settings
  void setMeasurementMode(uint8_t mode); // Can be 1 or 2
  uint8_t getMeasurementMode();
  // This is for the measurement mode 1 clock pre-divider.
  // This can be 1, 2 or 4.
  void setClkPreDiv(uint8_t div);
  uint8_t getClkPreDiv();
  // This is for the MM2 Auto calc, if on, writes the sum of
  // all hits to register 4
  void setAutoCalcOn(bool on);
  bool isAutoCalcOn();

  /// ALU processing operator settings
  // In MM1 ALU calculates HIT1-HIT2, MM2 it calcs HIT2 - HIT1
  // The operators are also different for both modes, see datasheet
  // Define HIT1 operator
  void defineHit1Op(uint8_t op);
  uint8_t getHit1Op();
  // Define HIT2 operator
  void defineHit2Op(uint8_t op);
  uint8_t getHit2Op();
  // Fast update the ALU hit operators for doing multiple ALU calculations.
  void updateALUInstruction(ALUInstruction instruction);

  /// Set the channel edge sensitivities
  // The edge sensitivity can be 0 (rising), 1 (falling) or 2 (both).
  // (NOTE: start cannot be both).
  void setEdgeSensitivity(uint8_t start, uint8_t stop1, uint8_t stop2);

  /// First wave mode settings
  void setFirstWaveMode(bool on);
  bool isFirstWaveMode();
  // This is to set the relative delay of the stops after the first wave.
  // Rule: 3 <= stop1 < stop2 < stop3 <= 63
  void setFirstWaveDelays(uint8_t stop1, uint8_t stop2, uint8_t stop3);
  // The pulse width measurement setting
  void setPulseWidthMeasOn(bool on);
  bool isPulseWidthMeasOn();
  // The first wave edge sensitivity setting
  void setFirstWaveRisingEdge(bool on);
  bool isFirstWaveRisingEdge();
  // The first wave offset setting controls the initial offset.
  // The value can be between -36 and +35 mV.
  void setFirstWaveOffset(int8_t offset);
  int8_t getFirstWaveOffset();

  // This writes the config register to the GP22.
  // Call this after changing any of the settings to update them on the GP22 itself.
  // (You can do a series of settings changes and call this at the end.)
  void updateConfig();

  // Will fill a 7 by 32 bit array with the config registers
  void getConfig(uint32_t * arrayToFill);
    
private:

  // The fast SPI transfer functions
  uint8_t transfer1B(uint8_t opcode, uint8_t byte1);
  uint16_t transfer2B(uint8_t opcode, uint8_t byte1, uint8_t byte2);
  uint32_t transfer4B(uint8_t opcode, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

  // The slave select pin used by SPI to communicate with the GP22
  int _ssPin;
  uint16_t _status;

  // Have the conversion from the raw result to time precalculated.
  void updateConversionFactor();
  float _conversionFactor;

  // Set the config to single pulse measurement mode 2 as default for now
  uint8_t _config[7][4] = {
    {0xF3, 0x07, 0x68, 0x00}, // Reg 0
    {0x21, 0x42, 0x00, 0x00}, // Reg 1
    {0x20, 0x00, 0x00, 0x00}, // Reg 2
    {0x20, 0x00, 0x00, 0x00}, // Reg 3
    {0x20, 0x00, 0x00, 0x00}, // Reg 4
    {0x40, 0x00, 0x00, 0x00}, // Reg 5
    {0x40, 0x20, 0x60, 0x00}  // reg 6
  };

  //uint8_t _readRegs[7] = {0x81};
};

#endif