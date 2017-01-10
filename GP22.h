#ifndef GP22_h
#define GP22_h

#include "stdlib.h"
#include "stdint.h"
#include "SPI.h"

//Setup the 32 to 8 bit variable type
union FourByte {
  uint32_t bit32;
  uint16_t bit16[2];
  uint8_t bit8[4];
};

class GP22
{
public:
  GP22(int slaveSelectPin);
  ~GP22();

  // Start communicating. Transfers the config over as well.
  void begin();

  // Initialise the GP22, then it waits for an event to measure.
  void measure();

  // Read the GP22s status register
  uint16_t readStatus();

  // The measurement reading command
  uint32_t readResult(uint8_t resultRegister);

  // Test to make sure that the communication is working
  bool testComms();

  // This is the conversion function which takes a raw input
  // and converts it to microseconds
  float measConv(uint32_t input);

  //// These are the config setting/getting functions
  void setExpectedHits(uint8_t hits);
  uint8_t getExpectedHits();
  void setSingleRes(bool on);
  bool isSingleRes();
  void setDoubleRes(bool on);
  bool isDoubleRes();
  void setQuadRes(bool on);
  bool isQuadRes();

  void setFirstWaveMode(bool on);
  bool isFirstWaveMode();

  // This writes the config register to the GP22.
  // Call this after changing any of the settings to update them on the GP22 itself.
  // (You can do a series of settings changes and call this at the end.)
  void updateConfig();
    
private:

  // The fast SPI transfer functions
  uint8_t transfer1B(uint8_t opcode, uint8_t byte1);
  uint16_t transfer2B(uint8_t opcode, uint8_t byte1, uint8_t byte2);
  uint32_t transfer4B(uint8_t opcode, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

  // The slave select pin used by SPI to communicate with the GP22
  int _ssPin;

  // Set the config to single pulse measurement as default for now
  uint8_t _config[7][4] = {
    {0xF3, 0x07, 0x68, 0x00},
    {0x21, 0x42, 0x00, 0x00}, // What is [1][1] = 0x42 vs. 0x23?
    {0x20, 0x00, 0x00, 0x00},
    {0x20, 0x00, 0x00, 0x00},
    {0x10, 0x00, 0x00, 0x00},
    {0x40, 0x00, 0x00, 0x00},
    {0x40, 0x20, 0x60, 0x00}
  };

  //uint8_t _readRegs[7] = {0x81};
};

#endif
