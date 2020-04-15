#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdbool.h> 
//#include "GPIOMap.h"

#define RF69_MAX_DATA_LEN       61 // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define CSMA_LIMIT              -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP         0 // XTAL OFF
#define RF69_MODE_STANDBY       1 // XTAL ON
#define RF69_MODE_SYNTH         2 // PLL ON
#define RF69_MODE_RX            3 // RX MODE
#define RF69_MODE_TX            4 // TX MODE

// available frequency bands
#define RF69_315MHZ            31 // non trivial values to avoid misconfiguration
#define RF69_433MHZ            43
#define RF69_868MHZ            86
#define RF69_915MHZ            91

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_BROADCAST_ADDR   0
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS   1000
#define RF69_FSTEP  61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

// TWS: define CTLbyte bits
#define RFM69_CTL_SENDACK   0x80
#define RFM69_CTL_REQACK    0x40

#define RFM69_ACK_TIMEOUT   30  // 30ms roundtrip req for 61byte packets

uint8_t initialize(uint8_t freqBand, uint16_t ID, uint8_t networkID=1); //-------//
/*void setAddress(uint16_t addr);
void setNetwork(uint8_t networkID);
bool canSend();
void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);
bool sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=RFM69_ACK_TIMEOUT);
bool receiveDone();
bool ACKReceived(uint16_t fromNodeID);
bool ACKRequested();
void sendACK(const void* buffer = "", uint8_t bufferSize=0);
int32_t getFrequency(); //-------//
void setFrequency(uint32_t freqHz);*/
void encrypt(const char* key);
/*void setCS(uint8_t newSPISlaveSelect); //-------//
int16_t readRSSI(bool forceTrigger=false); // *current* signal strength indicator; e.g. < -90dBm says the frequency channel is free + ready to transmit
void spyMode(bool onOff=true); //-------//
void promiscuous(bool onOff=true); //deprecated, replaced with spyMode()*/
void setHighPower(bool onOFF=true); // has to be called after initialize() for RFM69HW
/*void setPowerLevel(uint8_t level); // reduce/increase transmit power level
void sleep();
uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)
void rcCalibration();
void sendFrame(uint16_t toAddress, const void* buffer, uint8_t size, bool requestACK=false, bool sendACK=false);
void receiveBegin();*/
void setMode(uint8_t mode);
void setHighPowerRegs(bool onOff);
//void interruptHandler();
uint8_t readReg(uint8_t addr);
void writeReg(uint8_t addr, uint8_t val);
//void readAllRegs();
//bool shutdown();
void isr0();