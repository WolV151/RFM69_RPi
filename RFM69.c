#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdbool.h> 
#include "RFM69.h"
//#include "GPIOMap.h"
//#include "RFM69registers.h"

unsigned char _powerLevel = 31;
bool _isRFM69HW = 0;
unsigned char _mode = RF69_MODE_RX;
unsigned short _address;
bool _haveData = 0;

unsigned char initialize(unsigned char freqBand, unsigned short ID, unsigned char networkID, unsigned char intPin, unsigned char rstPin, unsigned char spiBus)
{
    wiringPiSetupPhys();
    //pinMode(intPin, INPUT);
    pinMode(rstPin, OUTPUT);
    //pinMode(rstPin, OUTPUT);
    wiringPiSPISetup(spiBus, 4000000);
    const unsigned char CONFIG[][2] =
    {
        /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
        /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
        /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_50000}, // default: 4.8 KBPS
        /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_50000},
        /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
        /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

        /* 0x07 */ { REG_FRFMSB, (unsigned char) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
        /* 0x08 */ { REG_FRFMID, (unsigned char) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
        /* 0x09 */ { REG_FRFLSB, (unsigned char) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

        // looks like PA1 and PA2 are not implemented on RFM69W/CW, hence the max output power is 13dBm
        // +17dBm and +20dBm are possible on RFM69HW
        // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
        // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
        // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
        ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
        ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

        // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
        /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
        //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
        /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
        /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
        /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
        /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
        ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
        /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
        /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
        /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
        //* 0x31 */ { REG_SYNCVALUE3, 0xAA },
        //* 0x31 */ { REG_SYNCVALUE4, 0xBB },
        /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
        /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
        ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
        /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
        /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
        {255, 0}
    };
  
    //Verify chip syncing
    unsigned int start = millis();
    unsigned char timeout = 50;
    
    do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
    start = millis();
    do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);

    for (unsigned char i = 0; CONFIG[i][0] != 255; i++)
        writeReg(CONFIG[i][0], CONFIG[i][1]);
    
    encrypt(0);

    setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
    setMode(RF69_MODE_STANDBY);
    start = millis();
    while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
    if (millis()-start >= timeout)
        return false;
    wiringPiISR(intPin, INT_EDGE_RISING, isr0);

    _address = ID;
    return true;
}
/*void setAddress(unsigned short addr);
void setNetwork(unsigned char networkID);
bool canSend();
void send(unsigned short toAddress, const void* buffer, unsigned char bufferSize, bool requestACK=false);
bool sendWithRetry(unsigned short toAddress, const void* buffer, unsigned char bufferSize, unsigned char retries=2, unsigned char retryWaitTime=RFM69_ACK_TIMEOUT);
bool receiveDone();
bool ACKReceived(unsigned short fromNodeID);
bool ACKRequested();
void sendACK(const void* buffer = "", unsigned char bufferSize=0);
int getFrequency(); //-------//
void setFrequency(unsigned int freqHz);*/
void encrypt(const char* key)
{
    setMode(RF69_MODE_STANDBY);
    if (key != 0)
    {
        char data = REG_AESKEY1 | 0x80;
        wiringPiSPIDataRW(0, &data, 1);
        for (unsigned char i = 0; i < 16; i++)
            wiringPiSPIDataRW(0, &key[i], 1);
    }
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1 : 0));
}
/*void setCS(unsigned char newSPISlaveSelect); //-------//
short readRSSI(bool forceTrigger=false); // *current* signal strength indicator; e.g. < -90dBm says the frequency channel is free + ready to transmit
void spyMode(bool onOff=true); //-------//
void promiscuous(bool onOff=true); //deprecated, replaced with spyMode()*/
void setHighPower(bool onOFF) // has to be called after initialize() for RFM69HW
{
    _isRFM69HW = onOFF;
    writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
    if (_isRFM69HW) // turning ON
        writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
    else
        writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}
/*void setPowerLevel(unsigned char level); // reduce/increase transmit power level
void sleep();
unsigned char readTemperature(unsigned char calFactor=0); // get CMOS temperature (8bit)
void rcCalibration();
void sendFrame(unsigned short toAddress, const void* buffer, unsigned char size, bool requestACK=false, bool sendACK=false);
void receiveBegin();*/
void setMode(unsigned char mode)
{
    if (mode == _mode)
        return;

    switch (mode) {
        case RF69_MODE_TX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
            if (_isRFM69HW) setHighPowerRegs(true);
                break;
        case RF69_MODE_RX:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
            if (_isRFM69HW) setHighPowerRegs(false);
                break;
        case RF69_MODE_SYNTH:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
            break;
        case RF69_MODE_STANDBY:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
            break;
        case RF69_MODE_SLEEP:
            writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
            break;
        default:
            return;
    }
}
void setHighPowerRegs(bool onOff)
{
    writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
    writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}
//void interruptHandler();
unsigned char readReg(unsigned char addr)
{
    char data[2]={0};
    data[0]=addr&0x7F;
    return wiringPiSPIDataRW(0, data, 2);
}
void writeReg(unsigned char addr, unsigned char val)
{
    char data[2]={0};
    data[0]=addr|0x80;
    data[1]=val;
    wiringPiSPIDataRW(0, data, 2);
}
/*void readAllRegs();
bool shutdown();*/
void isr0()
{
    _haveData = true;
}