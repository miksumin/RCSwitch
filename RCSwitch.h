/*
  RCSwitch - Arduino libary for remote control outlet switches
  Copyright (c) 2011 Suat Özgür.  All right reserved.

  Contributors:
  - Andre Koehler / info(at)tomate-online(dot)de
  - Gordeev Andrey Vladimirovich / gordeev(at)openpyro(dot)com
  - Skineffect / http://forum.ardumote.com/viewtopic.php?f=2&t=46
  - Dominik Fischer / dom_fischer(at)web(dot)de
  - Frank Oltmanns / <first name>.<last name>(at)gmail(dot)com
  - Max Horn / max(at)quendi(dot)de
  - Robert ter Vehn / <first name>.<last name>(at)gmail(dot)com
  
  Project home: https://github.com/sui77/rc-switch/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef _RCSwitch_h
#define _RCSwitch_h

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad, FraunchPad and StellarPad specific
    #include "Energia.h"
#elif defined(RPI) // Raspberry Pi
    #define RaspberryPi
    // Include libraries for RPi:
    #include <string.h> /* memcpy */
    #include <stdlib.h> /* abs */
    #include <wiringPi.h>
#elif defined(SPARK)
    #include "application.h"
#else
    #include "WProgram.h"
#endif

#include <stdint.h>

// At least for the ATTiny X4/X5, receiving has to be disabled due to
// missing libm depencies (udivmodhi4)
#if defined( __AVR_ATtinyX5__ ) or defined ( __AVR_ATtinyX4__ )
	#define RCSwitchDisableReceiving
#endif

// Number of maximum high/Low changes per packet.
// We can handle up to (unsigned long) => 32 bit * 2 H/L changes per bit + 2 for sync
#define RCSWITCH_MAX_CHANGES 	165		//75	//67
#define ROWDATA_MAX_BYTES		10

class RCSwitch {

  public:
  
    RCSwitch();
    
    void switchOn(int nGroupNumber, int nSwitchNumber);
    void switchOff(int nGroupNumber, int nSwitchNumber);
    void switchOn(const char* sGroup, int nSwitchNumber);
    void switchOff(const char* sGroup, int nSwitchNumber);
    void switchOn(char sFamily, int nGroup, int nDevice);
    void switchOff(char sFamily, int nGroup, int nDevice);
    void switchOn(const char* sGroup, const char* sDevice);
    void switchOff(const char* sGroup, const char* sDevice);
    void switchOn(char sGroup, int nDevice);
    void switchOff(char sGroup, int nDevice);

    void sendTriState(const char* sCodeWord);
    void send(unsigned long code, unsigned int length);
    void send(const char* sCodeWord);
    
    void enableTransmit(int nTransmitterPin, int nTransmitterPower);
    void disableTransmit();
    void setPulseLength(int nPulseLength);
    void setRepeatTransmit(int nRepeatTransmit);
    
#if not defined( RCSwitchDisableReceiving )
    
	void setReceiveTolerance(int nPercent);
	void enableReceive(int digitalPin);
    void enableReceive();
    void disableReceive();
    bool available();
    void resetAvailable();

    unsigned long getReceivedValue();
	unsigned int getReceivedChecksum();
    unsigned int getReceivedBitlength();
    unsigned int getReceivedDelay();
    unsigned int getReceivedProtocol();
    unsigned int* getReceivedTimings();
	// for 10 bytes Manchester code
	byte* getReceivedData();
	byte getReceivedLength();
	unsigned long getDeviceID();
	//
	byte getSwitchID();
	byte getChannelID();
	byte getDeviceType();
	//unsigned int getCheckSum();
	float getTemperature();
	byte getHumidity();
	byte getBattery();
	//boolean isCheckSumMatch();

#endif

    /**
     * Description of a single pule, which consists of a high signal
     * whose duration is "high" times the base pulse length, followed
     * by a low signal lasting "low" times the base pulse length.
     * Thus, the pulse overall lasts (high+low)*pulseLength
     */
    struct HighLow {
        uint8_t high;
        uint8_t low;
    };

    /**
     * A "protocol" describes how zero and one bits are encoded into high/low pulses.
     */
    struct Protocol {
        uint16_t pulseLength;	// base pulse length in microseconds, e.g. 350
        HighLow syncFactor;
        HighLow zeroBit;
        HighLow oneBit;
		bool invertedSignal;	// If true, interchange high and low logic levels in all transmissions.
        /*
         * By default, RCSwitch assumes that any signals it sends or receives
         * can be broken down into pulses which start with a high signal level,
         * followed by a a low signal level. This is e.g. the case for the
         * popular PT 2260 encoder chip, and thus many switches out there.
         *
         * But some devices do it the other way around, and start with a low
         * signal level, followed by a high signal level, e.g. the HT6P20B. To
         * accommodate this, one can set invertedSignal to true, which causes
         * RCSwitch to change how it interprets any HighLow struct FOO: It will
         * then assume transmissions start with a low signal lasting
         * FOO.high*pulseLength microseconds, followed by a high signal lasting
         * FOO.low*pulseLength microseconds.
         */
        
    };

    void setProtocol(Protocol protocol);
    void setProtocol(int nProtocol);
    void setProtocol(int nProtocol, int nPulseLength);

  private:
  
    char* getCodeWordA(const char* sGroup, const char* sDevice, bool bStatus);
    char* getCodeWordB(int nGroupNumber, int nSwitchNumber, bool bStatus);
    char* getCodeWordC(char sFamily, int nGroup, int nDevice, bool bStatus);
    char* getCodeWordD(char group, int nDevice, bool bStatus);
    
	void transmit(HighLow pulses);

    int nTransmitterPin;
    int nRepeatTransmit;
	int nTransmitterPower;
    
    Protocol protocol;
	
#if not defined( RCSwitchDisableReceiving )
	
    static void handleTimer();
	static void handleInterrupt();
    static bool receiveProtocol(const int p, unsigned int changeCount);
	int nReceiverInterrupt;
	
	static byte mCodeCheckData(byte length);
	static byte DecryptAndCheck(byte *data);
   
    static int nReceiveTolerance;
    const static unsigned int nSeparationLimit;
	
	static int nReceiverPin;
	
	volatile static unsigned long nReceivedValue;
    volatile static unsigned int nReceivedBitlength;
    volatile static unsigned int nReceivedDelay;
    volatile static unsigned int nReceivedProtocol;
	volatile static unsigned int nReceivedChecksum;
	
	//volatile static byte nReceivedData[ROWDATA_MAX_BYTES];
	volatile static byte nReceivedLength;
	
	//volatile static byte nCheckSum;
	//volatile static bool nCheckSumMatch;
	
	static unsigned long nDeviceID;
	static byte nSwitchID;
	static byte nChannelID;
	static byte nDeviceType;
	static float nTemperature;
	static byte nHumidity;
	static byte nBattery;
	
    /* 
     * timings[0] contains sync timing, followed by a number of bits
     */
    static unsigned int timings[RCSWITCH_MAX_CHANGES];
	static unsigned int timings2[255];
	static byte mCodeData[ROWDATA_MAX_BYTES];
	
#endif
    
};

#endif

