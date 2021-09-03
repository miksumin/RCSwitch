/*
  RCSwitch - Arduino libary for remote control outlet switches
  Copyright (c) 2011 Suat Özgür.  All right reserved.
  
  Contributors:
  - Andre Koehler / info(at)tomate-online(dot)de
  - Gordeev Andrey Vladimirovich / gordeev(at)openpyro(dot)com
  - Skineffect / http://forum.ardumote.com/viewtopic.php?f=2&t=46
  - Dominik Fischer / dom_fischer(at)web(dot)de
  - Frank Oltmanns / <first name>.<last name>(at)gmail(dot)com
  - Andreas Steinel / A.<lastname>(at)gmail(dot)com
  - Max Horn / max(at)quendi(dot)de
  - Robert ter Vehn / <first name>.<last name>(at)gmail(dot)com
  - Johann Richard / <first name>.<last name>(at)gmail(dot)com
  - Vlad Gheorghe / <first name>.<last name>(at)gmail(dot)com https://github.com/vgheo
  
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

#include "RCSwitch.h"

//#define RCSWITCH_DEBUG		// comment to disable Serial debugging

#ifdef RaspberryPi
    // PROGMEM and _P functions are for AVR based microprocessors,
    // so we must normalize these for the ARM processor:
    #define PROGMEM
    #define memcpy_P(dest, src, num) memcpy((dest), (src), (num))
#endif

#if defined(ESP8266)
    // interrupt handler and related code must be in RAM on ESP8266,
    // according to issue #46.
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
    #define VAR_ISR_ATTR
#elif defined(ESP32)
    #define RECEIVE_ATTR IRAM_ATTR
    #define VAR_ISR_ATTR DRAM_ATTR
#else
    #define RECEIVE_ATTR
    #define VAR_ISR_ATTR
#endif

/* Format for protocol definitions: {pulselength, Sync bit, "0" bit, "1" bit}
 * 
 * 		pulselength: pulse length in microseconds, e.g. 350
 * 
 *      _
 *     | |_______________________________  Sync bit: {1, 31} means 1 high pulse and 31 low pulses (perceived as a 31*pulselength long pulse, total length of sync bit is 32*pulselength microseconds)
 * 
 *      _
 *     | |___	"0" bit: waveform for a data bit of value "0", {1, 3} means 1 high pulse and 3 low pulses, total length (1+3)*pulselength
 * 
 *      ___
 *     |   |_	"1" bit: waveform for a data bit of value "1", e.g. {3,1}:
 *
 * These are combined to form Tri-State bits when sending or receiving codes.
 */

#if defined(ESP8266) || defined(ESP32)
static const RCSwitch::Protocol proto[] = {
#else
static const RCSwitch::Protocol PROGMEM proto[] = {
#endif

//{pulselength, Sync bit, "0" bit, "1" bit, inverted}
  //
  { 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1 (EV1527) - main protocol
  { 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
  { 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
  { 380, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
  { 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
  { 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true  },    // protocol 6 (HT6P20B)
  { 150, {  2, 62 }, {  1,  6 }, {  6,  1 }, false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
  { 200, { 3, 130 }, {  7, 16 }, {  3,  16}, false},     // protocol 8 Conrad RS-200 RX
  { 200, { 130, 7 }, {  16, 7 }, { 16,  3 }, true },     // protocol 9 Conrad RS-200 TX
  { 365, { 18,  1 }, {  3,  1 }, {  1,  3 }, true },     // protocol 10 (1ByOne Doorbell)
  { 270, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 11 (HT12E)
  { 320, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 12 (SM5212)
  //
  { 400, {  1, 22 }, {  1,  5 }, {  1, 10 }, false },    // protocol 13 (BL999 thermo/hydro sensor), Auriol H13726, Ventus WS155, Hama EWS 1500, Meteoscan W155/W160
  { 275, {  2, 14 }, {  2,  3 }, {  2, 7 }, false },    // protocol 14 (Garin WS-2) (3850/550/850/1900)
  //
};

enum {
   numProto = sizeof(proto) / sizeof(proto[0])
};

#if not defined(RCSwitchDisableReceiving)

volatile unsigned long RCSwitch::nReceivedValue = 0;
volatile unsigned int RCSwitch::nReceivedBitlength = 0;
volatile unsigned int RCSwitch::nReceivedDelay = 0;
volatile unsigned int RCSwitch::nReceivedProtocol = 0;
volatile unsigned int RCSwitch::nReceivedChecksum = 0;
volatile byte RCSwitch::nReceivedLength = 0;

//volatile static byte RCSwitch::nCheckSum;
//volatile static bool RCSwitch::nCheckSumMatch;
//volatile static int nReceiverPin;

// separationLimit: minimum microseconds between received codes, closer codes are ignored.
// according to discussion on issue #14 it might be more suitable to set the separation
// limit to the same time as the 'low' part of the sync signal for the current protocol.
const unsigned int VAR_ISR_ATTR RCSwitch::nSeparationLimit = 3500;	//4300;
unsigned int RCSwitch::timings[RCSWITCH_MAX_CHANGES];
unsigned int RCSwitch::timings2[255];
byte RCSwitch::mCodeData[ROWDATA_MAX_BYTES];
int RCSwitch::nReceiveTolerance = 60;
int RCSwitch::nReceiverPin;
unsigned long RCSwitch::nDeviceID;
byte RCSwitch::nSwitchID;
byte RCSwitch::nChannelID;
byte RCSwitch::nDeviceType;
float RCSwitch::nTemperature;
byte RCSwitch::nHumidity;
byte RCSwitch::nBattery;

#endif

// *******************************************
RCSwitch::RCSwitch() {
  
  this->nTransmitterPin = -1;
  this->nTransmitterPower = -1;
  this->setProtocol(1);
  this->setRepeatTransmit(4);				// 10
  
#if not defined( RCSwitchDisableReceiving )
  this->nReceiverPin = -1;
  this->nReceiverInterrupt = -1;
  this->setReceiveTolerance(60);
  RCSwitch::nReceivedValue = 0;
  //RCSwitch::nReceivedData = 0;
#endif
  
}

/**
  * Sets the protocol to send.
  */
void RCSwitch::setProtocol(Protocol protocol) {
  this->protocol = protocol;
}

/**
  * Sets the protocol to send, from a list of predefined protocols
  */
void RCSwitch::setProtocol(int nProtocol) {
  if (nProtocol < 1 || nProtocol > numProto) {
    nProtocol = 1;  // TODO: trigger an error, e.g. "bad protocol" ???
  }
#if defined(ESP8266) || defined(ESP32)
  this->protocol = proto[nProtocol-1];
#else
  memcpy_P(&this->protocol, &proto[nProtocol-1], sizeof(Protocol));
#endif
}

/**
  * Sets the protocol to send with pulse length in microseconds.
  */
void RCSwitch::setProtocol(int nProtocol, int nPulseLength) {
  setProtocol(nProtocol);
  this->setPulseLength(nPulseLength);
}

/**
  * Sets pulse length in microseconds
  */
void RCSwitch::setPulseLength(int nPulseLength) {
  this->protocol.pulseLength = nPulseLength;
}

/**
 * Sets Repeat Transmits
 */
void RCSwitch::setRepeatTransmit(int nRepeatTransmit) {
  this->nRepeatTransmit = nRepeatTransmit;
}

/**
 * Set Receiving Tolerance
 */
#if not defined( RCSwitchDisableReceiving )
void RCSwitch::setReceiveTolerance(int nPercent) {
  RCSwitch::nReceiveTolerance = nPercent;
}
#endif

/**
 * Enable transmissions
 *
 * @param nTransmitterPin    Arduino Pin to which the sender is connected to
 */
void RCSwitch::enableTransmit(int nTransmitterPin, int nTransmitterPower) {
  this->nTransmitterPin = nTransmitterPin;
  pinMode(this->nTransmitterPin, OUTPUT);
  this->nTransmitterPower = nTransmitterPower;
  pinMode(this->nTransmitterPower, OUTPUT);
}

/**
  * Disable transmissions
  */
void RCSwitch::disableTransmit() {
  this->nTransmitterPin = -1;
}

/**
 * Switch a remote switch on (Type D REV)
 *
 * @param sGroup        Code of the switch group (A,B,C,D)
 * @param nDevice       Number of the switch itself (1..3)
 */
void RCSwitch::switchOn(char sGroup, int nDevice) {
  this->sendTriState( this->getCodeWordD(sGroup, nDevice, true) );
}

/**
 * Switch a remote switch off (Type D REV)
 *
 * @param sGroup        Code of the switch group (A,B,C,D)
 * @param nDevice       Number of the switch itself (1..3)
 */
void RCSwitch::switchOff(char sGroup, int nDevice) {
  this->sendTriState( this->getCodeWordD(sGroup, nDevice, false) );
}

/**
 * Switch a remote switch on (Type C Intertechno)
 *
 * @param sFamily  Familycode (a..f)
 * @param nGroup   Number of group (1..4)
 * @param nDevice  Number of device (1..4)
  */
void RCSwitch::switchOn(char sFamily, int nGroup, int nDevice) {
  this->sendTriState( this->getCodeWordC(sFamily, nGroup, nDevice, true) );
}

/**
 * Switch a remote switch off (Type C Intertechno)
 *
 * @param sFamily  Familycode (a..f)
 * @param nGroup   Number of group (1..4)
 * @param nDevice  Number of device (1..4)
 */
void RCSwitch::switchOff(char sFamily, int nGroup, int nDevice) {
  this->sendTriState( this->getCodeWordC(sFamily, nGroup, nDevice, false) );
}

/**
 * Switch a remote switch on (Type B with two rotary/sliding switches)
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 */
void RCSwitch::switchOn(int nAddressCode, int nChannelCode) {
  this->sendTriState( this->getCodeWordB(nAddressCode, nChannelCode, true) );
}

/**
 * Switch a remote switch off (Type B with two rotary/sliding switches)
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 */
void RCSwitch::switchOff(int nAddressCode, int nChannelCode) {
  this->sendTriState( this->getCodeWordB(nAddressCode, nChannelCode, false) );
}

/**
 * Deprecated, use switchOn(const char* sGroup, const char* sDevice) instead!
 * Switch a remote switch on (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param nChannelCode  Number of the switch itself (1..5)
 */
void RCSwitch::switchOn(const char* sGroup, int nChannel) {
  const char* code[6] = { "00000", "10000", "01000", "00100", "00010", "00001" };
  this->switchOn(sGroup, code[nChannel]);
}

/**
 * Deprecated, use switchOff(const char* sGroup, const char* sDevice) instead!
 * Switch a remote switch off (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param nChannelCode  Number of the switch itself (1..5)
 */
void RCSwitch::switchOff(const char* sGroup, int nChannel) {
  const char* code[6] = { "00000", "10000", "01000", "00100", "00010", "00001" };
  this->switchOff(sGroup, code[nChannel]);
}

/**
 * Switch a remote switch on (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param sDevice       Code of the switch device (refers to DIP switches 6..10 (A..E) where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 */
void RCSwitch::switchOn(const char* sGroup, const char* sDevice) {
  this->sendTriState( this->getCodeWordA(sGroup, sDevice, true) );
}

/**
 * Switch a remote switch off (Type A with 10 pole DIP switches)
 *
 * @param sGroup        Code of the switch group (refers to DIP switches 1..5 where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 * @param sDevice       Code of the switch device (refers to DIP switches 6..10 (A..E) where "1" = on and "0" = off, if all DIP switches are on it's "11111")
 */
void RCSwitch::switchOff(const char* sGroup, const char* sDevice) {
  this->sendTriState( this->getCodeWordA(sGroup, sDevice, false) );
}


/**
 * Returns a char[13], representing the code word to be send.
 *
 */
char* RCSwitch::getCodeWordA(const char* sGroup, const char* sDevice, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  for (int i = 0; i < 5; i++) {
    sReturn[nReturnPos++] = (sGroup[i] == '0') ? 'F' : '0';
  }

  for (int i = 0; i < 5; i++) {
    sReturn[nReturnPos++] = (sDevice[i] == '0') ? 'F' : '0';
  }

  sReturn[nReturnPos++] = bStatus ? '0' : 'F';
  sReturn[nReturnPos++] = bStatus ? 'F' : '0';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * Encoding for type B switches with two rotary/sliding switches.
 *
 * The code word is a tristate word and with following bit pattern:
 *
 * +-----------------------------+-----------------------------+----------+------------+
 * | 4 bits address              | 4 bits address              | 3 bits   | 1 bit      |
 * | switch group                | switch number               | not used | on / off   |
 * | 1=0FFF 2=F0FF 3=FF0F 4=FFF0 | 1=0FFF 2=F0FF 3=FF0F 4=FFF0 | FFF      | on=F off=0 |
 * +-----------------------------+-----------------------------+----------+------------+
 *
 * @param nAddressCode  Number of the switch group (1..4)
 * @param nChannelCode  Number of the switch itself (1..4)
 * @param bStatus       Whether to switch on (true) or off (false)
 *
 * @return char[13], representing a tristate code word of length 12
 */
char* RCSwitch::getCodeWordB(int nAddressCode, int nChannelCode, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  if (nAddressCode < 1 || nAddressCode > 4 || nChannelCode < 1 || nChannelCode > 4) {
    return 0;
  }

  for (int i = 1; i <= 4; i++) {
    sReturn[nReturnPos++] = (nAddressCode == i) ? '0' : 'F';
  }

  for (int i = 1; i <= 4; i++) {
    sReturn[nReturnPos++] = (nChannelCode == i) ? '0' : 'F';
  }

  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = 'F';

  sReturn[nReturnPos++] = bStatus ? 'F' : '0';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * Like getCodeWord (Type C = Intertechno)
 */
char* RCSwitch::getCodeWordC(char sFamily, int nGroup, int nDevice, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  int nFamily = (int)sFamily - 'a';
  if ( nFamily < 0 || nFamily > 15 || nGroup < 1 || nGroup > 4 || nDevice < 1 || nDevice > 4) {
    return 0;
  }
  
  // encode the family into four bits
  sReturn[nReturnPos++] = (nFamily & 1) ? 'F' : '0';
  sReturn[nReturnPos++] = (nFamily & 2) ? 'F' : '0';
  sReturn[nReturnPos++] = (nFamily & 4) ? 'F' : '0';
  sReturn[nReturnPos++] = (nFamily & 8) ? 'F' : '0';

  // encode the device and group
  sReturn[nReturnPos++] = ((nDevice-1) & 1) ? 'F' : '0';
  sReturn[nReturnPos++] = ((nDevice-1) & 2) ? 'F' : '0';
  sReturn[nReturnPos++] = ((nGroup-1) & 1) ? 'F' : '0';
  sReturn[nReturnPos++] = ((nGroup-1) & 2) ? 'F' : '0';

  // encode the status code
  sReturn[nReturnPos++] = '0';
  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = 'F';
  sReturn[nReturnPos++] = bStatus ? 'F' : '0';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * Encoding for the REV Switch Type
 *
 * The code word is a tristate word and with following bit pattern:
 *
 * +-----------------------------+-------------------+----------+--------------+
 * | 4 bits address              | 3 bits address    | 3 bits   | 2 bits       |
 * | switch group                | device number     | not used | on / off     |
 * | A=1FFF B=F1FF C=FF1F D=FFF1 | 1=0FF 2=F0F 3=FF0 | 000      | on=10 off=01 |
 * +-----------------------------+-------------------+----------+--------------+
 *
 * Source: http://www.the-intruder.net/funksteckdosen-von-rev-uber-arduino-ansteuern/
 *
 * @param sGroup        Name of the switch group (A..D, resp. a..d) 
 * @param nDevice       Number of the switch itself (1..3)
 * @param bStatus       Whether to switch on (true) or off (false)
 *
 * @return char[13], representing a tristate code word of length 12
 */
char* RCSwitch::getCodeWordD(char sGroup, int nDevice, bool bStatus) {
  static char sReturn[13];
  int nReturnPos = 0;

  // sGroup must be one of the letters in "abcdABCD"
  int nGroup = (sGroup >= 'a') ? (int)sGroup - 'a' : (int)sGroup - 'A';
  if ( nGroup < 0 || nGroup > 3 || nDevice < 1 || nDevice > 3) {
    return 0;
  }

  for (int i = 0; i < 4; i++) {
    sReturn[nReturnPos++] = (nGroup == i) ? '1' : 'F';
  }

  for (int i = 1; i <= 3; i++) {
    sReturn[nReturnPos++] = (nDevice == i) ? '1' : 'F';
  }

  sReturn[nReturnPos++] = '0';
  sReturn[nReturnPos++] = '0';
  sReturn[nReturnPos++] = '0';

  sReturn[nReturnPos++] = bStatus ? '1' : '0';
  sReturn[nReturnPos++] = bStatus ? '0' : '1';

  sReturn[nReturnPos] = '\0';
  return sReturn;
}

/**
 * @param sCodeWord   a tristate code word consisting of the letter 0, 1, F
 */
void RCSwitch::sendTriState(const char* sCodeWord) {
  // turn the tristate code word into the corresponding bit pattern, then send it
  unsigned long code = 0;
  unsigned int length = 0;
  for (const char* p = sCodeWord; *p; p++) {
    code <<= 2L;
    switch (*p) {
      case '0':
        // bit pattern 00
        break;
      case 'F':
        // bit pattern 01
        code |= 1L;
        break;
      case '1':
        // bit pattern 11
        code |= 3L;
        break;
    }
    length += 2;
  }
  this->send(code, length);
}

/**
 * @param sCodeWord   a binary code word consisting of the letter 0, 1
 */
void RCSwitch::send(const char* sCodeWord) {
  // turn the tristate code word into the corresponding bit pattern, then send it
  unsigned long code = 0;
  unsigned int length = 0;
  for (const char* p = sCodeWord; *p; p++) {
    code <<= 1L;
    if (*p != '0')
      code |= 1L;
    length++;
  }
  this->send(code, length);
}

/**
 * Transmit the first 'length' bits of the integer 'code'. The
 * bits are sent from MSB to LSB, i.e., first the bit at position length-1,
 * then the bit at position length-2, and so on, till finally the bit at position 0.
 */
void RCSwitch::send(unsigned long code, unsigned int length) {
  if (this->nTransmitterPin == -1)
    return;

#if not defined( RCSwitchDisableReceiving )
  // make sure the receiver is disabled while we transmit
  int nReceiverInterrupt_backup = nReceiverInterrupt;
  if (nReceiverInterrupt_backup != -1) {
    this->disableReceive();
	digitalWrite(this->nTransmitterPower,HIGH);
  }
#endif

  for (int nRepeat = 0; nRepeat < nRepeatTransmit; nRepeat++) {
    
	this->transmit(protocol.syncFactor);
	for (int i = length-1; i >= 0; i--) {
      if (code & (1L << i))
        this->transmit(protocol.oneBit);
      else
        this->transmit(protocol.zeroBit);
    }
    //this->transmit(protocol.syncFactor);
  }

  // Disable transmit after sending (i.e., for inverted protocols)
  digitalWrite(this->nTransmitterPin, LOW);

#if not defined( RCSwitchDisableReceiving )
  // enable receiver again if we just disabled it
  if (nReceiverInterrupt_backup != -1) {
	digitalWrite(this->nTransmitterPower,LOW);
	this->nReceiverInterrupt = nReceiverInterrupt_backup;
	this->enableReceive();
  }
#endif
}

/**
 * Transmit a single high-low pulse.
 */
void RCSwitch::transmit(HighLow pulses) {
  
  uint8_t firstLogicLevel = (this->protocol.invertedSignal) ? LOW : HIGH;
  uint8_t secondLogicLevel = (this->protocol.invertedSignal) ? HIGH : LOW;
  
  digitalWrite(this->nTransmitterPin, firstLogicLevel);
  delayMicroseconds(this->protocol.pulseLength * pulses.high);
  digitalWrite(this->nTransmitterPin, secondLogicLevel);
  delayMicroseconds(this->protocol.pulseLength * pulses.low);
  
}

#if not defined( RCSwitchDisableReceiving )
/**
 * Enable receiving data
 */
void RCSwitch::enableReceive(int digitalPin) {
  RCSwitch::nReceiverPin = digitalPin;
  this->nReceiverInterrupt = digitalPinToInterrupt(digitalPin);			//interrupt;
  this->enableReceive();
}

void RCSwitch::enableReceive() {
  if (this->nReceiverInterrupt != -1) {
    RCSwitch::nReceivedValue = 0;
    RCSwitch::nReceivedBitlength = 0;
#if defined(RaspberryPi)
    wiringPiISR(this->nReceiverInterrupt, INT_EDGE_BOTH, &handleInterrupt);	// Raspberry Pi
#else
    attachInterrupt(this->nReceiverInterrupt, handleInterrupt, CHANGE);		// Arduino
#endif
  }
}

/**
 * Disable receiving data
 */
void RCSwitch::disableReceive() {
#if not defined(RaspberryPi) 					// Arduino
  detachInterrupt(this->nReceiverInterrupt);
#endif 											// For Raspberry Pi (wiringPi) you can't unregister the ISR
  this->nReceiverInterrupt = -1;
}

bool RCSwitch::available() {
  return ((RCSwitch::nReceivedValue != 0) || (RCSwitch::nReceivedLength != 0));
}

void RCSwitch::resetAvailable() {
  RCSwitch::nReceivedValue = 0;
  RCSwitch::nReceivedLength = 0;
}

unsigned int RCSwitch::getReceivedChecksum() {
  return RCSwitch::nReceivedChecksum;
}

unsigned long RCSwitch::getReceivedValue() {
  return RCSwitch::nReceivedValue;
}

unsigned int RCSwitch::getReceivedBitlength() {
  return RCSwitch::nReceivedBitlength;
}

unsigned int RCSwitch::getReceivedDelay() {
  return RCSwitch::nReceivedDelay;
}

unsigned int RCSwitch::getReceivedProtocol() {
  return RCSwitch::nReceivedProtocol;
}

unsigned int* RCSwitch::getReceivedTimings() {
  return RCSwitch::timings;
}

byte* RCSwitch::getReceivedData() {
  return RCSwitch::mCodeData;
}

byte RCSwitch::getReceivedLength() {
  return RCSwitch::nReceivedLength;
}

unsigned long RCSwitch::getDeviceID() {
  return RCSwitch::nDeviceID;
}

byte RCSwitch::getSwitchID() {
  return RCSwitch::nSwitchID;
}

byte RCSwitch::getChannelID() {
  return RCSwitch::nChannelID;
}

byte RCSwitch::getDeviceType() {
  return RCSwitch::nDeviceType;
}

float RCSwitch::getTemperature() {
    return RCSwitch::nTemperature;
}

byte RCSwitch::getHumidity() {
	return RCSwitch::nHumidity;
}

byte RCSwitch::getBattery() {
	return RCSwitch::nBattery;
}

/* unsigned int RCSwitch::getCheckSum() {
  return RCSwitch::nCheckSum;
} */

/* bool RCSwitch::isCheckSumMatch() {
	return RCSwitch::nCheckSumMatch;
} */

/* helper function for the receiveProtocol method */
static inline unsigned int diff(int A, int B) {
  return abs(A - B);
}

unsigned int bitReverse(unsigned int number, byte lengh) {
  unsigned int result = 0;
  for(byte i = 0; i < lengh; i++) {
    result <<= 1;
    result |= number & 1;
    number >>= 1;
  }
  return result;
}

/* For protocols that start low, the sync period looks like
 *               _________
 * _____________|         |XXXXXXXXXXXX|	- inverted
 *
 * |--1st dur--|-2nd dur-|-Start data-|
 *
 * The 3rd saved duration starts the data.
 *
 * For protocols that start high, the sync period looks like
 *
 *  ______________
 * |              |____________|XXXXXXXXXXXXX|
 *
 * |-filtered out-|--1st dur--|--Start data--|
 *
 * The 2nd saved duration starts the data
 */

// ***** Receive protocol *****
bool RECEIVE_ATTR RCSwitch::receiveProtocol(const int p, unsigned int changeCount) {

#if defined(ESP8266) || defined(ESP32)
    const Protocol &pro = proto[p-1];
#else
    Protocol pro;		// struct Protocol
    memcpy_P(&pro, &proto[p-1], sizeof(Protocol));
#endif

    static unsigned long lastTime = 0;
	unsigned long data = 0;			// for 24 bit data (max 4 bytes (8 nibbles))
	byte checkSum = 0;				// for 36 bit data (9 nibble = checkSum)
    
	//Assuming the longer pulse length is the pulse captured in timings[0]
    const unsigned int syncLengthInPulses = ((pro.syncFactor.low) > (pro.syncFactor.high)) ? (pro.syncFactor.low) : (pro.syncFactor.high);
    const unsigned int pulseLength = RCSwitch::timings[0] / syncLengthInPulses;		// calculated pulse length
    const unsigned int delayTolerance = pulseLength * RCSwitch::nReceiveTolerance / 100;
    const unsigned int firstDataTiming = (pro.invertedSignal) ? (2) : (1);

    // read data from timings received
	for (unsigned int i = firstDataTiming; i < changeCount - 1; i += 2) {
        //
		if (i < 64+firstDataTiming)			// only 8 nibbles = 4 bytes
			data <<= 1;
		else 
			checkSum <<= 1;					// 9th nibble = checksum
		//
		if (diff(RCSwitch::timings[i], pulseLength * pro.zeroBit.high) < delayTolerance &&
			diff(RCSwitch::timings[i + 1], pulseLength * pro.zeroBit.low) < delayTolerance) {
			// zero bit
		} 
		else if (diff(RCSwitch::timings[i], pulseLength * pro.oneBit.high) < delayTolerance &&
				   diff(RCSwitch::timings[i + 1], pulseLength * pro.oneBit.low) < delayTolerance) {
			// one bit
			if (i < 64+firstDataTiming)			// only 8 nibbles = 4 bytes
				data |= 1;
			else 
				checkSum |= 1;					// 9th nibble = checksum
		} 
		else {
			// Failed reading data for current protocol
			return false;
		}
    }
	
    if (changeCount > 7) {    				// ignore very short transmissions: no device sends them, so this must be noise
        
		if ((millis() - lastTime) > 350) {
			RCSwitch::nReceivedValue = data;
		}
		
		RCSwitch::nReceivedBitlength = (changeCount - 1) / 2;
        RCSwitch::nReceivedDelay = pulseLength;
        RCSwitch::nReceivedProtocol = p;
		
		if ((p > 0) && (p < 13)) {
			RCSwitch::nDeviceID = data >> 4;
			RCSwitch::nSwitchID = data & 0x0F;
		}
		
		if (changeCount > 64) {
			RCSwitch::nReceivedChecksum = checkSum;		// 9th nibble (0x0F)
		}
		
		if (p == 13) {					// for protocol = 13 (BL999 thermo/hydro sensor) & 14
		
			// checkSum = sum of nibbles
			byte sum = 0;
			unsigned long data1 = data;
			for (int i=0; i<8; i++) {
				sum += bitReverse(data1 & 0xF, 4);
				data1 >>= 4;
			}
			sum &= 0xF;
			sum = bitReverse(sum, 4);
			//RCSwitch::nCheckSum = checkSum;
			//RCSwitch::nCheckSumMatch = (sum == checkSum);
			
			if (sum == checkSum) {
			
				RCSwitch::nDeviceID = (data >> 24) & 0xF3;			// device ID (mask channel ID)
				RCSwitch::nChannelID = (data >> 26) & 0x03;			// channel ID
				RCSwitch::nBattery = ((data >> 23) & 0x01) ^ 1;		// battery low flag (inversed, i.e. 0 = low battery)
				bool isValidData = ((data >> 21) & 0x03) != 3;		// B11 = Non temperature/humidity data
				//RCSwitch::unknown = (data >> 20) & 0x3;			// unknown
				
				if (!isValidData)
					return false;
				
				//Temperature is stored in T4,T5,T6 nibbles, lowest nibble - first
				//since we already reversed bits order in these nibbles
				//all we have to do is to reverse nibbles order
				
				//temperature = (((int)bl999_data[5] << 8) | ((int)bl999_data[4] << 4) | (int)bl999_data[3]);

				int temperature = (data >> 8) & 0xFFF;
				temperature = bitReverse(temperature,12);
							   
				//if ((bl999_data[5] & 1) == 1) {
				if ((temperature & 0x800)) {
					//negative number, use two's compliment conversion
					temperature = ~temperature + 1;
					//clear higher bits and convert to negative
					temperature = -1 * (temperature & 0xFFF);
				}

				RCSwitch::nTemperature = temperature/10.0;
				
				//Humidity is stored in nibbles T7,T8
				//since bits in these nibbles are already in reversed order
				//we just have to get number by reversing nibbles orderßßß
				
				//int humidity = ((int)bl999_data[7] << 4) | (int)bl999_data[6];
				byte humidity = data & 0xFF;

				if (humidity != 0xFF) {
					//negative number, use two's compliment conversion
					humidity = ~humidity + 1;
					//humidity is stored as 100 - humidity
					RCSwitch::nHumidity = 100 - (byte)humidity;
				}
				else
					RCSwitch::nHumidity = humidity;
			
			}
		}
		
		if (p == 14) {
			RCSwitch::nDeviceID = (data >> 24) & 0xFF;			// device ID
			RCSwitch::nChannelID = (data >> 20) & 0x03;			// channel ID (0,1,2)
			RCSwitch::nBattery = (data >> 23) & 0x01;			// battery (1 = normal, 0 = low)
			// (data >> 22) & 0x01;								// ????? maybe 3d bit for channel ID
			// (data >> 4) & 0x0F;								// ?????
			int temperature = (data >> 8) & 0xFFF;
			RCSwitch::nTemperature = temperature/10.0;
			byte humidity = ((data << 4) | checkSum) & 0xFF;
			RCSwitch::nHumidity = humidity;
		}
		
		lastTime = millis();
		return true;
    }

    return false;
}

/* Encrypt data byte to send to station */ 
byte EncryptByte(byte b) { 
	byte a; 
	for(a = 0; b; b <<= 1)
		a ^= b;
	return a;
}

/* Decrypt raw received data byte */ 
byte RECEIVE_ATTR DecryptByte(byte b) { 
	return b ^ (b << 1); 
}

/* The second checksum. Input is OldChecksum^NewByte */
byte SecondCheck(byte b) {
	byte c; 
	if (b&0x80) 
		b^=0x95; 
	c = b^(b>>1); 
	if (b&1) 
		c^=0x5f; 
	if (c&1) 
		b^=0x5f; 
	return b^(c>>1); 
}

/*
Decrypt and check a package, 
Input: Buffer holds the received raw data. 
Returns ERROR number, Buffer now holds decrypted data 
*/
#define NO_ERROR 	 0
#define ERROR_HEADER 1
#define ERROR_CS1 	 2
#define ERROR_CS2 	 3

byte RECEIVE_ATTR RCSwitch::DecryptAndCheck(byte *data) {
	byte cs1,cs2,count,i; 
	if (RCSwitch::mCodeData[0] != 0x75) 
		return ERROR_HEADER; 
	count = (DecryptByte(RCSwitch::mCodeData[2]) >> 1) & 0x1F; 
	cs1 = 0; 
	cs2 = 0; 
	for (i=1; i<count+2; i++) {
		cs1 ^= RCSwitch::mCodeData[i];
		cs2 = SecondCheck(RCSwitch::mCodeData[i]^cs2);
		RCSwitch::mCodeData[i] = DecryptByte(RCSwitch::mCodeData[i]);
	}
	if (cs1)
		return ERROR_CS1;
	if (cs2 != RCSwitch::mCodeData[count+2])
		return ERROR_CS2;
	return NO_ERROR;
}

byte RECEIVE_ATTR RCSwitch::mCodeCheckData(byte length) {		// for protocol = 0 (manchester encoded)
	
	static unsigned long lastTime = 0;
	
	byte error = DecryptAndCheck(RCSwitch::mCodeData);
	
	//RCSwitch::nCheckSumMatch = error;
	
	if (!error) {
		RCSwitch::nReceivedProtocol = 0;
		//RCSwitch::nReceivedData = RCSwitch::mCodeData;
		if ((millis() - lastTime) > 200) {
			RCSwitch::nReceivedLength = length;
		}
		// mCodeData[1] == 0x75;									// package header
		RCSwitch::nDeviceID = RCSwitch::mCodeData[1];				// Device ID
		RCSwitch::nChannelID = (RCSwitch::mCodeData[1] >> 6) + 1;	// Channel number
		// ************* channel number table *************
		//if ( data[1] > 0x1f && data[1] < 0x40) channel=1;	// thermo/hydro
		//if ( data[1] > 0x3f && data[1] < 0x60) channel=2;	// thermo/hydro
		//if ( data[1] > 0x5f && data[1] < 0x80) channel=3;	// thermo/hydro
		//if ( data[1] > 0x7f && data[1] < 0xa0) channel=1; 	// Anemometer/rainmeter and uvsensor - no channel settings
		//if ( data[1] > 0x9f && data[1] < 0xc0) channel=4;	// thermo/hydro
		//if ( data[1] > 0xbf && data[1] < 0xE0) channel=5;	// thermo/hydro
		RCSwitch::nBattery = (RCSwitch::mCodeData[2] >> 6);			// Battery status: 0 - low battery
		byte packageNumber = (RCSwitch::mCodeData[3] >> 6) & 0x03; 	//package number
		RCSwitch::nDeviceType = RCSwitch::mCodeData[3] & 0x1F;		// Device Type (bits 4...0)
		// ********* various sensor types *******************
		//if (data[3] == 0x0c ) {                       // Anemometer
		//if (data[3] == 0x0d ) {                       // UV Sensor
		//if (data[3] == 0x0e ) {                       // Rain meter  // 9F 80 CC 4E 76 00 66 12 
		if (RCSwitch::nDeviceType == 0x1E) {                      // Thermo/Hygro
			//~((mCodeData[5] >> 8) & 0x1)*(-1)					// temp sign
			//((mCodeData[5]) & 0xF)							// temp first digit
			//((mCodeData[4] >> 4) & 0xF)						// temp units digit
			//((mCodeData[4]) & 0xF)							// temp tenths digit
			double temperature = ((RCSwitch::mCodeData[5]) & 0xF)*10 + ((RCSwitch::mCodeData[4] >> 4) & 0xF) + ((RCSwitch::mCodeData[4]) & 0xF)/10.0;;
			if ((RCSwitch::mCodeData[5] & 0x80) != 0x80)
				temperature = -1 * temperature;
			RCSwitch::nTemperature = temperature;
			RCSwitch::nHumidity = ((RCSwitch::mCodeData[6] >> 4) & 0xF)*10 + ((RCSwitch::mCodeData[6]) & 0xF);
		}
	}
	
	lastTime = millis();
	//return error;
}

// handle Interrupt on CHANGE
void RECEIVE_ATTR RCSwitch::handleInterrupt() {			// recieved transition

	//byte value = digitalRead(RCSwitch::nReceiverPin);
	//Serial.printf("Interrupt received: %d\n",value);

  static unsigned int changeCount = 0;
  static unsigned int changeCount2 = 0;
  static unsigned int bitCount = 0;
  static unsigned int byteCount = 0;
  static unsigned long lastTime = 0;
  static unsigned long mCodeTime = 0;
  static unsigned int repeatCount = 0;
  //static unsigned int pulseLengh = 0;
  static byte mCodeBytes = 0;
  static bool mCode = true;
  static byte parity = 0;
  
  //Timer1.initialize(50000);		// 50 msec
  //Timer1.attachInterrupt(handleTimer);
  
  const long time = micros();
  const unsigned int duration = time - lastTime;
  const unsigned int duration2 = time - mCodeTime;

  //if (duration < 100) return;
  
/*   	if (changeCount == 0) {
		mCodeTime = time;
		Serial.println(">");
	} */
  
  // reading mCode timings
  if ((changeCount > 0) && (mCode)) {
	  
	if ((duration < 30) || (duration2 > 1250)) {
		mCode = false;
	}
	
	// 
	if ((duration2 > 650) && (duration2 < 1250) && (duration > 250)) {		// skip reading single shot pulse
		
/* 		Serial.print(changeCount);
		Serial.print(">");
		Serial.print(time - mCodeTime);
		Serial.print(";"); */
		
		if (bitCount < 8) {
		
			RCSwitch::mCodeData[byteCount] >>= 1;					// read bits in reverse order !!!
			byte bitValue = digitalRead(RCSwitch::nReceiverPin);	// read current bit value
			if (!bitValue) {
				RCSwitch::mCodeData[byteCount] |= 0x80;				// read bit inversed
				parity = parity ^1;
			}
			
			bitCount++;
			
		}
		else {
			byte bitValue = digitalRead(RCSwitch::nReceiverPin);	// read parity bit value
			if (!bitValue != parity) {
				// byte parity error
			}
			if (byteCount == 0) {							// finish recieving 1 data byte
				if (RCSwitch::mCodeData[0] != 0x75) {		// is not a Cresta protocol
					mCode = false;
				}
			}
			if (byteCount == 2) {
				mCodeBytes = ((DecryptByte(RCSwitch::mCodeData[2]) >> 1) & 0x1F);	// read package length
			}
			bitCount = 0;
			parity = 0;
			byteCount++;
		}
		
		if (byteCount == mCodeBytes + 3) {
			
			// debug print
/* 			Serial.print(byteCount);
			Serial.print(">>>");
			for (byte i=0; i<byteCount; i++) {
				Serial.print(RCSwitch::mCodeData[i],HEX);
				Serial.print(",");
			}
			Serial.println(); */
			
			mCodeCheckData(byteCount);					// go decrypt received data
			
			// reset counter
			changeCount = 0;
			changeCount2 = 0;
			
			//mCodeTime = 0;
			bitCount = 0;
			byteCount = 0;
		}
		
		RCSwitch::timings2[changeCount2++] = duration2;
		mCodeTime = time;
		
	}
	
  }	// end of reading mCode timings
  
  // pulse lenth > 4300 - reset counter 
  if (duration > RCSwitch::nSeparationLimit) {
    // A long stretch without signal level change occurred. 
	// This could be the gap between two transmission.
	
	// And it could be the first pulse in transmission.
	
	
/* 	Timer1.initialize(750);		// 0.75 msec
	Timer1.attachInterrupt(handleTimer);
	Timer1.start(); */
	
    if (diff(duration, RCSwitch::timings[0]) < 200) {		// detect repeat
      // This long signal is close in length to the long signal which started the previously recorded timings;
      // this suggests that it may indeed by a a gap between two transmissions 
      // (we assume here that a sender will send the signal multiple times with roughly the same gap between them).
      
	  repeatCount++;
      
	  if (repeatCount == 2) {
		  
		// debug print received timings
/* 		Serial.print(changeCount);
		Serial.print(">>");
		for (int i=0; i<changeCount; i++) {
			Serial.print(RCSwitch::timings[i]);
			Serial.print(",");
		}
		Serial.println(); */
		
		// let's try to define protocol on the 2nd transmission
        for(unsigned int i = 1; i <= numProto; i++) {
          if (receiveProtocol(i, changeCount)) {
            // receive succeeded for protocol <i>
            break;
          }
        }
		
        repeatCount = 0;
		
      }
	  
    }
	
	// debug print
/* 	if (changeCount2 > 89) {
		Serial.print(changeCount);
		Serial.print(">");
		for (int i=0; i<changeCount; i++) {
			Serial.print(RCSwitch::timings[i]);
			Serial.print(";");
		}
		Serial.println();
		//
		Serial.print(changeCount2);
		Serial.print(">>");
		for (int i=0; i<changeCount2; i++) {
			Serial.print(RCSwitch::timings2[i]);
			Serial.print(";");
		}
		Serial.println();
		
	} */
	
#ifdef RCSWITCH_DEBUG
 	Serial.print(duration);
	Serial.print("(");
	byte bitValue = digitalRead(RCSwitch::nReceiverPin);
	Serial.print(bitValue);
	Serial.println(")>");
#endif
	
	changeCount = 0;		// reset counter when pulse lenth > 4300
	
	changeCount2 = 0;
	mCode = true;
	mCodeTime = time;
	bitCount = 0;
	byteCount = 0;
	
  }

  // detect counter overflow - reset counter
  if (changeCount >= RCSWITCH_MAX_CHANGES) {
	
	//delayMicroseconds(1000);
	
	// debug print
/* 	Serial.print(changeCount);
	Serial.print(">");
	for (int i=0; i<changeCount; i++) {
		Serial.print(RCSwitch::timings[i]);
		Serial.print(",");
	}
	Serial.println(); */
	  
    changeCount = 0;
    repeatCount = 0;
  }

  RCSwitch::timings[changeCount] = duration;
  changeCount++;
  lastTime = time;
  
}

#endif
