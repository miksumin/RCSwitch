/*
  Example for receiving
  
  https://github.com/sui77/rc-switch/
  
  If you want to visualize a telegram copy the raw data and 
  paste it into http://test.sui.li/oszi/
*/

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
}

void loop() {
  if (mySwitch.available()) {
	  
	  unsigned long value = mySwitch.getReceivedValue();
	  unsigned int length = mySwitch.getReceivedBitlength();
	  unsigned int delay = mySwitch.getReceivedDelay();
	  unsigned int* raw = mySwitch.getReceivedRawdata();
	  unsigned int protocol = mySwitch.getReceivedProtocol();
	  byte deviceID = mySwitch.getDeviceID();
	  byte channelID = mySwitch.getChannelID();
	  float temperature = mySwitch.getTemperature();
	  byte humidity = mySwitch.getHumidity();
	  byte battery = mySwitch.getBattery();
	  
    output(value, length, delay, raw, protocol, deviceID, channelID, temperature, humidity, battery);	// output to serial
    
	mySwitch.resetAvailable();
  }
}

static const char* bin2tristate(const char* bin);
static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength);

void output(unsigned long value, unsigned int length, unsigned int delay, unsigned int* raw, unsigned int protocol, byte deviceID, byte channelID, float temperature, byte humidity, byte battery) {

  const char* b = dec2binWzerofill(value, length);
  Serial.print("Value: ");
  Serial.print(value);
  Serial.print(" (");
  Serial.print( length );
  Serial.print("Bit) Binary: ");
  Serial.print( b );
  Serial.print(" Tri-State: ");
  Serial.print( bin2tristate(b) );
  Serial.print(" PulseLength: ");
  Serial.print(delay);
  Serial.print(" microseconds");
  Serial.print(" Protocol: ");
  Serial.println(protocol);

  Serial.print("Raw data: ");
  for (unsigned int i=0; i<= length*2; i++) {
    Serial.print(raw[i]);
    Serial.print(",");
  }
  Serial.println();
  
  Serial.print("DeviceID: ");
  Serial.print(deviceID);
  Serial.print(" ChannelID: ");
  Serial.print(channelID);
  Serial.print(" Temperature: ");
  Serial.print(temperature);
  Serial.print(" Humidity: ");
  Serial.print(humidity);
  Serial.print(" Battery: ");
  Serial.print(battery);
  
  Serial.println();
}

static const char* bin2tristate(const char* bin) {
  static char returnValue[50];
  int pos = 0;
  int pos2 = 0;
  while (bin[pos]!='\0' && bin[pos+1]!='\0') {
    if (bin[pos]=='0' && bin[pos+1]=='0') {
      returnValue[pos2] = '0';
    } else if (bin[pos]=='1' && bin[pos+1]=='1') {
      returnValue[pos2] = '1';
    } else if (bin[pos]=='0' && bin[pos+1]=='1') {
      returnValue[pos2] = 'F';
    } else {
      return "not applicable";
    }
    pos = pos+2;
    pos2++;
  }
  returnValue[pos2] = '\0';
  return returnValue;
}

static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength) {
  static char bin[64]; 
  unsigned int i=0;

  while (Dec > 0) {
    bin[32+i++] = ((Dec & 1) > 0) ? '1' : '0';
    Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j< bitLength; j++) {
    if (j >= bitLength - i) {
      bin[j] = bin[ 31 + i - (j - (bitLength - i)) ];
    } else {
      bin[j] = '0';
    }
  }
  bin[bitLength] = '\0';
  
  return bin;
}


