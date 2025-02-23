# rc-switch
[![Build Status](https://travis-ci.org/sui77/rc-switch.svg?branch=master)](https://travis-ci.org/sui77/rc-switch)

Use your Arduino or [Raspberry Pi](https://github.com/r10r/rcswitch-pi) to operate remote radio controlled devices

## Download
https://github.com/sui77/rc-switch/releases/latest

rc-switch is also listed in the arduino library manager.

## Wiki
https://github.com/sui77/rc-switch/wiki

## Info
### Send RC codes

Use your Arduino or Raspberry Pi to operate remote radio controlled devices.
This will most likely work with all popular low cost power outlet sockets. If
yours doesn't work, you might need to adjust the pulse length.

All you need is a Arduino or Raspberry Pi, a 315/433MHz AM transmitter and one
or more devices with one of the supported chipsets:

 - SC5262 / SC5272
 - HX2262 / HX2272
 - PT2262 / PT2272
 - EV1527 / RT1527 / FP1527 / HS1527 
 - Intertechno outlets
 - HT6P20X

### Receive and decode RC codes

Find out what codes your remote is sending. Use your remote to control your
Arduino.

All you need is an Arduino, a 315/433MHz AM receiver (altough there is no
instruction yet, yes it is possible to hack an existing device) and a remote
hand set.

For the Raspberry Pi, clone the https://github.com/ninjablocks/433Utils project to
compile a sniffer tool and transmission commands.

### Improvements

Added support for receiving and decoding Manchester encoded RF transitions for Cresta weather 
sensor protocol (https://kipdf.com/cresta-weather-sensor-protocol_5ae9eab67f8b9a40528b45e3.html), 
marked as protocol #0.

Added support for receiving and decoding BL999 thermo/hydro sensors (marked as protocol #13).
Example for using BL999 thermo/hydro sensor can be found in ReceiveDemo_BL999\ReceiveDemo_BL999.ino

Added support for receiving and decoding Garin WS-2 protocol (marked as protocol #14).
