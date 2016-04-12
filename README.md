Adafruit LSM9DS0 Library
========================

This is a library for the Adafruit triple-axis accelerometer/magnetometer/gyroscope LSM9DS0 breakouts

Designed specifically to work with the Adafruit LSM9DS0 Breakout & Flora Sensor
  ----> https://www.adafruit.com/products/2020
  ----> https://www.adafruit.com/products/2021

These sensors use I2C to communicate, 2 pins are required to interface
Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Check out the links above for our tutorials and wiring diagrams 

Written by Kevin Townsend for Adafruit Industries.  
BSD license, all text above must be included in any redistribution

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder Adafruit_LSM9DS0. Check that the Adafruit_LSM9DS0 folder contains Adafruit_LSM9DS0.cpp and Adafruit_LSM9DS0.h

Place the Adafruit_LSM9DS0 library folder your *arduinosketchfolder*/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE.

<!-- START COMPATIBILITY TABLE -->

## Compatibility

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
Atmega328 @ 16MHz  |      X       |             |            | 
Atmega328 @ 12MHz  |      X       |             |            | 
Atmega32u4 @ 16MHz |      X       |             |            | 
Atmega32u4 @ 8MHz  |      X       |             |            | 
ESP8266            |      X       |             |            | 
Atmega2560 @ 16MHz |      X       |             |            | 
ATSAM3X8E          |      X       |             |            | 
ATSAM21D           |      X       |             |            | 
ATtiny85 @ 16MHz   |             |             |     X       | 
ATtiny85 @ 8MHz    |             |             |     X       | 
Intel Curie @ 32MHz |      X       |             |            | 
STM32F2            |             |             |     X       | 

  * ATmega328 @ 16MHz : Arduino UNO, Adafruit Pro Trinket 5V, Adafruit Metro 328, Adafruit Metro Mini
  * ATmega328 @ 12MHz : Adafruit Pro Trinket 3V
  * ATmega32u4 @ 16MHz : Arduino Leonardo, Arduino Micro, Arduino Yun, Teensy 2.0
  * ATmega32u4 @ 8MHz : Adafruit Flora, Bluefruit Micro
  * ESP8266 : Adafruit Huzzah
  * ATmega2560 @ 16MHz : Arduino Mega
  * ATSAM3X8E : Arduino Due
  * ATSAM21D : Arduino Zero, M0 Pro
  * ATtiny85 @ 16MHz : Adafruit Trinket 5V
  * ATtiny85 @ 8MHz : Adafruit Gemma, Arduino Gemma, Adafruit Trinket 3V

<!-- END COMPATIBILITY TABLE -->
