# Adafruit LSM9DS0 Library [![Build Status](https://github.com/adafruit/Adafruit_LSM9DS0_Library/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_LSM9DS0_Library/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_LSM9DS0_Library/html/index.html)

[<img src="https://cdn-shop.adafruit.com/970x728/2021-00.jpg" width="500px">](https://www.adafruit.com/products/2021)


This is the Arduino library for the Adafruit triple-axis accelerometer/magnetometer/gyroscope LSM9DS0 breakouts

Designed specifically to work with the Adafruit LSM9DS0 Breakout & Flora Sensor
 * https://www.adafruit.com/products/2020
 * https://www.adafruit.com/products/2021

**note** both of the above sensors are discontinued. Check out the update, the [LSM9DS1 breakout](https://www.adafruit.com/product/3387) and [and its corresponding library](https://github.com/adafruit/Adafruit_LSM9DS1)


Check out the links above for our tutorials and wiring diagrams

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

# Installation
To install, use the Arduino Library Manager and search for "Adafruit LSM9DS0 Library" and install the library.

## Dependencies
 * [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
 * [Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor)
 * [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)

# Contributing

Contributions are welcome! Please read our [Code of Conduct](https://github.com/adafruit/Adafruit_LSM9DS0_Library/blob/master/CODE_OF_CONDUCT.md>)
before contributing to help this project stay welcoming.

## Documentation and doxygen
Documentation is produced by doxygen. Contributions should include documentation for any new code added.

Some examples of how to use doxygen can be found in these guide pages:

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips

## Formatting and clang-format
This library uses [`clang-format`](https://releases.llvm.org/download.html) to standardize the formatting of `.cpp` and `.h` files.
Contributions should be formatted using `clang-format`:

The `-i` flag will make the changes to the file.
```bash
clang-format -i *.cpp *.h
```
If you prefer to make the changes yourself, running `clang-format` without the `-i` flag will print out a formatted version of the file. You can save this to a file and diff it against the original to see the changes.

Note that the formatting output by `clang-format` is what the automated formatting checker will expect. Any diffs from this formatting will result in a failed build until they are addressed. Using the `-i` flag is highly recommended.

### clang-format resources
  * [Binary builds and source available on the LLVM downloads page](https://releases.llvm.org/download.html)
  * [Documentation and IDE integration](https://clang.llvm.org/docs/ClangFormat.html)

## About this Driver
Written by Kevin (KTOWN) Townsend for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution

