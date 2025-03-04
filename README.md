# Based on the Adafruit RA8875 [![Build Status](https://github.com/adafruit/Adafruit_RA8875/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_RA8875/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_RA8875/html/index.html)

This is a library for the Adafruit RA8875 driver board 40-pin TFT Touch Displays - 800x480 Max
It will work with your own boards using the RA8875 TFT controller as well.

Written by Limor Fried/Ladyada  for Adafruit Industries. BSD license, check license.txt for more information. 

This is a derived work that is a more portable, generic C code implementation of the C++ library from Adafruit
which is targeting the Arduino IDE. This is re-implemented as a C code library in which you instantiate an
"instance" of the TFT display type, and each operation is performed on that type by passing the pointer to 
the instance to the function. While this is inconvenient, it's portable to systems not using the Arduino IDE.

Original copyright and license is included in this repository in accordance with BSD Licensing requirements.
