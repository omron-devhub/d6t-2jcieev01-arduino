# d6t-2jcieev01-arduino
It is a sample projects for D6T OMRON MEMS Thermal Sensors with
evaluation kit **2JCIE-EV01-AR1**,
**2JCIE-EV01-FT1** and some Arduino boards.

D6T sensor series are High Sensitivity Enables Detection
of Stationary Human Presence,

- OMRON's unique MEMS and ASIC technology achieve a high SNR.
- Superior noise immunity with a digital output.
- High-precision area temperature detection with low cross-talk field of
    view characteristics

## Description
this Arduino sample projects for acquiring data from sensors on 2JCIE-EV01.
sample projects output the sensor data to USB-Serial ports.

| example | description                | baord |
|:-------:|:---------------------------|:-----------------------|
| dt6-1a  | for the element type 1x1   | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |
| dt6-8l  | for the element type 1x8   | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |
| dt6-44l | for the element type 4x4   | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |
| dt6-32l | for the element type 32x32 | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |

## DEMO
T.B.D (console output)

![Console output for D6T](console output)

## Installation
see `https://www.arduino.cc/en/guide/libraries`

### Install from Arduino IDE
1. download .zip from this repo [releases](releases)
    or [master](archive/master.zip) .
2. Import the zip from Arduino IDE

    ![install-ide-import-lib](https://user-images.githubusercontent.com/48547675/55043017-9a34e980-5077-11e9-885d-03f9f82e3491.JPG)

    ![install-select-zip](https://user-images.githubusercontent.com/48547675/55043034-a7ea6f00-5077-11e9-99d5-26423fb652b5.JPG)

3. Then, you can see the samples in `File >> Examples` menu.

    ![install-select-examples](https://user-images.githubusercontent.com/48547675/55043028-a28d2480-5077-11e9-8365-6745cda417ff.JPG)

4. Select examples for your favorite sensors, build and program to boards.

### Manual install
1. download this repo

    ```shell
    $ git clone https://github.com/omron-devhub/d6t-2jcieev01-arduino
    ```

2. launch Arduino-IDE and select our sketch to load.
3. build and program to boards.


## Dependencies
None

## Links
- [Arduino samples for 2JCIE-01-AR1/FT1](https://github.com/omron-devhub/2jcieev01-arduino)
- [RaspberryPi samples for 2JCIE-01-RP1](https://github.com/omron-devhub/2jcieev01-raspberrypi)


## Licence
Copyright (c) OMRON Corporation. All rights reserved.

Licensed under the MIT License.

