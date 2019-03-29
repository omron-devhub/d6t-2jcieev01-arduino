# 2jcieev01-arduino
It is a sample projects for OMRON sensor evaluation kit **2JCIE-EV01-AR1** and
**2JCIE-EV01-FT1** with some Arduino boards.

2JCIE-EV01 sensor evaluation kits are ...T.B.D...
(see 2SMPB-02E projects abstract)

## Description
Arduino sample projects for acquiring data from sensors on 2JCIE-EV01.
there samples output the sensor data to USB-Serial ports.

| example | description                     | baord |
|:-------:|:--------------------------------|:-----------------------|
| baro-ar | The Barometer sensor sample     | Arduino MKR-WiFi1010   |
| baro-ft | The Barometer sensor sample     | Adafruit Feather ESP32 |
| illm-ar | The Illuminance sensor sample   | Arduino MKR-WiFi1010   |
| illm-ft | The Illuminance sensor sample   | Adafruit Feather ESP32 |
| humi-ar | The Humidity sensor sample      | Arduino MKR-WiFi1010   |
| humi-ft | The Humidity sensor sample      | Adafruit Feather ESP32 |
| accl-ar | The Accelerometer sensor sample | Arduino MKR-WiFi1010   |
| accl-ft | The Accelerometer sensor sample | Adafruit Feather ESP32 |
| mmic-ar | The MEMS Microphone document    | Arduino MKR-WiFi1010   |
| mmic-ft | The MEMS Microphone document    | Adafruit Feather ESP32 |

## DEMO

![Console output 2SMPB](Graph_2SMPB.png)

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
    $ git clone https://github.com/omron-devhub/2jcieev01-arduino
    ```

2. launch Arduino-IDE and select our sketch to load.
3. build and program to boards.


## Dependencies
### MEMS Microphone in 2JCIE-EV01-AR1 (base baord Arduino MKR WiFi 1010)
* please use the example `InputSerialPlotter` from Arduino IDE,
  `Examples -> Examples for Arduino MKR WiFi 1010 -> I2S -> InputSerialPlotter`

### MEMS Microphone in 2JCIE-EV01-FT1 (base baord Adafruit Feather ESP32)
* please use the example `InputSerialPlotter` from
  `https://github.com/maspetsberger/esp32-i2s-mems`


## Licence
Copyright (c) OMRON Corporation. All rights reserved.

Licensed under the MIT License.

