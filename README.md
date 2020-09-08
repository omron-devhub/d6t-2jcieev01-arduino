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

| example | Sensor type                | board |
|:-------:|:---------------------------|:-----------------------|
| d6t-1a  | D6T-1A-01 / D6T-1A-02   (1x1)   | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |
| d6t-8l  | D6T-8L-09   (1x8)   | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |
| d6t-8lh  | D6T-8L-09H   (1x8) | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |
| d6t-44l | D6T-44L-06 / D6T-44L-06H   (4x4)   | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |
| d6t-32l | D6T-32L-01A   (32x32) | Arduino MKR-WiFi1010/ Adafruit Feather ESP32 |


## DEMO
sample output for D6T-32L

```
17:55:54.514 -> PTAT:35.4
17:55:54.514 -> 24.0,26.6,25.1,24.9,25.7,25.9,25.0,25.5,25.4,26.8,25.5,25.4,24.1,25.3,25.6,25.2,25.0,25.2,24.2,24.2,25.1,25.4,23.2,26.1,26.4,25.3,27.3,26.4,24.4,26.8,23.4,22.0 [degC]
17:55:54.514 -> 26.7,26.5,26.6,25.1,26.5,26.2,25.8,25.0,26.2,25.7,26.8,25.5,24.1,25.6,25.6,25.3,25.1,25.3,24.2,24.5,25.0,23.2,23.9,24.1,25.4,27.3,24.8,25.8,26.8,23.4,25.1,23.1 [degC]
17:55:54.514 -> 26.4,26.7,26.5,26.0,27.0,26.2,26.2,26.2,26.6,25.8,25.9,25.8,26.9,26.3,26.8,25.6,25.6,25.9,25.4,25.4,24.8,23.9,24.1,25.3,25.8,24.8,26.0,25.2,26.9,25.2,23.1,24.6 [degC]
17:55:54.549 -> 26.8,26.5,27.4,27.1,26.0,27.0,27.1,26.8,25.9,27.1,26.4,25.6,26.6,26.1,26.3,25.8,26.2,26.1,26.1,26.2,26.4,25.7,25.3,25.3,26.2,25.8,25.2,27.0,25.9,26.1,24.6,21.4 [degC]
17:55:54.549 -> 26.8,27.3,27.0,27.4,26.8,27.4,27.6,26.8,27.2,26.5,26.4,26.9,26.4,26.6,27.3,27.0,27.0,27.1,26.5,26.2,25.6,26.4,26.2,26.1,25.7,26.8,26.7,25.6,26.2,24.7,26.0,25.6 [degC]
17:55:54.549 -> 27.0,28.1,27.1,26.7,27.9,27.3,27.2,27.0,27.4,26.7,27.7,27.4,27.0,27.4,27.2,27.2,27.0,27.3,26.9,27.2,26.8,25.8,26.3,25.8,26.5,26.9,26.2,26.5,24.7,25.8,25.7,25.4 [degC]
17:55:54.549 -> 26.9,27.1,28.0,27.3,27.8,27.5,27.3,27.5,27.8,27.9,27.5,27.7,27.4,27.9,27.5,26.8,26.5,27.3,27.0,26.6,26.7,26.6,26.9,26.9,26.6,26.0,26.0,26.8,26.0,25.8,25.1,24.3 [degC]
17:55:54.583 -> 28.3,26.9,26.9,27.7,27.5,27.1,28.2,27.2,27.5,28.0,27.8,28.1,27.8,27.7,27.7,27.4,28.0,27.8,27.3,27.1,27.3,26.9,26.8,27.3,26.2,26.9,27.0,26.2,25.6,25.9,24.3,25.9 [degC]
17:55:54.583 -> 26.6,27.9,28.5,26.9,28.1,27.1,28.0,27.5,27.9,28.3,27.7,27.7,27.9,28.2,27.4,27.6,27.8,27.8,27.6,27.2,27.3,27.4,27.0,27.8,26.6,26.7,25.9,25.6,25.8,25.5,26.1,26.0 [degC]
17:55:54.583 -> 28.8,28.4,28.4,28.3,28.1,27.5,27.9,28.2,28.0,28.0,28.1,28.0,28.0,27.6,28.1,28.1,27.9,27.9,27.8,27.4,27.2,27.4,27.5,27.2,27.4,27.3,27.3,26.0,26.1,25.9,26.7,26.4 [degC]
17:55:54.583 -> 28.4,29.4,28.2,27.5,28.5,28.5,27.3,28.4,28.6,28.4,27.7,27.7,28.4,28.3,28.2,27.7,27.8,27.5,27.7,27.9,27.0,27.4,27.4,27.3,27.0,27.2,27.2,25.4,26.2,26.5,26.5,26.4 [degC]
17:55:54.583 -> 28.4,28.4,28.4,28.5,28.7,28.1,28.9,28.4,27.9,28.2,28.1,28.0,27.8,27.8,27.8,27.4,28.0,27.7,27.2,27.3,27.8,27.9,27.3,27.5,27.3,27.4,27.0,27.0,26.8,26.4,26.3,26.3 [degC]
17:55:54.618 -> 28.2,28.2,28.4,28.3,28.6,28.7,28.7,28.5,29.0,28.4,28.5,27.6,28.2,28.3,27.9,27.8,27.9,27.8,27.8,27.8,27.5,28.0,27.6,26.9,27.7,27.3,26.6,26.9,27.4,26.4,26.5,26.6 [degC]
17:55:54.618 -> 28.4,28.5,29.1,29.0,28.7,28.8,28.8,28.8,28.9,28.4,28.6,28.4,28.0,27.7,28.2,28.6,27.9,27.8,28.2,27.9,27.8,27.8,27.8,28.2,28.1,27.2,27.5,27.1,26.4,27.2,26.8,26.6 [degC]
17:55:54.618 -> 28.7,28.8,29.1,28.7,28.8,29.1,28.7,29.2,28.4,28.3,28.4,28.4,28.5,28.4,28.2,28.3,27.9,27.8,28.1,28.2,27.4,27.7,27.8,28.1,27.7,27.7,27.8,27.0,27.1,26.9,26.2,26.0 [degC]
17:55:54.618 -> 28.6,28.6,28.6,28.8,28.5,28.8,29.2,28.6,28.5,29.0,28.5,28.3,28.6,28.4,28.6,28.4,28.4,28.3,28.1,27.9,27.9,28.0,27.5,27.8,27.5,27.1,27.9,27.4,27.0,26.9,26.4,26.3 [degC]
17:55:54.653 -> 28.6,28.8,29.3,28.9,29.0,28.8,28.6,29.0,28.8,28.6,28.5,29.0,28.3,28.6,28.2,28.1,28.5,28.0,28.4,28.6,28.6,28.1,27.8,27.9,27.4,27.3,27.5,27.1,27.7,26.6,26.6,26.6 [degC]
17:55:54.653 -> 28.4,28.5,28.6,29.1,29.0,29.2,28.6,28.8,28.7,28.6,28.8,28.6,28.7,28.7,28.5,28.6,28.5,28.3,28.1,28.3,27.9,28.4,28.0,28.1,28.3,27.8,27.5,27.4,26.1,26.7,26.0,25.8 [degC]
17:55:54.653 -> 28.5,28.7,29.4,28.5,28.7,29.3,29.1,29.6,28.8,28.8,29.0,28.7,28.7,28.4,28.6,28.8,28.3,28.3,28.6,28.3,28.5,27.8,28.0,28.0,28.2,27.6,27.5,27.4,27.2,26.5,26.1,26.0 [degC]
17:55:54.653 -> 28.3,28.3,28.6,28.4,28.8,28.6,29.7,28.7,28.9,29.5,29.2,29.1,28.6,28.2,28.8,28.5,28.6,28.9,28.8,28.7,29.0,28.6,28.4,27.7,28.1,28.1,27.5,27.2,27.7,26.8,26.0,25.8 [degC]
17:55:54.687 -> 28.0,28.2,29.0,28.8,29.0,28.5,29.0,29.4,28.8,29.1,29.3,29.5,29.0,28.9,28.8,28.9,28.7,28.7,28.7,28.6,28.9,28.0,28.3,28.2,27.6,27.9,27.7,26.5,27.2,26.3,26.1,26.0 [degC]
17:55:54.687 -> 28.5,28.0,28.6,28.9,29.0,28.8,28.3,28.8,29.5,28.5,29.2,29.4,29.6,28.8,28.8,29.0,28.9,28.7,29.0,28.6,28.3,28.4,28.5,28.3,28.4,28.2,27.7,27.0,27.7,26.9,26.1,26.7 [degC]
17:55:54.687 -> 28.0,28.7,29.5,28.9,29.2,28.8,28.9,29.5,28.5,29.4,29.3,29.1,28.9,28.6,29.4,29.2,29.2,28.9,28.8,28.0,28.3,28.8,28.9,28.3,28.5,28.3,27.4,27.4,26.5,27.1,27.1,26.2 [degC]
17:55:54.687 -> 28.4,29.1,29.0,28.5,28.9,28.7,28.8,29.0,29.3,29.3,28.9,29.2,28.9,28.8,28.7,28.9,29.2,29.0,29.2,28.9,28.9,28.7,28.0,28.7,28.0,27.6,28.1,27.0,26.8,26.9,27.5,26.7 [degC]
17:55:54.722 -> 28.8,28.9,28.9,28.8,28.0,28.5,29.1,28.8,29.0,28.5,29.2,28.9,29.3,29.0,28.4,29.0,28.7,28.7,28.7,28.5,28.8,28.4,28.0,28.4,28.1,28.1,27.0,27.3,27.3,27.2,26.5,26.7 [degC]
17:55:54.722 -> 28.9,28.2,28.7,28.4,28.7,29.2,28.9,29.0,29.4,28.4,29.3,28.9,29.3,28.8,28.9,28.7,29.2,28.5,28.3,28.6,29.0,28.7,28.5,28.1,28.8,27.4,27.7,27.5,27.8,27.1,26.9,26.5 [degC]
17:55:54.722 -> 28.4,28.8,28.3,28.6,29.2,28.8,28.3,29.1,28.3,28.7,28.9,29.3,28.7,28.9,29.1,28.7,28.9,28.6,28.4,26.9,27.6,28.3,28.2,28.8,28.5,27.6,27.9,27.9,27.1,26.9,27.0,26.8 [degC]
17:55:54.722 -> 28.6,28.1,28.6,28.2,29.1,28.7,29.4,28.3,29.1,28.7,28.7,28.6,29.0,28.1,29.3,28.8,28.3,28.7,29.0,28.8,28.6,28.3,28.1,27.9,27.8,28.2,27.6,27.1,27.3,27.1,27.1,26.1 [degC]
17:55:54.757 -> 27.3,28.7,28.2,29.0,28.7,29.4,28.3,28.8,28.8,28.9,28.3,29.0,28.8,28.2,28.5,28.1,28.6,28.5,28.5,28.9,28.6,28.1,27.8,27.9,28.2,28.4,28.2,27.5,27.3,27.5,27.2,26.8 [degC]
17:55:54.757 -> 28.7,27.9,28.8,28.5,29.2,28.8,28.9,28.7,28.8,28.3,28.9,28.1,29.1,28.5,28.5,28.2,28.7,28.6,28.3,28.5,28.0,28.8,27.6,28.2,28.0,27.6,27.2,28.4,27.5,26.7,27.0,27.2 [degC]
17:55:54.757 -> 27.9,28.6,28.9,29.3,28.6,29.0,28.5,28.9,28.3,29.0,28.1,28.2,27.8,28.1,28.4,28.5,29.1,27.7,28.0,27.7,28.6,28.4,28.7,27.9,27.8,28.0,27.8,27.5,26.9,27.6,26.6,27.0 [degC]
17:55:54.757 -> 28.0,28.9,29.2,28.5,28.9,28.6,28.9,28.3,28.7,28.1,28.9,28.2,27.8,28.0,28.4,28.6,29.2,27.5,27.9,27.5,28.8,28.5,28.2,27.5,27.9,27.8,28.0,27.0,26.6,26.9,27.6,26.3 [degC]
```


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
- [RaspberryPi sample for D6T on 2JCIE-01-RP1](https://github.com/omron-devhub/d6t-2jcieev01-raspberrypi)
- [Arduino sample for D6F on 2JCIE-01-AR1/FT1](https://github.com/omron-devhub/d6f-2jcieev01-arduino)
- [RaspberryPi sample for D6F on 2JCIE-01-RP1](https://github.com/omron-devhub/d6f-2jcieev01-raspberrypi)
- [Arduino sample for B5W on 2JCIE-01-AR1/FT1](https://github.com/omron-devhub/b5w-2jcieev01-arduino)

projects by another authors.

- [d6t-grove-tinkerboard project](https://github.com/omron-devhub/d6t-grove-tinkerboard)
- [d6t-grove-m5stack project](https://github.com/omron-devhub/d6t-grove-m5stack)
- [d6t-grove-arduino project](https://github.com/omron-devhub/d6t-grove-arduino)  
    only for element 4x4 type, but libralized.


## Licence
Copyright (c) OMRON Corporation. All rights reserved.

Licensed under the MIT License.

