/*
 * MIT License
 * Copyright (c) 2019, 2018 - present OMRON Corporation
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/* includes */
#include <Arduino.h>
#include <OmronD6T.h>

OmronD6T d6t( OmronD6T::D6T_44L );

/** <!-- setup {{{1 -->
 * 1. initialize a Serial port for output.
 * 2. initialize an I2C peripheral.
 */
void setup() {
    Serial.begin( 115200 );  // Serial baudrate = 115200bps
    Wire.begin();  // i2c master
    d6t.begin();
}


/** <!-- loop - Thermal sensor {{{1 -->
 * 1. read sensor.
 * 2. output results, format is: [degC]
 */
void loop() {
    d6t.read();
    // 1st data is PTAT measurement (: Proportional To Absolute Temperature)
    Serial.print( "PTAT:" );
    Serial.println( d6t.ambientTempC(), 1 );

    // loop temperature pixels of each thrmopiles measurements
    for (size_t row = 0, rows = d6t.rows(); row < rows; row++) {
        Serial.print( d6t.objectTempF( 0, row ), 1 );  // print PTAT & Temperature
        for (size_t col = 1, cols = d6t.cols(); col < cols; col++) {
            Serial.print( "," );   // print delimiter
            Serial.print( d6t.objectTempF( col, row ), 1 );  // print PTAT & Temperature
        }
        Serial.println( " [degC]" );  // wrap text at ROW end.
    }
    delay( 1000 );
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
