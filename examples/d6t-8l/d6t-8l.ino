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
#include <Wire.h>

/* defines */
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02

#define N_ROW 8
#define N_PIXEL 8
#define N_READ ((N_PIXEL + 1) * 2 + 1)

uint8_t rbuf[N_READ];
double ptat;
double pix_data[N_PIXEL];

uint8_t calc_crc(uint8_t data) {
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

/** <!-- D6T_checkPEC {{{ 1--> D6T PEC(Packet Error Check) calculation.
 * calculate the data sequence,
 * from an I2C Read client address (8bit) to thermal data end.
 */
bool D6T_checkPEC(uint8_t buf[], int n) {
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret) {
        Serial.print("PEC check failed:");
        Serial.print(crc, HEX);
        Serial.print("(cal) vs ");
        Serial.print(buf[n], HEX);
        Serial.println("(get)");
    }
    return ret;
}

/** <!-- conv8us_s16_le {{{1 --> convert a 16bit data from the byte stream.
 */
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    uint16_t ret;
    ret = (uint16_t)buf[n];
    ret += ((uint16_t)buf[n + 1]) << 8;
    return (int16_t)ret;   // and convert negative.
}

/** <!-- setup {{{1 -->
 * 1. Initialize 
     - initialize a Serial port for output.
	   - initialize I2C.
	   - Send inisilize setting to D6T.
 */
void setup() {
    Serial.begin(115200);  // Serial baudrate = 115200bps
    Wire.begin();  // i2c master
	
    delay(20);	
    
	Wire.beginTransmission(D6T_ADDR); // I2C slave address
    Wire.write(0x02);                  // D6T register
    Wire.write(0x00);                  // Write Data
    Wire.write(0x01);                  // Write Data
    Wire.write(0xEE);                  // Write Data
    Wire.endTransmission();            // I2C Stop
    Wire.beginTransmission(D6T_ADDR);  // I2C slave address
    Wire.write(0x05);                  // D6T register
    Wire.write(0x90);                  // Write Data
    Wire.write(0x3A);                  // Write Data
    Wire.write(0xB8);                  // Write Data
    Wire.endTransmission();            // I2C Stop
    Wire.beginTransmission(D6T_ADDR);  // I2C slave address
    Wire.write(0x03);                  // D6T register
    Wire.write(0x00);                  // Write Data
    Wire.write(0x03);                  // Write Data
    Wire.write(0x8B);                  // Write Data
    Wire.endTransmission();            // I2C Stop
    Wire.beginTransmission(D6T_ADDR);  // I2C slave address
    Wire.write(0x03);                  // D6T register
    Wire.write(0x00);                  // Write Data
    Wire.write(0x07);                  // Write Data
    Wire.write(0x97);                  // Write Data
    Wire.endTransmission();            // I2C Stop
    Wire.beginTransmission(D6T_ADDR);  // I2C slave address
    Wire.write(0x02);                  // D6T register
    Wire.write(0x00);                  // Write Data
    Wire.write(0x00);                  // Write Data
    Wire.write(0xE9);                  // Write Data
    Wire.endTransmission();            // I2C Stop

    delay(500);	

}

/** <!-- loop - Thermal sensor {{{1 -->
 * 2. read data.
 */
void loop() {
    int i = 0;
	int16_t itemp = 0;
	
	// Read data via I2C
	// I2C buffer of "Arduino MKR" is 256 buffer. (It is enough)
    memset(rbuf, 0, N_READ);
    Wire.beginTransmission(D6T_ADDR);  // I2C slave address
    Wire.write(D6T_CMD);               // D6T register
    Wire.endTransmission();            
    Wire.requestFrom(D6T_ADDR, N_READ);
    while (Wire.available()) {
        rbuf[i++] = Wire.read();
    }
    D6T_checkPEC(rbuf, N_READ - 1);

    //Convert to temperature data (degC)
    ptat = (double)conv8us_s16_le(rbuf, 0) / 10.0;
	for (i = 0; i < N_PIXEL; i++) {
		itemp = conv8us_s16_le(rbuf, 2 + 2*i);
		pix_data[i] = (double)itemp / 10.0;
	}
    
    //Output results
	Serial.print("PTAT:");
    Serial.print(ptat, 1);
    Serial.print(" [degC], Temperature: ");
	for (i = 0; i < N_PIXEL; i++) {
	    Serial.print(pix_data[i], 1);
		Serial.print(", ");
	}	
    Serial.println(" [degC]");
	
    delay(250);	
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
