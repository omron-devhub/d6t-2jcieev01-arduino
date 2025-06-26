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
#define D6T_CMD 0x4D  // for D6T-32L-01A, compensated output.
#define D6T_SET_ADD 0x01

#define N_ROW 32
#define N_PIXEL (32 * 32)
#define N_READ ((N_PIXEL + 1) * 2 + 1)

uint8_t rbuf[N_READ];
double ptat;
double pix_data[N_PIXEL];

/******* setting parameter *******/
#define D6T_IIR 0x00 
#define D6T_AVERAGE 0x04  
/*********************************/

#if defined(ARDUINO_FEATHER_ESP32)
    #define PIN_WIRE_SDA SDA
    #define PIN_WIRE_SCL SCL
#endif

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

// D6T-32L needs to be read 32x32x2 bytes at one-time from I2C,
// so Arduino's Wire can not be used.
// (Wire library buffers are 128 or 256 in MKR-WiFi1010/Feather ESP32.)
#define W 10  // I2C speed = 1 / (4 * N) [MHz]

typedef enum OC_ACKNACK {
    OC_ACK = 0,
    OC_NACK = 1
};

/** <!-- i2c_start {{{1 --> publish I2C start condition to the bus.
 */
void i2c_start() {
    // Serial.print("i2c_start...");
    digitalWrite(PIN_WIRE_SDA, LOW);
    delayMicroseconds(W * 4);
    digitalWrite(PIN_WIRE_SCL, LOW);
    delayMicroseconds(W * 4);
}

/** <!-- i2c_stop {{{1 --> publish I2C stop condition to the bus.
 */
void i2c_stop() {
    digitalWrite(PIN_WIRE_SDA, HIGH);
    delayMicroseconds(W * 4);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(W * 4);
    //Serial.println("i2c_stop");
}

/** <!-- i2c_write_8cycles {{{1 --> I2C write 8 bits to the bus.
 */
void i2c_write_8cycles(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        uint8_t v = (data & 0x80) != 0 ? HIGH: LOW;
        data <<= 1;

        digitalWrite(PIN_WIRE_SDA, v);
        delayMicroseconds(W);
        digitalWrite(PIN_WIRE_SCL, HIGH);
        delayMicroseconds(W * 2);
        digitalWrite(PIN_WIRE_SCL, LOW);
        delayMicroseconds(W);
    }
}

/** <!-- i2c_write_ack {{{1 --> check acknowledge among I2C writing.
 */
bool i2c_write_ack() {
    digitalWrite(PIN_WIRE_SDA, HIGH);
    pinMode(PIN_WIRE_SDA, INPUT);
    delayMicroseconds(W);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(W);
    uint8_t ret = digitalRead(PIN_WIRE_SDA);
    delayMicroseconds(W);
    digitalWrite(PIN_WIRE_SCL, LOW);
    delayMicroseconds(W * 10);
    pinMode(PIN_WIRE_SDA, OUTPUT);
    return ret == HIGH;  // check Nack
}

uint32_t i2c_write_reg8(uint8_t addr7, uint8_t* data, int length) {
    i2c_start();
    i2c_write_8cycles(addr7<<1);
    if (i2c_write_ack()) {
        Serial.print("Nack in write8_1");
        i2c_stop();
        return true;
    }
    for(int i = 0; i < length; i++){
        i2c_write_8cycles(data[i]);
        if (i2c_write_ack()) {
            Serial.print("Nack in write8_2");
            i2c_stop();
            return true;
        }       
    }
    digitalWrite(PIN_WIRE_SCL, HIGH);
    return false;
}

/** <!-- i2c_read_8cycles {{{1 --> I2C read 8 bits from the bus.
 */
uint8_t i2c_read_8cycles() {
    uint8_t ret = 0;

    pinMode(PIN_WIRE_SDA, INPUT);
    for (int i = 0; i < 8; i++) {
        delayMicroseconds(W);
        digitalWrite(PIN_WIRE_SCL, HIGH);
        delayMicroseconds(W);
        uint8_t b = digitalRead(PIN_WIRE_SDA);
        delayMicroseconds(W);
        digitalWrite(PIN_WIRE_SCL, LOW);
        delayMicroseconds(W);

        ret = (ret << 1) | (b == HIGH ? 1: 0);
    }
    pinMode(PIN_WIRE_SDA, OUTPUT);
    return ret;
}

/** <!-- i2c_read_ack_cycle {{{1 --> publish acknowledge among I2C reading.
 */
void i2c_read_ack_cycle(int ack_or_nack) {
    digitalWrite(PIN_WIRE_SDA, ack_or_nack == OC_ACK ? LOW: HIGH);
    delayMicroseconds(W);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(W * 2);
    digitalWrite(PIN_WIRE_SCL, LOW);
    delayMicroseconds(W * 10);
}

/** <!-- i2c_read_reg8 {{{1 --> I2C read bytes.
 */
bool i2c_read_reg8(uint8_t addr7, uint8_t reg, uint8_t* buf, int n) {
    if (i2c_write_reg8(addr7, &reg, 1)) {
        Serial.print("Nack in read8_1");
        return true;  // NAck
    }
    //digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(W);
    i2c_start();
    i2c_write_8cycles((addr7 << 1) | 1);
    if (i2c_write_ack()) {
        Serial.print("Nack in read8_2");
        return true;  // NAck
    }

    for (int i = 0; i < n - 1; i++) {
        buf[i] = i2c_read_8cycles();
        i2c_read_ack_cycle(OC_ACK);
    }

    buf[n - 1] = i2c_read_8cycles();
    i2c_read_ack_cycle(OC_NACK);
    i2c_stop();
    return false;
}

#undef W

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
    // D6T-32L can not be read with Wire library, so pins are used as GPIO.
    pinMode(PIN_WIRE_SDA, OUTPUT);
    pinMode(PIN_WIRE_SCL, OUTPUT);
    digitalWrite(PIN_WIRE_SDA, HIGH);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    
    delay(350); 
    uint8_t data[2] = {D6T_SET_ADD, (((uint8_t)D6T_IIR << 4)&&0xF0) | (0x0F && (uint8_t)D6T_AVERAGE)};
    i2c_write_reg8(D6T_ADDR, data, 2);
    delay(390);  
}

/** <!-- loop - Thermal sensor {{{1 -->
 * 2. read data.
 */
void loop() {
  int i = 0;
	int16_t itemp = 0;
	
	// Read data via I2C
	memset(rbuf, 0, N_READ);
	i2c_read_reg8(D6T_ADDR, D6T_CMD, rbuf, N_READ);
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
	
  delay(200);	
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
