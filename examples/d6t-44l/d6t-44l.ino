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
#if 1
#define OC_SOFTI2C 1
#endif

#define D6T_addr 0x0A  // for I2C 7bit address
#if defined(D6T_4C)
#define D6T_cmd 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02
#else
#define D6T_cmd 0x4D  // for D6T-32L-01A
#endif

#if defined(D6T_32L)
#define N_ROW 32
#define N_PIXEL (32 * 32)
#elif defined(D6T_44L)
#define N_ROW 1
#define N_PIXEL (4 * 4)
#elif defined(D6T_8L)
#define N_ROW 1
#define N_PIXEL 8
#elif defined(D6T_1A)
#define N_ROW 1
#define N_PIXEL 1
#endif

uint8_t rbuf[N_PIXEL * 2 + 1];


int D6T_checkPEC(byte buf[] , int pPEC ) {
    int i;
    uint8_t crc = calc_crc(0x15);
    for (i = 0; i < pPEC; i++) {
        crc = calc_crc(rbuf[i] ^ crc);
    }
    return (crc == rbuf[pPEC]);
}

byte calc_crc(byte data) {
    int index;
    byte temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

#if defined(D6T_32L)
#define W 10  // I2C speed = 1 / (4 * N) [MHz]

typedef enum OC_ACKNACK {
    OC_ACK = 0,
    OC_NACK = 1
};

void i2c_start() {
    // Serial.print("i2c_start...");
    digitalWrite(PIN_WIRE_SDA, LOW);
    delayMicroseconds(W * 4);
    digitalWrite(PIN_WIRE_SCL, LOW);
    delayMicroseconds(W * 4);
}

void i2c_stop() {
    digitalWrite(PIN_WIRE_SDA, HIGH);
    delayMicroseconds(W * 4);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(W * 4);
    Serial.println("i2c_stop");
}

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

bool i2c_write_8(uint8_t addr8, uint8_t reg) {
    i2c_start();
    i2c_write_8cycles(addr8);
    if (i2c_write_ack()) {
        Serial.print("Nack in write8_1");
        i2c_stop();
        return true;
    }
    i2c_write_8cycles(reg);
    if (i2c_write_ack()) {
        Serial.print("Nack in write8_2");
        i2c_stop();
        return true;
    }
    return false;
}

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

void i2c_read_ack_cycle(int ack_or_nack) {
    digitalWrite(PIN_WIRE_SDA, ack_or_nack == OC_ACK ? LOW: HIGH);
    delayMicroseconds(W);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(W * 2);
    digitalWrite(PIN_WIRE_SCL, LOW);
    delayMicroseconds(W * 10);
}

bool i2c_read_8(uint8_t addr7, uint8_t reg, uint8_t* buf, int n) {
    if (i2c_write_8(addr7 << 1, reg)) {
        Serial.print("Nack in read8_1");
        return true;  // NAck
    }
    digitalWrite(PIN_WIRE_SCL, HIGH);
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
}

#undef W
#endif

/** <!-- conv8us_s16_le {{{1 -->
 */
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    int ret;
    ret = buf[n];
    ret += buf[n] << 8;
    return (int16_t)ret;   // and convert negative.
}


/** <!-- setup {{{1 -->
 * 1. initialize a Serial port for output.
 * 2. initialize an I2C peripheral.
 */
void setup() {
    Serial.begin(115200);  // Serial bourd rate = 115200bps
    #if defined(D6T_32L)
    pinMode(PIN_WIRE_SDA, OUTPUT);
    pinMode(PIN_WIRE_SCL, OUTPUT);
    digitalWrite(PIN_WIRE_SDA, HIGH);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    #else
    Wire.begin();  // i2c master
    #endif
}


/** <!-- loop - Thermal sensor {{{1 -->
 * 1. read and convert sensor.
 * 2. output results, format is: [degC]
 */
void loop() {
    int i, j;

    #if 0   // split to 32bytes (regular arduino)
    Wire.beginTransmission(D6T_addr);  // I2C start
    Wire.write(D6T_cmd);  // I2C write
    Wire.endTransmission();  // I2C stop
//  Wire.requestFrom(D6T_addr, 5); // for 1×1
//  Wire.requestFrom(D6T_addr, 19); // for 1×8
//  Wire.requestFrom(D6T_addr, 35); // for 4×4
//  Wire.requestFrom(D6T_addr, 2051); // for 32×32
    for (int n = 0; n < 64; n++) {
        Wire.requestFrom(D6T_addr, 32, 0);
        while (Wire.available()) {
            rbuf[i++] = Wire.read();
        }
    }
    Wire.requestFrom(D6T_addr, 3);
    while (Wire.available()) {
        rbuf[i++] = Wire.read();
    }
    #elif defined(OC_SOFTI2C)
    delay(10000);
    i2c_read_8(D6T_addr, D6T_cmd, rbuf, N_PIXEL);

    #else  // succeed: 2019/07/18
    Wire.beginTransmission(D6T_addr);  // I2C client address
    Wire.write(D6T_cmd);               // I2C register
    Wire.endTransmission();            // I2C repeated start for read
    Wire.requestFrom(D6T_addr, N_PIXEL);
    i = 0;
    while (Wire.available()) {
        rbuf[i++] = Wire.read();
    }
    #endif

    // loop pixels + ptat data.
    for (i=0, j=0; i < N_PIXEL + 1; i++, j += 2) {
        int16_t itemp = conv8us_s16_le(rbuf, j);
        if (i == 0) {  // a result of PTAT measurement
            Serial.print("PTAT:");
            Serial.print(itemp, 1);
        } else {      // each thrmopiles temperature measurements
            if (i % N_ROW == 1) {
                Serial.println("");  // wrap text at ROW end.
            }
            Serial.print(itemp, 1);  // print PTAT & Temperature
            Serial.print(" ");       // print space
        } else {
            Serial.println(itemp, 1);  // print Temperature
        }
    }
    delay(3000);
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
