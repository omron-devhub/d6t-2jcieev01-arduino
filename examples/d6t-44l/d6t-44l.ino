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
#define D6T_32L 1
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
#define N_ROW 4
#define N_PIXEL (4 * 4)
#elif defined(D6T_8L)
#define N_ROW 8
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
    ret += buf[n + 1] << 8;
    return (int16_t)ret;   // and convert negative.
}


/** <!-- setup {{{1 -->
 * 1. initialize a Serial port for output.
 * 2. initialize an I2C peripheral.
 */
void setup() {
    Serial.begin(115200);  // Serial bourd rate = 115200bps
    #if defined(D6T_32L)
    // Wire library buffers are 128 or 256 in MKR-WiFi1010/Feather ESP32.
    // D6T-32L needs to be read 32x32x2 bytes at one-time from I2C,
    // so Arduino's Wire can not be used.
    pinMode(PIN_WIRE_SDA, OUTPUT);
    pinMode(PIN_WIRE_SCL, OUTPUT);
    digitalWrite(PIN_WIRE_SDA, HIGH);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    #else
    Wire.begin();  // i2c master
    #endif
}


/** <!-- loop - Thermal sensor {{{1 -->
 * 1. read sensor.
 * 2. output results, format is: [degC]
 */
void loop() {
    int i, j;

    memset(rbuf, 0, N_PIXEL * 2 + 1);
    #if defined(D6T_32L)
    // Wire library can not be used with D6T-32L by narrow buffers,
    // see setup().
    i2c_read_8(D6T_addr, D6T_cmd, rbuf, (N_PIXEL + 1) * 2 + 1);

    #else
    // Wire buffers are enough to read D6T-16L data (33bytes) with
    // MKR-WiFi1010/Feather ESP32.
    // these have 256 and 128 buffers in their libraries.
    Wire.beginTransmission(D6T_addr);  // I2C client address
    Wire.write(D6T_cmd);               // D6T register
    Wire.endTransmission();            // I2C repeated start for read
    Wire.requestFrom(D6T_addr, (N_PIXEL + 1) * 2 + 1);
    i = 0;
    while (Wire.available()) {
        rbuf[i++] = Wire.read();
    }
    #endif

    // 1st data is PTAT measurement (: Proportional To Absolute Temperature)
    int16_t itemp = conv8us_s16_le(rbuf, 0);
    Serial.print("PTAT:");
    Serial.println(itemp / 10.0, 1);

    // loop temperature pixels of each thrmopiles measurements
    for (i = 0, j = 2; i < N_PIXEL; i++, j += 2) {
        itemp = conv8us_s16_le(rbuf, j);
        Serial.print(itemp / 10.0, 1);  // print PTAT & Temperature
        if ((i % N_ROW) == N_ROW - 1) {
            Serial.println(" [degC]");  // wrap text at ROW end.
        } else {
            Serial.print(",");   // print delimiter
        }
    }
    delay(1000);
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
