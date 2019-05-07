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
#include "sht30.h"
#include <Wire.h>

/* defines */
#define SHT30_ADDR  0x44
#define SHT30_STATUSMASK  0xFC1F

#define GPIO_LED_R_PIN A12
#define GPIO_LED_G_PIN A11
#define GPIO_LED_B_PIN A1

#define conv16_u8_h(a) (uint8_t)(a >> 8)
#define conv16_u8_l(a) (uint8_t)(a & 0xFF)

#define conv8s_u16_be(b, n) ((uint16_t)b[n + 1] | ((uint16_t)b[n] << 8))
#define ap_halt(a) {Serial.println(a); while (1) {}}

/* I2C functions */
/** <!-- i2c_write_reg16 {{{1 --> I2C write function for byte transfer.
 */
bool i2c_write_reg16(uint8_t slave_addr, uint16_t register_addr,
                        uint8_t *write_buff, uint8_t len) {
    Wire.beginTransmission(slave_addr);
    Wire.write(conv16_u8_h(register_addr));
    Wire.write(conv16_u8_l(register_addr));

    if (len > 0) {
        for (uint8_t i = 0; i < len; i++) {
            Wire.write(write_buff[i]);
        }
    }
    Wire.endTransmission();
    return false;
}

/** <!-- i2c_read_reg16 {{{1 --> I2C read function for bytes transfer.
 */
bool i2c_read_reg16(uint8_t slave_addr, uint16_t register_addr,
                       uint8_t *read_buff, uint8_t len) {
    i2c_write_reg16(slave_addr, register_addr, NULL, 0);

    Wire.requestFrom(slave_addr, len);

    if (Wire.available() != len) {
        return true;
    }
    for (uint16_t i = 0; i < len; i++) {
        read_buff[i] = Wire.read();
    }
    return false;
}

/** <!-- sht30_setup {{{1 --> setup a humidity sensor.
 */
void sht30_setup() {
    sht30_reset();
    delay(10);
    sht30_startcheck();
    sht30_measstart();
}

/** <!-- sht30_reset {{{1 --> issue software reset and wait for it.
 */
void sht30_reset(void) {
    i2c_write_reg16(SHT30_ADDR, SHT30_SOFTRESET, NULL, 0);
    delay(10);
}

/** <!-- sht30_startcheck {{{1 --> clear status and check start status.
 */
void sht30_startcheck(void) {
    uint16_t stat = 0;
    uint32_t retry = 100;

    i2c_write_reg16(SHT30_ADDR, SHT30_CLEARSTATUS, NULL, 0);  // clear status
    delay(1);

    do {
        stat = sht30_readstatus();  // check status
        retry--;
        delay(10);
    } while (((stat & SHT30_STATUSMASK) != 0x0000) && (retry > 0));

    if (((stat & SHT30_STATUSMASK) != 0x0000) || (retry == 0)) {
        ap_halt("cannot detect SHT30 working.");
    }
}

/** <!-- sht30_measstart {{{1 --> start measurement.
 */
void sht30_measstart(void) {
    i2c_write_reg16(SHT30_ADDR, SHT30_MEAS_HIGHPRD, NULL, 0);
}

/** <!-- sht30_readstatus {{{1 --> */
uint16_t sht30_readstatus(void) {
    bool result;
    uint8_t readbuffer[3] = {0, 0, 0};
    uint16_t stat = 0xFFFF;

    result = i2c_read_reg16(SHT30_ADDR, SHT30_READSTATUS, readbuffer, 3);
    if (!result) {
        stat = (((uint16_t)readbuffer[0] << 8) | (uint16_t)readbuffer[1]);
    }
    return stat;
}

/** <!-- sht30_readTempHumi {{{1 --> read raw digits and convert them to
 * physical values.
 */
bool sht30_readTempHumi(int32_t* humi, int32_t* temp) {
    bool result;
    uint8_t readbuffer[6];

    result = i2c_read_reg16(SHT30_ADDR, SHT30_READ_PERIODIC, readbuffer, 6);
    if (result) {
        return true;
    }
    if (readbuffer[2] != sht30_crc8(readbuffer, 2)) {
        return true;  // check crc8 code failed.
    }
    if (readbuffer[5] != sht30_crc8(readbuffer + 3, 2)) {
        return true;  // check crc8 code failed.
    }

    uint16_t ST, SRH;
    ST = conv8s_u16_be(readbuffer, 0);
    SRH = conv8s_u16_be(readbuffer, 3);

    double stemp = (double)ST * 17500.0 / 65535.0 - 4500.0;
    *temp = (int32_t)stemp;

    //  Serial.print("SRH = "); Serial.println(SRH);
    double shum = (double)SRH * 10000.0 / 65535.0;
    *humi = (int32_t)shum;
    return false;
}

/** <!-- sht30_crc8 {{{1 --> CRC-8 formula from page 14 of SHT spec pdf
 * Test data 0xBE, 0xEF should yield 0x92.
 *
 * Initialization data 0xFF
 * Polynomial 0x31 (x8 + x5 +x4 +1)
 * Final XOR 0x00
 */
uint8_t sht30_crc8(const uint8_t *data, int len) {
    const uint8_t POLYNOMIAL(0x31);
    uint8_t crc(0xFF);

    for (int j = len; j; --j) {
        crc ^= *data++;

        for (int i = 8; i; --i) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

/** <!-- setup - humidity sensor {{{1 -->
 * 1. setup LED gpio.
 * 2. setup sensor
 */
void setup() {
    Serial.begin(115200);
    Serial.println("peripherals: GPIO");
    pinMode(GPIO_LED_R_PIN, OUTPUT);
    pinMode(GPIO_LED_G_PIN, OUTPUT);
    pinMode(GPIO_LED_B_PIN, OUTPUT);

    digitalWrite(GPIO_LED_R_PIN, LOW);
    digitalWrite(GPIO_LED_G_PIN, LOW);
    digitalWrite(GPIO_LED_B_PIN, LOW);

    Serial.println("peripherals: I2C");
    Wire.begin();  // master

    Serial.println("sensor: illuminance");
    sht30_setup();
    delay(32);
}

/** <!-- loop - humidity sensor {{{1 -->
 * 1. blink LEDs
 * 2. read and convert sensor.
 * 3. output results, format is: x100[%RH], x100[degC]
 */
void loop() {
    static bool blink = false;
    int32_t humi, temp;

    blink = !blink;
    digitalWrite(GPIO_LED_R_PIN, blink ? HIGH: LOW);
    digitalWrite(GPIO_LED_G_PIN, blink ? HIGH: LOW);
    digitalWrite(GPIO_LED_B_PIN, blink ? HIGH: LOW);
    delay(900);
    sht30_readTempHumi(&humi, &temp);
    Serial.print("sensor output:");
    Serial.print(humi);
    Serial.print(",");
    Serial.print(temp);
    Serial.println("");
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
