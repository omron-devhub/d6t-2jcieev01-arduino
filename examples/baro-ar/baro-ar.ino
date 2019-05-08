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
#include "baro_2smpb02e.h"
#include <Wire.h>

/* defines */
#define BARO_2SMPB02E_CHIP_ID     0x5C

#define GPIO_LED_R_PIN 4
#define GPIO_LED_G_PIN 5
#define GPIO_LED_B_PIN 6

/* values */
baro_2smpb02e_setting_t baro_2smpb02e_setting;

/* macros */
#define conv8s_s24_be(a, b, c) \
        (int32_t)((((uint32_t)a << 16) & 0x00FF0000) | \
                  (((uint32_t)b << 8) & 0x0000FF00) | \
                   ((uint32_t)c & 0x000000FF))

#define baro_halt(a) {Serial.println(a); while (1) {}}


/* I2C functions */
/** <!-- i2c_write_reg8 {{{1 --> I2C write function for bytes transfer.
 */
bool i2c_write_reg8(uint8_t slave_addr, uint8_t register_addr,
                    uint8_t *write_buff, uint8_t len) {
    Wire.beginTransmission(slave_addr);

    Wire.write(register_addr);
    if (len != 0) {
        for (uint8_t i = 0; i < len; i++) {
            Wire.write(write_buff[i]);
        }
    }
    Wire.endTransmission();
    return false;
}

/** <!-- i2c_read_reg8 {{{1 --> I2C read function for bytes transfer.
 */
bool i2c_read_reg8(uint8_t slave_addr, uint8_t register_addr,
                   uint8_t *read_buff, uint8_t len) {
    i2c_write_reg8(slave_addr, register_addr, NULL, 0);

    Wire.requestFrom(slave_addr, len);

    if (Wire.available() != len) {
        return true;
    }
    for (uint16_t i = 0; i < len; i++) {
        read_buff[i] = Wire.read();
    }
    return false;
}


/** <!-- baro_2smpb02e_setup {{{1 --> setup for 2SMPB-02E
 * 1. check CHIP_ID to confirm I2C connections.
 * 2. read coefficient values for compensations.
 * 3. sensor setup and start to measurements.
 */
bool baro_2smpb02e_setup(void) {
    bool result;
    uint8_t rbuf[32] = {0};
    uint8_t ex;

    // 1.
    result = i2c_read_reg8(BARO_2SMPB02E_ADDRESS,
                           BARO_2SMPB02E_REGI2C_CHIP_ID, rbuf, 1);
    if (result || rbuf[0] != BARO_2SMPB02E_CHIP_ID) {
        baro_halt("cannot find 2SMPB-02E sensor, halted...");
    }

    // 2.
    result = i2c_read_reg8(BARO_2SMPB02E_ADDRESS,
            BARO_2SMPB02E_REGI2C_COEFS, rbuf, 25);
    if (result) {
        baro_halt("failed to read 2SMPB-02E coeffients, halted...");
    }

    // pressure parameters
    ex = (rbuf[24] & 0xf0) >> 4;
    baro_2smpb02e_setting._B00 = baro_2smpb02e_conv20q4_dbl(rbuf, ex, 0);
    baro_2smpb02e_setting._BT1 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BT1, BARO_2SMPB02E_COEFF_S_BT1, rbuf, 2);
    baro_2smpb02e_setting._BT2 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BT2, BARO_2SMPB02E_COEFF_S_BT2, rbuf, 4);
    baro_2smpb02e_setting._BP1 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BP1, BARO_2SMPB02E_COEFF_S_BP1, rbuf, 6);
    baro_2smpb02e_setting._B11 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_B11, BARO_2SMPB02E_COEFF_S_B11, rbuf, 8);
    baro_2smpb02e_setting._BP2 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BP2, BARO_2SMPB02E_COEFF_S_BP2, rbuf, 10);
    baro_2smpb02e_setting._B12 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_B12, BARO_2SMPB02E_COEFF_S_B12, rbuf, 12);
    baro_2smpb02e_setting._B21 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_B21, BARO_2SMPB02E_COEFF_S_B21, rbuf, 14);
    baro_2smpb02e_setting._BP3 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_BP3, BARO_2SMPB02E_COEFF_S_BP3, rbuf, 16);

    // temperature parameters
    ex = (rbuf[24] & 0x0f);
    baro_2smpb02e_setting._A0 = baro_2smpb02e_conv20q4_dbl(rbuf, ex, 18);
    baro_2smpb02e_setting._A1 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_A1, BARO_2SMPB02E_COEFF_S_A1, rbuf, 20);
    baro_2smpb02e_setting._A2 = baro_2smpb02e_conv16_dbl(
            BARO_2SMPB02E_COEFF_A_A2, BARO_2SMPB02E_COEFF_S_A2, rbuf, 22);

    // 3. setup a sensor at 125msec sampling and 32-IIR filter.
    rbuf[0] = BARO_2SMPB02E_VAL_IOSETUP_STANDBY_0125MS;
    i2c_write_reg8(BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_IO_SETUP,
                   rbuf, sizeof(rbuf));

    rbuf[0] = BARO_2SMPB02E_VAL_IIR_32TIMES;
    i2c_write_reg8(BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_IIR,
                   rbuf, sizeof(rbuf));

    // then, start to measurements.
    result = baro_2smpb02e_trigger_measurement(
            BARO_2SMPB02E_VAL_MEASMODE_ULTRAHIGH);
    if (result) {
        baro_halt("failed to wake up 2SMPB-02E sensor, halted...");
    }
    return false;
}

/** <!-- baro_2smpb02e_conv16_dbl {{{1 --> convert bytes buffer to double.
 * bytes buffer format is a signed-16bit Big-Endian.
 */
static double baro_2smpb02e_conv16_dbl(double a, double s,
                                       uint8_t* buf, int offset) {
    uint16_t val;
    int16_t ret;

    val = (uint16_t)(
            (uint16_t)(buf[offset] << 8) | (uint16_t)buf[offset + 1]);
    if ((val & 0x8000) != 0) {
        ret = (int16_t)((int32_t)val - 0x10000);
    } else {
        ret = val;
    }
    return a + (double)ret * s / 32767.0;
}

/** <!-- baro_2smpb02e_conv20q4_dbl {{{1 --> convert bytes buffer to double.
 * bytes buffer format is signed 20Q4, from -32768.0 to 32767.9375
 *
 * ### bit field of 20Q4
 * ```
 * |19,18,17,16|15,14,13,12|11,10, 9, 8| 7, 6, 5, 4| 3, 2, 1, 0|
 * | buf[offset]           | buf[offset+1]         | ex        |
 *                                                 A
 *                                                 |
 *                                                 +-- Decimal point
 * ```
 */
static double baro_2smpb02e_conv20q4_dbl(uint8_t* buf,
                                         uint8_t ex, int offset) {
    int32_t ret;
    uint32_t val;

    val = (uint32_t)((buf[offset] << 12) | (buf[offset + 1] << 4) | ex);
    if ((val & 0x80000) != 0) {
        ret = (int32_t)val - 0x100000;
    } else {
        ret = val;
    }
    return (double)ret / 16.0;
}

/** <!-- baro_2smpb02e_trigger_measurement {{{1 --> start the sensor
 */
static bool baro_2smpb02e_trigger_measurement(uint8_t mode) {
    uint8_t wbuf[1] = {
        (uint8_t)(mode | BARO_2SMPB02E_VAL_POWERMODE_NORMAL)};

    i2c_write_reg8(BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_CTRL_MEAS,
                   wbuf, sizeof(wbuf));
    return false;
}

/** <!-- baro_2smpb02e_read {{{1 --> read the sensor digit and convert to
 * physical values.
 */
int baro_2smpb02e_read(uint32_t* pres, int16_t* temp,
                      uint32_t* dp, uint32_t* dt) {
    bool ret;
    uint8_t rbuf[6] = {0};
    uint32_t rawtemp, rawpres;

    ret = i2c_read_reg8(
            BARO_2SMPB02E_ADDRESS, BARO_2SMPB02E_REGI2C_PRES_TXD2,
            rbuf, sizeof(rbuf));
    if (ret) {
        return 1;
    }

    *dp = rawpres = conv8s_s24_be(rbuf[0], rbuf[1], rbuf[2]);
    *dt = rawtemp = conv8s_s24_be(rbuf[3], rbuf[4], rbuf[5]);
    return baro_2smpb02e_output_compensation(rawtemp, rawpres, pres, temp);
}

/** <!-- baro_2smpb02e_output_compensation {{{1 --> compensate sensors
 * raw output digits to [Pa] and [degC].
 */
bool baro_2smpb02e_output_compensation(uint32_t raw_temp_val,
                                       uint32_t raw_press_val,
                                       uint32_t* pres, int16_t* temp
) {
    double Tr, Po;
    double Dt, Dp;

    Dt = (int32_t)raw_temp_val - 0x800000;
    Dp = (int32_t)raw_press_val - 0x800000;

    // temperature compensation
    baro_2smpb02e_setting_t* c = &baro_2smpb02e_setting;
    Tr = c->_A0 + c->_A1 * Dt + c->_A2 * (Dt * Dt);

    // barometer compensation
    Po = c->_B00 + (c->_BT1 * Tr) + (c->_BP1 * Dp) +
         (c->_B11 * Tr * Dp) + c->_BT2 * (Tr * Tr) +
         (c->_BP2 * (Dp * Dp)) + (c->_B12 * Dp * (Tr * Tr)) +
         (c->_B21 * (Dp * Dp) * Tr) + (c->_BP3 * (Dp * Dp * Dp));

    *temp = (int16_t)(Tr / 2.56);     // x100degC
    *pres = (uint32_t)(Po * 10.0);    // x10Pa
    return false;
}


/** <!-- setup - barometer sensor {{{1 -->
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

    Serial.println("sensor: barometer");
    baro_2smpb02e_setup();
    delay(32);
}

/** <!-- loop - barometer sensor {{{1 -->
 * 1. blink LEDs
 * 2. read and convert sensor.
 * 3. output results, format is: x10[Pa], x100[degC],digit,digit
 */
void loop() {
    static bool blink = false;
    uint32_t pres, dp, dt;
    int16_t temp;

    blink = !blink;
    digitalWrite(GPIO_LED_R_PIN, blink ? HIGH: LOW);
    digitalWrite(GPIO_LED_G_PIN, blink ? HIGH: LOW);
    digitalWrite(GPIO_LED_B_PIN, blink ? HIGH: LOW);
    delay(900);
    int ret = baro_2smpb02e_read(&pres, &temp, &dp, &dt);
    Serial.print("sensor output:");
    Serial.print(pres / 10.0);
    Serial.print("[Pa], ");
    Serial.print(temp / 100.0);
    Serial.print("[degC], ");
    Serial.print(dp);
    Serial.print("[],");
    Serial.print(dt);
    Serial.print("[], retun code:");
    Serial.println(ret);
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
