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
#if !defined(__LIS2DW_H__)
#define __LIS2DW_H__
#include <Arduino.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
//    #define LIS2DW_DEFAULT_ADDRESS  (0x18)    // if SDO/SA0 is 3V, its 0x19
/*=========================================================================*/

//#define LIS2DW_REG_STATUS1       0x07
//#define LIS2DW_REG_OUTADC1_L     0x08
//#define LIS2DW_REG_OUTADC1_H     0x09
//#define LIS2DW_REG_OUTADC2_L     0x0A
//#define LIS2DW_REG_OUTADC2_H     0x0B
//#define LIS2DW_REG_OUTADC3_L     0x0C
//#define LIS2DW_REG_OUTADC3_H     0x0D
//#define LIS2DW_REG_INTCOUNT      0x0E
#define LIS2DW_REG_TEMP_L        0x0D
#define LIS2DW_REG_TEMP_H        0x0E
#define LIS2DW_REG_WHOAMI        0x0F
//#define LIS2DW_REG_TEMPCFG       0x1F
#define LIS2DW_REG_CTRL1         0x20
#define LIS2DW_REG_CTRL2         0x21
#define LIS2DW_REG_CTRL3         0x22
#define LIS2DW_REG_CTRL4         0x23
#define LIS2DW_REG_CTRL5         0x24
#define LIS2DW_REG_CTRL6         0x25
#define LIS2DW_REG_REFERENCE     0x26
//#define LIS2DW_REG_STATUS2       0x27
#define LIS2DW_REG_STATUS        0x27
#define LIS2DW_REG_OUT_X_L       0x28
#define LIS2DW_REG_OUT_X_H       0x29
#define LIS2DW_REG_OUT_Y_L       0x2A
#define LIS2DW_REG_OUT_Y_H       0x2B
#define LIS2DW_REG_OUT_Z_L       0x2C
#define LIS2DW_REG_OUT_Z_H       0x2D
#define LIS2DW_REG_FIFOCTRL      0x2E
#define LIS2DW_REG_FIFOSRC       0x2F
#define LIS2DW_REG_INT1CFG       0x30
#define LIS2DW_REG_INT1SRC       0x31
#define LIS2DW_REG_INT1THS       0x32
#define LIS2DW_REG_INT1DUR       0x33
#define LIS2DW_REG_CLICKCFG      0x38
#define LIS2DW_REG_CLICKSRC      0x39
#define LIS2DW_REG_CLICKTHS      0x3A
#define LIS2DW_REG_TIMELIMIT     0x3B
#define LIS2DW_REG_TIMELATENCY   0x3C
#define LIS2DW_REG_TIMEWINDOW    0x3D
#define LIS2DW_REG_ACTTHS        0x3E
//#define LIS2DW_REG_ACTDUR        0x3F
#define LIS2DW_REG_CTRL7        0x3F

#define LIS2DW_FIFO_SIZE   1
#define LIS2DW_MEAS_TIME   10   //100hz

typedef enum
{
  LIS2DW_RANGE_16_G         = 0b11,   // +/- 16g
  LIS2DW_RANGE_8_G           = 0b10,   // +/- 8g
  LIS2DW_RANGE_4_G           = 0b01,   // +/- 4g
  LIS2DW_RANGE_2_G           = 0b00,    // +/- 2g (default value)
} lis2dw_range_t;

typedef enum
{
  LIS2DW_AXIS_X         = 0x0,
  LIS2DW_AXIS_Y         = 0x1,
  LIS2DW_AXIS_Z         = 0x2,
} lis2dw_axis_t;


/* Used with register 0x2A (LIS2DW_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
  LIS2DW_DATARATE_400_HZ     = 0b0111, //  400Hz 
  LIS2DW_DATARATE_200_HZ     = 0b0110, //  200Hz
  LIS2DW_DATARATE_100_HZ     = 0b0101, //  100Hz
  LIS2DW_DATARATE_50_HZ      = 0b0100, //   50Hz
  LIS2DW_DATARATE_25_HZ      = 0b0011, //   25Hz
  LIS2DW_DATARATE_10_HZ      = 0b0010, // 10 Hz
  LIS2DW_DATARATE_1_HZ       = 0b0001, // 1 Hz
  LIS2DW_DATARATE_POWERDOWN  = 0,
  LIS2DW_DATARATE_LOWPOWER_1K6HZ  = 0b1000,
  LIS2DW_DATARATE_LOWPOWER_5KHZ  =  0b1001,

} lis2dw_dataRate_t;

typedef struct {
//  int16_t x, y, z;
  int16_t x_g[LIS2DW_FIFO_SIZE];
  int16_t y_g[LIS2DW_FIFO_SIZE];
  int16_t z_g[LIS2DW_FIFO_SIZE];
  int16_t acctemp;
} lis2dw_data_t;

extern volatile boolean acc_int1flag;
extern uint32_t accms;
extern int16_t sens_accel[3];

extern uint32_t debugacccounter;

void lis2dw_fifo_int1(void);
void lis2dw_fifo_read(uint16_t* pdata);
void lis2dw_getinitialval(uint8_t* pdata);
boolean lis2dw_threscheck(int16_t* pdata);
void lis2dw_getTemp(int16_t* pdata);
#endif
