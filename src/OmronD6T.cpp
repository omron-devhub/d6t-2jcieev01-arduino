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
 * 
 * Refactored by: Aron Rubin 2020
 */

#include <Arduino.h>
#include <Wire.h>
#include "OmronD6T.h"


namespace d6t {
enum CmdTgt : uint8_t {
  TGT_CTRL = 0x02,
  TGT_CNFG = 0x03,
  TGT_RUNT = 0x05,
  TGT_DATA = 0x4C
};

enum RegAdr : uint8_t {
  REG_CTRL = 0x00,
  REG_CNFG = 0x00,
  REG_RUNT = 0x90
};

enum RegVal : uint8_t {
  CTRL_ON  = 0x01,
  CTRL_OFF = 0x00,
  CNFG_1 = 0x03,
  CNFG_2 = 0x07,
  RUN_VAL = 0x3A
};
} // namespace d6t

OmronD6T::OmronD6T( Model model ) {
  m_model = model > D6T_32L ? D6T_1A : model;
  switch (m_model) {
    case D6T_1A: m_numElements = 1; m_rows = 1; m_cols = 1; break;
    case D6T_8L: m_numElements = 8; m_rows = 1; m_cols = 8; break;
    case D6T_44L: m_numElements = 4*4; m_rows = 4; m_cols = 4; break;
    case D6T_32L: m_numElements = 32*32; m_rows = 32; m_cols = 32; break;
    default: m_numElements = 1; m_rows = 1; m_cols = 1; break;
  }
  m_objTempMaxIdx = 0;
  m_ambientTempTenthC = 0;
  m_objTempsTenthC = new int16_t[m_numElements];
}

OmronD6T::~OmronD6T() {
  if (m_objTempsTenthC) {
    delete [] m_objTempsTenthC;
  }
}


uint8_t OmronD6T::read8Array( uint8_t regaddr, uint8_t *buf, size_t len, bool checkPec ) const {
  Wire.beginTransmission( m_addr ); // start transmission to device
  Wire.write( regaddr );           // sends register address to read from
  uint8_t ret = Wire.endTransmission( false );   // end transmission
  if (ret != I2C_ERROR_OK) {
    return ret;
  }

  uint8_t pecl = 0;
  ret = Wire.requestFrom( m_addr, len + 1U ); // send request for (len + 1) bytes (len data, 1 pec)
  if (ret == 0) {
    return Wire.lastError();
  }
  pecl = crc8up( pecl, (m_addr << 1) | 0x1 ); // update CRC with 7-bit address shifted with master flag (1)
  for (size_t pos = 0; pos < len; pos++) {
    buf[pos] = Wire.read();  // receive payload byte
    pecl = crc8up( pecl, buf[pos] ); // update CRC from payload byte
  }
  uint8_t pecr = Wire.read();   // receive remote PEC
  if (pecr != pecl) {
    ret = SMBUS_ERROR_PEC;
  }
  return ret;
}

uint8_t OmronD6T::read16sArray( uint8_t regaddr, int16_t *buf, size_t numwords, bool sendStop ) const {
  Wire.beginTransmission( m_addr ); // start transmission to device
  Wire.write( regaddr );           // sends register address to read from
  uint8_t ret = Wire.endTransmission( false );   // end transmission
  if (ret != I2C_ERROR_OK) {
    return ret;
  }

  uint8_t pecl = 0;
  ret = Wire.requestFrom( m_addr, 2*numwords + 1U ); // send request for (2*numwords + 1) bytes
  if (ret == 0) {
    return Wire.lastError();
  }
  pecl = crc8up( pecl, (m_addr << 1) | 0x1 ); // update CRC with 7-bit address shifted with master flag (1)
  for (size_t pos = 0; pos < numwords; pos++) {
    int paylo = Wire.read();  // receive payload byte
    pecl = crc8up( pecl, paylo ); // update CRC from payload byte
    int payhi = Wire.read();  // receive payload byte
    pecl = crc8up( pecl, payhi ); // update CRC from payload byte
    buf[pos] = payhi << 8 | (paylo & 0xFF);  // receive payload byte
  }
  uint8_t pecr = Wire.read();   // receive remote PEC
  if (pecr != pecl) {
    ret = SMBUS_ERROR_PEC;
  }
  return ret;
}

uint8_t OmronD6T::crc8up( uint8_t crc, uint8_t data ) {
  crc ^= data;
  for (uint8_t bit_i = 8; bit_i; bit_i--) {
     // if MSB set then shift and xor with polynomial, otherwise just shift
    crc = (crc & 0x80) ? (crc << 1) ^ 0x7 : crc << 1;
  }
  return crc;
}

uint8_t OmronD6T::write16( uint8_t regaddr, uint8_t data0, uint8_t data1, bool sendStop ) const {
  uint8_t pec = 0;
  Wire.beginTransmission( m_addr );
  pec = crc8up( 0, m_addr << 1 );
  Wire.write( regaddr );  // register address to write
  pec = crc8up( pec, regaddr );
  Wire.write( data0 );    // lo
  pec = crc8up( pec, data0 );
  Wire.write( data1 );    // hi
  pec = crc8up( pec, data1 );
  Wire.write( pec );      // pec
  return Wire.endTransmission( sendStop );
}

bool OmronD6T::begin( uint8_t i2caddr ) {
  m_addr = i2caddr;

  bool ret = Wire.begin();
  if (!ret) {
    return false;
  }

  write16( d6t::TGT_CTRL, d6t::REG_CTRL, d6t::CTRL_ON );
  write16( d6t::TGT_RUNT, d6t::REG_RUNT, d6t::RUN_VAL );
  write16( d6t::TGT_CNFG, d6t::REG_CNFG, d6t::CNFG_1 );
  write16( d6t::TGT_CNFG, d6t::REG_CNFG, d6t::CNFG_2 );
  write16( d6t::TGT_CTRL, d6t::REG_CTRL, d6t::CTRL_OFF );
  // check the writes
  uint8_t bytebuf[2];
  if (!read8Array( d6t::TGT_CTRL, bytebuf, sizeof(bytebuf), false ) || bytebuf[0] != d6t::REG_CTRL || bytebuf[1] != d6t::CTRL_OFF) {
    Serial.println( "Error initializing D6T. Unexpected value" );
    ret = false;
  }
  if (!read8Array( d6t::TGT_RUNT, bytebuf, sizeof(bytebuf), false ) || bytebuf[0] != d6t::REG_RUNT || bytebuf[1] != d6t::RUN_VAL) {
    Serial.println( "Error initializing D6T. Unexpected value" );
    ret = false;
  }
  if (!read8Array( d6t::TGT_CNFG, bytebuf, sizeof(bytebuf), false ) || bytebuf[0] != d6t::REG_CNFG || bytebuf[1] != d6t::CNFG_2) {
    Serial.println( "Error initializing D6T. Unexpected value" );
    ret = false;
  }

  return ret;
}

uint8_t OmronD6T::read() {
  m_objTempMaxIdx = 0;

  // Wire buffers are enough to read D6T-16L data (33bytes) with
  // MKR-WiFi1010 and Feather ESP32,
  // these have 256 and 128 buffers in their libraries.
  Wire.beginTransmission( m_addr ); // start transmission to device
  Wire.write( d6t::TGT_DATA );           // sends register address to read from
  uint8_t ret = Wire.endTransmission( false );   // end transmission
  if (ret != I2C_ERROR_OK) {
    return ret;
  }

  uint8_t pecl = 0;
  ret = Wire.requestFrom(
      m_addr,
      sizeof(m_ambientTempTenthC) + m_numElements*sizeof(m_objTempsTenthC[0]) + 1U );
  if (ret == 0) {
    return Wire.lastError();
  }
  pecl = crc8up( pecl, (m_addr << 1) | 0x1 ); // update CRC with 7-bit address shifted with master flag (1)
  // 1st data is PTAT measurement (: Proportional To Absolute Temperature)
  int paylo = Wire.read();  // receive payload byte
  pecl = crc8up( pecl, paylo ); // update CRC from payload byte
  int payhi = Wire.read();  // receive payload byte
  pecl = crc8up( pecl, payhi ); // update CRC from payload byte
  m_ambientTempTenthC = payhi << 8 | (paylo & 0xFF); // pack into le int16

  memset( m_objTempsTenthC, 0, m_numElements*sizeof(m_objTempsTenthC[0]) );
  for (size_t elem = 0; elem < m_numElements; elem++) {
    paylo = Wire.read();  // receive payload byte
    pecl = crc8up( pecl, paylo ); // update CRC from payload byte
    payhi = Wire.read();  // receive payload byte
    pecl = crc8up( pecl, payhi ); // update CRC from payload byte
    m_objTempsTenthC[elem] = payhi << 8 | (paylo & 0xFF);  // receive payload byte
    if (m_objTempsTenthC[m_objTempMaxIdx] < m_objTempsTenthC[elem]) {
      m_objTempMaxIdx = elem;
    }
  }
  uint8_t pecr = Wire.read();   // receive remote PEC
  if (pecr != pecl) {
    ret = SMBUS_ERROR_PEC;
  }
  return ret;
}
