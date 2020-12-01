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


#ifndef OMRON_D6T_H_INCLUDED
#define OMRON_D6T_H_INCLUDED

#include <stdint.h>
#include <stddef.h>

#ifndef SMBUS_ERROR_PEC
#define SMBUS_ERROR_PEC 16
#endif

class OmronD6T {
public:
  static constexpr const char *devclassname = "OmronD6T";
  static constexpr uint8_t I2CADDR = 0x0A;

  enum Model : uint8_t {
    D6T_1A,
    D6T_8L,
    D6T_44L,
    D6T_32L
  };

  OmronD6T( Model model = D6T_1A );
  ~OmronD6T();

  bool begin( uint8_t i2caddr = I2CADDR );
  uint8_t read();

  size_t numElements() const {
    return m_numElements;
  }
  size_t cols() const {
    return m_cols;
  }
  size_t rows() const {
    return m_rows;
  }

  int16_t objectTempMaxTenthC() const {
    return m_objTempsTenthC[m_objTempMaxIdx];
  }

  int16_t objectTempTenthC( size_t element ) const {
    if (element > m_numElements) {
      return 0;
    }
    return m_objTempsTenthC[element];
  }

  // matrix style
  int16_t objectTempTenthC( size_t col, size_t row ) const {
    if (col > m_cols || row > m_rows) {
      return 0;
    }
    return m_objTempsTenthC[row*m_cols + col];
  }

  int16_t ambientTempTenthC() const {
    return m_ambientTempTenthC;
  }

  double objectTempMaxC() const {
    return m_objTempsTenthC[m_objTempMaxIdx] * 0.1;
  }

  double objectTempC( size_t element ) const {
    return objectTempTenthC( element ) * 0.1;
  }

  double objectTempC( size_t col, size_t row ) const {
    return objectTempTenthC( col, row ) * 0.1;
  }

  double ambientTempC() const {
    return m_ambientTempTenthC * 0.1;
  }

  double objectTempMaxF() const {
    return objectTempMaxC() * 9.0 / 5.0 + 32.0;
  }

  double objectTempF( size_t element ) const {
    return objectTempC( element ) * 9.0 / 5.0 + 32.0;
  }

  double objectTempF( size_t col, size_t row ) const {
    return objectTempC( col, row ) * 9.0 / 5.0 + 32.0;
  }

  double ambientTempF() const {
    return ambientTempC() * 9.0 / 5.0 + 32.0;
  }

private:
  uint8_t read8Array( uint8_t regaddr, uint8_t *buf, size_t len, bool checkPec = false ) const;
  uint8_t read16sArray( uint8_t regaddr, int16_t *buf, size_t numwords, bool checkPec = false ) const;
  uint8_t write8( uint8_t regaddr, uint8_t data, bool sendStop = true ) const;
  uint8_t write16( uint8_t regaddr, uint8_t data0, uint8_t data1, bool sendStop = true ) const;
  static uint8_t crc8up( uint8_t initcrc, uint8_t data );

  uint8_t m_addr;
  Model m_model;
  size_t m_numElements;
  uint8_t m_rows;
  uint8_t m_cols;
  int16_t m_ambientTempTenthC;
  int16_t *m_objTempsTenthC;    // alias into tempsTenthC
  size_t m_objTempMaxIdx;
};


#endif // OMRON_D6T_H_INCLUDED
