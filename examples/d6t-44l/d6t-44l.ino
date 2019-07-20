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

void setup()
{
   Serial.begin(9600); // Set bourd rate = 9600bps
   Wire.begin(); // i2c master
}

#define  D6T_addr  0x0A // !! 7bit expression !!
#define  D6T_cmd   0x4C
byte  rbuf[32];
//int  tdata[8];
int  tdata[16];
int  t_PTAT;
byte crc;

int D6T_checkPEC( byte buf[] , int pPEC )
    {
    int i;
//    crc = calc_crc( 0x14 );
//    crc = calc_crc( 0x4C ^ crc );
//    crc = calc_crc( 0x15 ^ crc );
    crc = calc_crc( 0x15 );
    for(i=0;i<pPEC;i++)
      {
      crc = calc_crc( rbuf[i] ^ crc );
        }
    return (crc == rbuf[pPEC]);
    }

byte calc_crc( byte data )
    {
    int index;
    byte temp;
    for(index=0;index<8;index++)
      {
      temp = data;
      data <<= 1;
      if(temp & 0x80) data ^= 0x07;
    }
    return data;
    }

void loop()
{
  int  i,j;
  int  itemp;
  
  Wire.beginTransmission(D6T_addr);
  Wire.write(D6T_cmd);
  Wire.endTransmission();
//  Wire.requestFrom(D6T_addr, 19); // for D6T-8L-06
  Wire.requestFrom(D6T_addr, 32); // for D6T-44L-06
  i = 0;
  while(Wire.available()){
    rbuf[i++] = Wire.read();
    }

//  if(D6T_checkPEC(rbuf, 32) == 0){ ; // error routine 
//    Serial.println("PEC Error !!");       //print space
//  }
//  for(i=0,j=0;i<9;i++){ // for 1x8
  for(i=0,j=0;i<16;i++){ // for 4x4
    itemp = 0x00ff & rbuf[j++];
    itemp += rbuf[j++] << 8;
    if(i == 0) { // PTAT measurement mode
      t_PTAT = itemp;
      Serial.print(t_PTAT, 1); //print PTAT & Temperature
      Serial.print(" ");       //print space
    }
    else { // each thrmopile temperature mode
      tdata[i-1] = itemp;
      if(i<15){
        Serial.print(tdata[i-1], 1); //print PTAT & Temperature
        Serial.print(" ");       //print space
    }
      else{
        Serial.print(tdata[i-1], 1); //print Temperature
        Serial.println(" ");           //print space
//        Serial.print(rbuf[18], HEX); //print read PEC
//        Serial.print(" ");           //print space
//        Serial.println(crc, HEX);      //print calc PEC
    }
    }
    }
  ; // post operation.
  delay(500);
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
