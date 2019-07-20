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
#define  D6T_addr  0x0A // !! 7bit expression !!
//#define  D6T_cmd   0x4C // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02
#define  D6T_cmd   0x4D // for D6T-32L-01A

byte rbuf[2051]; // for D6T-32L-01A
int  tdata[1024];
//byte rbuf[35]; // for D6T-44L-06/06H
//int  tdata[16];
//byte rbuf[19]; // for D6T-8L-09/09H
//int  tdata[8];
//byte rbuf[5];  // for D6T-1A-01/02
//int  tdata[1];
int  t_PTAT;
byte crc;

int D6T_checkPEC( byte buf[] , int pPEC ){
    int i;
    crc = calc_crc( 0x15 );
    for(i=0;i<pPEC;i++)
      {
      crc = calc_crc( rbuf[i] ^ crc );
      }
    return (crc == rbuf[pPEC]);
    }

byte calc_crc( byte data ){
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
    
void setup()
{
   Serial.begin(115200); // Set bourd rate = 115200bps
   Wire.begin(); // i2c master
}

 /** <!-- loop - Thermal sensor {{{1 -->
 * 1. read and convert sensor.
 * 2. output results, format is: [degree]
 */
void loop()
{
  int i = 0;
  int j = 0;
  int  itemp;
  
  Wire.beginTransmission(D6T_addr); // I2C start
  Wire.write(D6T_cmd); // I2C write
  Wire.endTransmission(); // I2C stop
//  Wire.requestFrom(D6T_addr, 5); // for 1×1
//  Wire.requestFrom(D6T_addr, 19); // for 1×8
//  Wire.requestFrom(D6T_addr, 35); // for 4×4
//  Wire.requestFrom(D6T_addr, 2051); // for 32×32
  for(int n = 0; n < 64; n++) {
    Wire.requestFrom(D6T_addr, 32, 0);
    while(Wire.available()){
      rbuf[i++] = Wire.read();
    }
  }
  Wire.requestFrom(D6T_addr, 3);
  while(Wire.available()){
    rbuf[i++] = Wire.read();
  }

  for(i=0,j=0;i<1025;i++){ // for 32x32
//  for(i=0,j=0;i<17;i++){ // for 4x4
//  for(i=0,j=0;i<9;i++){ // for 1x8
//  for(i=0,j=0;i<2;i++){ // for 1x1
    itemp = 0x00ff & rbuf[j++];
    itemp += rbuf[j++] << 8;
    if(i == 0) { // PTAT measurement mode
      t_PTAT = itemp;
      Serial.print("PTAT:");
      Serial.print(t_PTAT, 1); //print PTAT & Temperature
      //Serial.println(t_PTAT, 1); //print PTAT & Temperature
      //Serial.print(", sensor output:");
    }
    else { // each thrmopile temperature mode
      if (i % 32 == 1) {
        Serial.println("");
      }
      tdata[i-1] = itemp;
      if(i<1024){
      //if(i<16){
      //if(i<8){
      //if(i<1){
        Serial.print(tdata[i-1], 1); //print PTAT & Temperature
        Serial.print(" ");       //print space
      }
      else{
        Serial.println(tdata[i-1], 1); //print Temperature
//        Serial.println(" ");           //print space
//        Serial.print(rbuf[18], HEX); //print read PEC
//        Serial.print(" ");           //print space
//        Serial.println(crc, HEX);      //print calc PEC
      }
    }
  }
  ; // post operation.
  delay(3000);
}
// vi: ft=arduino:fdm=marker:et:sw=4:tw=80
