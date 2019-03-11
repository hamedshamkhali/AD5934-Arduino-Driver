#include "Arduino.h"
#include "Wire.h"
#include "AD5934.h"



/**************************************************************************/
/*
    i2c read
*/
/**************************************************************************/
static uint8_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.requestFrom(i2cAddress, (uint8_t)1);
  return Wire.read();
}


/**************************************************************************/
/*
    i2c write
*/
/**************************************************************************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
}


/**************************************************************************/
/*
    instantiates a new AD5934 class w/appropriate properties
*/
/**************************************************************************/
AD5934::AD5934(uint8_t i2cAddress)
{
   m_i2cAddress = i2cAddress;
   m_gain = GAIN_ONE;
   m_exVolt = EXV200;
}


/**************************************************************************/
/*
    initializ i2c
*/
/**************************************************************************/
void AD5934::begin() {
  Wire.begin();
}

/**************************************************************************/
/*
    sets the gain
*/
/**************************************************************************/
void AD5934::setGain(adsGain_t gain)
{
  m_gain = gain;
}

/**************************************************************************/
/*
    sets the excitation voltage
*/
/**************************************************************************/
void AD5934::setExVolt(adExVolt exVolt)
{
  m_exVolt = exVolt;
}

/**************************************************************************/
/*
    sets the start frequency
*/
/**************************************************************************/
void AD5934::setstratFreq(uint32_t  stratFreq)
{
  m_stratFreq = (uint32_t)(stratFreq*16/1000000*(pow(2,27)));
}

/**************************************************************************/
/*
    sets the  frequency increaments
*/
/**************************************************************************/
void AD5934::setFreqInc(uint32_t  freqInc)
{
  m_freqInc = (uint32_t)(freqInc*16/1000000*(pow(2,27)));
}

/**************************************************************************/
/*
    sets the  number of settling time
*/
/**************************************************************************/
void AD5934::setNumInc(uint16_t  numInc)
{
  m_numInc = numInc;
}

/**************************************************************************/
/*
    number of settling time
*/
/**************************************************************************/
void AD5934::setNumInc(uint16_t  numSet)
{
  m_numSet = numSet;
}

/**************************************************************************/
/*
    Gets impedance readings
*/
/**************************************************************************/
struct AD5934::readImpedance_frequencySweep() {

   // Reset
  uint16_t cnt2 =   AD5934_REG_CNT2_MASK   |  AD5934_REG_POINTER_numSet2
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_CNT2, cnt2);

  //  output struct
  struct output{
    int imagImp[m_numInc];
    int realImp[m_numInc];
    int freq[m_numInc];
  };

  // Start with default values
  uint16_t cnt1 =   AD5934_REG_CNT1_STANDBY   | // STANDBY
                    m_exVolt   | //excitation voltage
                    m_gain;      // Gain
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_CNT1, cnt1);

  // start frequency register
  uint8_t value = m_stratFreq;
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_sFreq3, value);
  uint8_t value = (m_stratFreq >> 8);
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_sFreq12, value);
  uint8_t value = (m_stratFreq >> 16);
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_sFreq1, value);

  //  frequency increaments register
  uint8_t value = m_freqInc;
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_freqInc3, value);
  uint8_t value = (m_freqInc >> 8);
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_freqInc2, value);
  uint8_t value = (m_freqInc >> 16);
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_freqInc1, value);

  //  number of increaments
  uint8_t value = m_numInc;
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_numInc2, value);
  uint8_t value = (m_numInc >> 8);
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_numInc1, value);

  //  number of settlin time
  uint8_t value = m_numSet;
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_numSet2, value);
  uint8_t value = (m_numSet >> 8);
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_numSet1, value);

  // initializ frequency
  cnt1 &= AD5934_REG_CNT1_MASK;
  cnt1 &= AD5934_REG_CNT1_INITFREQ;
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_CNT1, cnt1);
  uint8_t status= readRegister(m_i2cAddress, AD5934_REG_POINTER_STATUS);

  // real and imaginary data
  uint16_t r=0,i=0,cnt=1;
  struct output results;

  // Start frequency sweep
  cnt1 &= AD5934_REG_CNT1_MASK;
  cnt1 &= AD5934_REG_CNT1_STARTFREQ;
  writeRegister(m_i2cAddress, AD5934_REG_POINTER_CNT1, cnt1);
  status= readRegister(m_i2cAddress, AD5934_REG_POINTER_STATUS);

  while (status != 118){

            while ( (status != 114) && (status != 118)){
                status= readRegister(m_i2cAddress, AD5934_REG_POINTER_STATUS);
              }

            if (status != 118){

              r = r readRegister(m_i2cAddress, AD5934_REG_POINTER_rData1);
              r= r << 8;
              r = r | readRegister(m_i2cAddress, AD5934_REG_POINTER_rData2);

              i = i readRegister(m_i2cAddress, AD5934_REG_POINTER_iData1);
              i= i << 8;
              i = i | readRegister(m_i2cAddress, AD5934_REG_POINTER_iData2);

              results.realImp[cnt]=r;
              results.imagImp[cnt]=i;
              results.freq[cnt]=m_stratFreq*cnt;

              cnt=cnt+1;
              }
    }

}
