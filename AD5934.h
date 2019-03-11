#include "Arduino.h"
#include "Wire.h"



/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define AD5934_ADDRESS                 (0x0d)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define AD5934_REG_POINTER_CNT1        (0x80)
    #define AD5934_REG_POINTER_CNT2        (0x81)
    #define AD5934_REG_POINTER_sFreq1      (0x82)
    #define AD5934_REG_POINTER_sFreq2      (0x83)
    #define AD5934_REG_POINTER_sFreq3      (0x84)
    #define AD5934_REG_POINTER_freqInc1    (0x85)
    #define AD5934_REG_POINTER_freqInc2    (0x86)
    #define AD5934_REG_POINTER_freqInc3    (0x87)
    #define AD5934_REG_POINTER_numInc1     (0x88)
    #define AD5934_REG_POINTER_numInc2     (0x89)
    #define AD5934_REG_POINTER_numSet1     (0x8a)
    #define AD5934_REG_POINTER_numSet2     (0x8b)
    #define AD5934_REG_POINTER_STATUS      (0x88)
    #define AD5934_REG_POINTER_rData1      (0x94)
    #define AD5934_REG_POINTER_rData2      (0x95)
    #define AD5934_REG_POINTER_iData1      (0x96)
    #define AD5934_REG_POINTER_iData2      (0x97)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define AD5934_REG_CNT1_MASK         (0x07)
    #define AD5934_REG_CNT1_INITFREQ     (0x10)
    #define AD5934_REG_CNT1_STARTFREQ    (0x20)
    #define AD5934_REG_CNT1_INCRFREQ     (0x30)
    #define AD5934_REG_CNT1_REPFREQ      (0x40)
    #define AD5934_REG_CNT1_POWERDOWN    (0xa0)
    #define AD5934_REG_CNT1_STANDBY      (0xb0)
    #define AD5934_REG_CNT1_GAIN1        (0x01)
    #define AD5934_REG_CNT1_GAIN5        (0x00)
    #define AD5934_REG_CNT1_2000         (0x00)
    #define AD5934_REG_CNT1_200          (0x02)
    #define AD5934_REG_CNT1_400          (0x04)
    #define AD5934_REG_CNT1_1000         (0x06)

    #define AD5934_REG_CNT2_MASK         (0x08)
    #define AD5934_REG_CNT2_RESET        (0x10)

/*=========================================================================*/

typedef enum
{
  EXV2000         = AD5934_REG_CNT1_2000,
  EXV200          = AD5934_REG_CNT1_200,
  EXV400          = AD5934_REG_CNT1_400,
  EXV1000         = AD5934_REG_CNT1_1000
} adExVolt;
typedef enum
{
  GAIN_ONE        = AD5934_REG_CNT1_GAIN1,
  GAIN_FIVE       = AD5934_REG_CNT1_GAIN5
} adGain;

class AD5934
{
protected:
   // Instance-specific properties
   uint8_t   m_i2cAddress;
   uint8_t   m_conversionDelay;
   uint32_t  m_STARTFREQ
   uint32_t  m_freqInc
   uint16_t  m_numInc
   uint8_t   m_exVolt;
   adGain    m_gain;
   uint16_t  m_numSet;

 public:
  AD5934(uint8_t i2cAddress = AD5934_ADDRESS);
  void begin(void);
  uint16_t  readImpedance();
  void      setGAIN(adGain m_gain);
  void      setExVolt(adExVolt m_exVolt);
  void      setSTARTFREQ(uint32_t  m_STARTFREQ);
  void      setFreqInc(uint32_t  m_freqInc);
  void      setNumInc(uint16_t  m_numInc);
  void      setNumSet(uint16_t  m_numSet);


 private:
};
