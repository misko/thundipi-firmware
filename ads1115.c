
#include <stddef.h>
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "stddef.h"
#include "ads1115.h"
#include "em_i2c.h"
#include "sl_i2cspm_instances.h"

/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.cpp
    @author   K.Townsend (Adafruit Industries)

    @mainpage Adafruit ADS1X15 ADC Breakout Driver

    @section intro_sec Introduction

    This is a library for the Adafruit ADS1X15 ADC breakout boards.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section author Author

    Written by Kevin "KTOWN" Townsend for Adafruit Industries.

    @section  HISTORY

    v1.0  - First release
    v1.1  - Added ADS1115 support - W. Earl

    @section license License

    BSD license, all text here must be included in any redistribution

    Modified by Misko
*/
/**************************************************************************/

static sl_status_t sl_ads1115_read_data(struct I2C_ADS1115 * sensor, uint8_t i2c_addr, uint8_t reg, uint16_t *data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  i2c_read_data[0]=0;
  i2c_read_data[1]=0;

  seq.addr  = i2c_addr << 1;
  //seq.flags = I2C_FLAG_WRITE_READ;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  seq.buf[0].data = reg;
  //THIS SHOULD BE ADDY OF REG? TODO
  seq.buf[0].len  = 1;

  ret = I2CSPM_Transfer(sensor->i2cspm, &seq);

  if (ret != i2cTransferDone) {
    *data = 0;
    return SL_STATUS_TRANSMIT;
  }

  seq.addr  = i2c_addr << 1;
  //seq.flags = I2C_FLAG_WRITE_READ;
  seq.flags = I2C_FLAG_READ;
  /* Select command to issue */
  /* Select location/length of data to be read */
  seq.buf[0].data = i2c_read_data;
  seq.buf[0].len  = 2;

  ret = I2CSPM_Transfer(sensor->i2cspm, &seq);

  if (ret != i2cTransferDone) {
    *data = 0;
    return SL_STATUS_TRANSMIT;
  }

  //*data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);
  *data = (i2c_read_data[0] << 8) + (i2c_read_data[1]);

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * @brief  Reading I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
/*int ReadI2C(I2C_TypeDef *i2c,uint8_t Address, uint8_t Register, uint8_t *Buffer, int BufferSize)
{
  int ret;

  ret = WriteI2CByte(i2c,Address,Register);
  if (ret != i2cTransferDone)
    return (int)ret;

  I2C_TransferSeq_TypeDef    seq;
      seq.addr  = 0x00ff & Address;
      seq.flags = I2C_FLAG_READ;


      seq.buf[0].data = Buffer;
      seq.buf[0].len  = BufferSize;

      ret = I2CSPM_Transfer(i2c, &seq);
      if (ret != i2cTransferDone)
      {
        return((int) ret);
      }

      return 0;
}*/


/**************************************************************************//**
 * @brief  Transmitting I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
/*int WriteI2CByte(I2C_TypeDef *i2c,uint8_t Address, uint8_t value)
{
  uint8_t buffer[1];

  int ret;

  buffer[0] =  value;


  I2C_TransferSeq_TypeDef    seq;
      seq.addr  = 0x00ff & Address;
      seq.flags = I2C_FLAG_WRITE;


      seq.buf[0].data = buffer;
      seq.buf[0].len  = 1;

      ret = I2CSPM_Transfer(i2c, &seq);
      if (ret != i2cTransferDone)
      {
        return((int) ret);
      }

      return 0;
}*/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library

    @param x byte to write
*/
/**************************************************************************/
static sl_status_t sl_ads1115_write_data(struct I2C_ADS1115 * sensor, uint8_t i2c_addr, uint8_t reg, int16_t value)
{
  sl_status_t retval = SL_STATUS_OK;

  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[3];

  seq.addr  = i2c_addr << 1;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  i2c_write_data[0] = reg;
  i2c_write_data[1] = value >> 8;
  i2c_write_data[2] = value & 0xFF;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 3;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CSPM_Transfer(sensor->i2cspm, &seq);

  if (ret != i2cTransferDone) {
    retval = SL_STATUS_TRANSMIT;
  }

  return retval;
}


void sl_ads_init(struct I2C_ADS1115 * sensor)
{
  sl_status_t sc;
  sensor->i2cspm = sl_i2cspm_ads1115; //sl_sensor_select(SL_BOARD_SENSOR_RHT);

  sensor->m_conversionDelay = ADS1115_CONVERSIONDELAY;
  sensor->m_bitShift = 0;
  sensor->m_gain = GAIN_ONE; //GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
  //(void)sl_board_enable_sensor(SL_BOARD_SENSOR_RHT);
  //sl_app_assert(NULL != rht_sensor, "Si70xx sensor not available\n");
  //sc = sl_si70xx_init(rht_sensor, RHT_ADDRESS);
  //sl_app_assert(SL_STATUS_OK == sc, "[E: %#04x] Si70xx sensor init failed\n", sc);
}

/**************************************************************************/
/*!
    @brief  Writes 16-bits to the specified destination register

    @param i2cAddress I2C address of device
    @param reg register address to write to
    @param value value to write to register
*/
/**************************************************************************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  /*Wire.beginTransmission(i2cAddress);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value >> 8));
  i2cwrite((uint8_t)(value & 0xFF));
  Wire.endTransmission();*/
}

/**************************************************************************/
/*!
    @brief  Read 16-bits from the specified destination register

    @param i2cAddress I2C address of device
    @param reg register address to read from

    @return 16 bit register value read
*/
/**************************************************************************/
static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  /*Wire.beginTransmission(i2cAddress);
  i2cwrite(reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  return ((i2cread() << 8) | i2cread());*/
}



/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel

    @param channel ADC channel to read

    @return the ADC reading
*/
/**************************************************************************/
uint16_t ads1115_readADC_SingleEnded(struct I2C_ADS1115 * sensor, uint8_t channel) {
  if (channel > 3) {
    return 0;
  }

  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= sensor->m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  //writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
  sl_ads1115_write_data(sensor,ADS1015_ADDRESS,ADS1015_REG_POINTER_CONFIG,config);

  // Wait for the conversion to complete
  //delay(m_conversionDelay);

  sl_sleeptimer_delay_millisecond(sensor->m_conversionDelay); //*10);

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
 //return readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;

  uint16_t data;
  sl_ads1115_read_data(sensor, ADS1015_ADDRESS,ADS1015_REG_POINTER_CONVERT, &data);

  //static sl_status_t sl_ads1115_read_data(sl_i2cspm_t *i2cspm, uint8_t addr, uint32_t *data)
  return data;
}

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN0) and N (AIN1) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
/*int16_t Adafruit_ADS1015::readADC_Differential_0_1() {
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_0_1; // AIN0 = P, AIN1 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res =
      readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}*/

/**************************************************************************/
/*!
    @brief  Reads the conversion results, measuring the voltage
            difference between the P (AIN2) and N (AIN3) input.  Generates
            a signed value since the difference can be either
            positive or negative.

    @return the ADC reading
*/
/**************************************************************************/
/*int16_t Adafruit_ADS1015::readADC_Differential_2_3() {
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set channels
  config |= ADS1015_REG_CONFIG_MUX_DIFF_2_3; // AIN2 = P, AIN3 = N

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res =
      readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}*/

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.

    @param channel ADC channel to use
    @param threshold comparator threshold
*/
/**************************************************************************/
/*void Adafruit_ADS1015::startComparator_SingleEnded(uint8_t channel,
                                                   int16_t threshold) {
  // Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1
                                        // match
      ADS1015_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_REG_CONFIG_CMODE_TRAD |   // Traditional comparator (default val)
      ADS1015_REG_CONFIG_DR_1600SPS |   // 1600 samples per second (default)
      ADS1015_REG_CONFIG_MODE_CONTIN |  // Continuous conversion mode
      ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_HITHRESH,
                threshold << m_bitShift);

  // Write config register to the ADC
  writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
}*/

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.

    @return the last ADC reading
*/
/**************************************************************************/
/*nt16_t Adafruit_ADS1015::getLastConversionResults() {
  // Wait for the conversion to complete
  delay(m_conversionDelay);

  // Read the conversion results
  uint16_t res =
      readRegister(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
  if (m_bitShift == 0) {
    return (int16_t)res;
  } else {
    // Shift 12-bit results right 4 bits for the ADS1015,
    // making sure we keep the sign bit intact
    if (res > 0x07FF) {
      // negative number - extend the sign to 16th bit
      res |= 0xF000;
    }
    return (int16_t)res;
  }
}*/
