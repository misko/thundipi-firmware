
#include <stddef.h>
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "INA3221.h"

/**************************************************************************/
/*!
    @file     INA3221.c


*/
/**************************************************************************/

static sl_status_t sl_INA3221_read_data(struct I2C_INA3221 * sensor, uint8_t i2c_addr, uint8_t reg, uint16_t *data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  i2c_read_data[0]=0;
  i2c_read_data[1]=0;
  uint8_t                    i2c_write_data[1];
  i2c_write_data[0]=reg;

  seq.addr  = i2c_addr<<1;
  //seq.flags = I2C_FLAG_WRITE_READ;
  seq.flags = I2C_FLAG_WRITE;
  /* Select command to issue */
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1;

  ret = I2CSPM_Transfer(sensor->i2cspm, &seq);

  if (ret != i2cTransferDone) { // check if i2cTransferDone
    *data = 0;
    return ret;
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
    return ret;
  }

  //*data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);
  *data = (i2c_read_data[0] << 8) + (i2c_read_data[1]);

  return SL_STATUS_OK;
}


/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library

    @param x byte to write
*/
/**************************************************************************/
static sl_status_t sl_INA3221_write_data(struct I2C_INA3221 * sensor, uint8_t i2c_addr, uint8_t reg, int16_t value)
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

  if (ret!=i2cTransferDone) {
      retval = SL_STATUS_TRANSMIT;
  }
  return retval;
}

sl_status_t sl_ina3221_set_config(struct I2C_INA3221 * sensor)  {

  // Set Config register to take into account the settings above
  uint16_t config = INA3221_CONFIG_ENABLE_CHAN1 |
                    INA3221_CONFIG_ENABLE_CHAN2 |
                    INA3221_CONFIG_ENABLE_CHAN3 |
                    INA3221_CONFIG_AVG1 |
                    INA3221_CONFIG_AVG0 |
                    INA3221_CONFIG_VBUS_CT2 |
                    INA3221_CONFIG_VSH_CT2 |
                    INA3221_CONFIG_MODE_2 |
                    INA3221_CONFIG_MODE_1 |
                    INA3221_CONFIG_MODE_0;
  return sl_INA3221_write_data(sensor, sensor->m_i2cAddress, INA3221_REG_CONFIG, config);

}

void sl_ina3221_init(struct I2C_INA3221 * sensor, uint8_t i2c_addr, float shunt_resistor_value) {
  sensor->i2cspm = sl_i2cspm_INA3221;
  sensor->shunt_resistor_value=shunt_resistor_value;
  sensor->m_i2cAddress=i2c_addr;

  sl_ina3221_set_config(sensor);
}


/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/

int16_t from_twos(uint16_t twos) {
  if((twos>>15 && 0x01) == 1)   {
      twos = ~(twos & 0x7FFF) + 1;
  }
  return twos;
}

double INA3221_getShuntVoltageV(struct I2C_INA3221 * sensor, uint8_t channel) {
  uint16_t value;

  sl_status_t ret=sl_INA3221_read_data(sensor, sensor->m_i2cAddress, INA3221_REG_SHUNTVOLTAGE_1+(channel -1) *2, &value);

  if (ret != SL_STATUS_OK) {
	  return -1.0;
  }
  double value_f=value>>3;
  return value_f*0.000004;
}


/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/

double INA3221_getBusVoltageV(struct I2C_INA3221 * sensor, uint8_t channel) {
  uint16_t value;

  sl_status_t ret=sl_INA3221_read_data(sensor, sensor->m_i2cAddress, INA3221_REG_BUSVOLTAGE_1+(channel -1) *2, &value);

  if (ret != SL_STATUS_OK) {
      return -1.0;
  }

  double value_f=value>>3;
  return value_f*0.008;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
double INA3221_getCurrentA(struct I2C_INA3221 * sensor, uint8_t channel) {
    return INA3221_getShuntVoltageV(sensor, channel)/sensor->shunt_resistor_value;
}



