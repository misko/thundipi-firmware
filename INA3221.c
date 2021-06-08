
#include <stddef.h>
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "INA3221.h"
#include "i2c_utils.h"
#include <machine/endian.h>

/**************************************************************************/
/*!
    @file     INA3221.c
*/
/**************************************************************************/

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
  sl_status_t sc= i2c_write_data_uint16(sensor->i2cspm, sensor->m_i2cAddress, INA3221_REG_CONFIG, __htons(config));
  return sc;

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


float INA3221_getShuntVoltage_mV(struct I2C_INA3221 * sensor, uint8_t channel) {
  int16_t value;

  sl_status_t ret=i2c_read_data_uint16(sensor->i2cspm, sensor->m_i2cAddress, INA3221_REG_SHUNTVOLTAGE_1+(channel -1) *2, &value);

  if (ret != SL_STATUS_OK) {
	  return -1.0;
  }

  double value_f=(double)(value>>3); // last three bits are not useful
  value_f*=0.04; // 40uV is LSB, 40uV = 0.04mV
  return value_f;
}


/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/

float INA3221_getBusVoltageV(struct I2C_INA3221 * sensor, uint8_t channel) {
  uint16_t value;

  sl_status_t ret=i2c_read_data_uint16(sensor->i2cspm, sensor->m_i2cAddress, INA3221_REG_BUSVOLTAGE_1+(channel -1) *2, &value);

  if (ret != SL_STATUS_OK) {
      return -1.0;
  }

  double value_f=(double)(value>>3); // last three bits are not useful
  value_f*=0.008;  // LSB 8mV
  return value_f;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
float INA3221_getCurrentA(struct I2C_INA3221 * sensor, uint8_t channel) {
	float shunt_voltage_mv=INA3221_getShuntVoltage_mV(sensor, channel);
	float current_mA =  shunt_voltage_mv/sensor->shunt_resistor_value;
	//printf("Current %d \r\n\n",(int)(current_mA));
    return current_mA;
}



