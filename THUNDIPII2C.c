
#include <stddef.h>
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "THUNDIPII2C.h"

/**************************************************************************/
/*!
    @file     THUNDIPII2C.c
*/
/**************************************************************************/


uint16_t thundipi_read_id(struct I2C_THUNDIPII2C * sensor) {
  uint16_t value;

  sl_status_t ret=i2c_read_data(sensor->i2cspm, sensor->m_i2cAddress, THUNDIPI_ID_REGISTER, &value);

  if (ret != SL_STATUS_OK) {
	  return 0;
  }
  return value;
}

void sl_thundipii2c_init(struct I2C_THUNDIPII2C * sensor, uint8_t i2c_addr) {
  sensor->i2cspm = sl_i2cspm_THUNDIPII2C;
  sensor->m_i2cAddress=i2c_addr;

  sl_ina3221_set_config(sensor);
}



