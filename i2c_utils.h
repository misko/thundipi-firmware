#include <stddef.h>
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "app.h"

sl_status_t i2c_read_data(sl_i2cspm_t *i2cspm, uint8_t i2c_addr,
		uint8_t reg, uint16_t *data);
sl_status_t sl_INA3221_write_data(sl_i2cspm_t *i2cspm,
		uint8_t i2c_addr, uint8_t reg, int16_t value);

#if T_TYPE == T_SWITCH
volatile bool thundi_pi_slave_i2c_gotTargetAddress;
volatile bool thundi_pi_slave_i2c_rxInProgress;
void thundipi_slave_initI2C(void);
#endif
