/*
 * THUNDIPII2C.h
 *
 *  Created on: Apr 22, 2021
 *      Author: michaeldzamba
 */

#ifndef THUNDIPII2C_H_
#define THUNDIPII2C_H_

/**************************************************************************/
/*!
    @file     Adafruit_THUNDIPII2C.h

    This is a library for the Adafruit ADS1X15 ADC breakout boards.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    Written by Kevin "KTOWN" Townsend for Adafruit Industries.

    BSD license, all text here must be included in any redistribution

    Modified by Misko for EFR32BG22
*/
/**************************************************************************/
#include <stdint.h>

#include <stdbool.h>
#include "sl_status.h"
#include "sl_i2cspm.h"


/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define THUNDIPII2C_ADDRESS (0x18)
/*=========================================================================*/

#define THUNDIPII2C_READ                            (0x01)
/**************************************************************************/
/*!
    @brief  Sensor driver for the Adafruit THUNDIPII2C ADC breakout.
*/
/**************************************************************************/


#define THUNDIPI_SLAVE_I2C_ADDRESS                     0x30 //E2
#define THUNDIPI_SLAVE_I2C_ID_OFFSET 		0x01 // 2 bytes
#define THUNDIPI_SLAVE_I2C_OUR_PASSKEY_OFFSET 	0x05 // 4 bytes
#define THUNDIPI_SLAVE_I2C_THEIR_PASSKEY_OFFSET 	0x09 // 4 bytes
#define THUNDIPI_SLAVE_I2C_MASTER_BLE_ADDR_OFFSET	0x0F
#define THUNDIPI_SLAVE_I2C_MASTER_BLE_INDICATOR_OFFSET      0x16
#define THUNDIPI_SLAVE_I2C_SLAVE_BLE_ADDR_OFFSET	0x18

struct I2C_THUNDIPII2C {
  sl_i2cspm_t *i2cspm;
  uint8_t m_i2cAddress;      ///< the I2C address
};


void sl_thundipii2c_init(struct I2C_THUNDIPII2C * sensor, uint8_t i2c_addr);
uint16_t thundipi_read_id(struct I2C_THUNDIPII2C * sensor);
void thundipi_read_slave_address(struct I2C_THUNDIPII2C * sensor, uint8_t * address);
void thundipi_write_passkey(struct I2C_THUNDIPII2C * sensor, uint32_t passkey);
uint32_t thundipi_read_passkey(struct I2C_THUNDIPII2C * sensor) ;
void thundipi_write_address(struct I2C_THUNDIPII2C * sensor, uint8_t * address);
void thundipi_slave_initI2C(void);
uint8_t * thundipi_read_master_addr();
uint8_t thundipi_read_master_indicator();
void thundipi_write_slave_addr(uint8_t * address);
uint32_t get_their_key();


#endif /* THUNDIPII2C_H_ */
