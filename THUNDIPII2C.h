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
#define THUNDIPI_ID_REGISTER 0x01
/**************************************************************************/
/*!
    @brief  Sensor driver for the Adafruit THUNDIPII2C ADC breakout.
*/
/**************************************************************************/



struct I2C_THUNDIPII2C {
  sl_i2cspm_t *i2cspm;
  uint8_t m_i2cAddress;      ///< the I2C address
};


void sl_thundipii2c_init(struct I2C_THUNDIPII2C * sensor, uint8_t i2c_addr);





#endif /* THUNDIPII2C_H_ */
