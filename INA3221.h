/*
 * INA3221.h
 *
 *  Created on: Apr 22, 2021
 *      Author: michaeldzamba
 */

#ifndef INA3221_H_
#define INA3221_H_

/**************************************************************************/
/*!
    @file     Adafruit_INA3221.h

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
#define INA3221_ADDRESS (0x40)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
#define INA3221_CONVERSIONDELAY (1) ///< Conversion delay
#define INA3221_CONVERSIONDELAY (9) ///< Conversion delay
/*=========================================================================*/

#define INA3221_MANUFACTURER_ID 0xFE
/*=========================================================================*/
/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define INA3221_ADDRESS                         (0x40)    // 1000000 (A0+A1=GND)
    #define INA3221_READ                            (0x01)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER (R/W)
    -----------------------------------------------------------------------*/
    #define INA3221_REG_CONFIG                      (0x00)
    /*---------------------------------------------------------------------*/
    #define INA3221_CONFIG_RESET                    (0x8000)  // Reset Bit

    #define INA3221_CONFIG_ENABLE_CHAN1             (0x4000)  // Enable Channel 1
    #define INA3221_CONFIG_ENABLE_CHAN2             (0x2000)  // Enable Channel 2
    #define INA3221_CONFIG_ENABLE_CHAN3             (0x1000)  // Enable Channel 3

    #define INA3221_CONFIG_AVG2                     (0x0800)  // AVG Samples Bit 2 - See table 3 spec
    #define INA3221_CONFIG_AVG1                     (0x0400)  // AVG Samples Bit 1 - See table 3 spec
    #define INA3221_CONFIG_AVG0                     (0x0200)  // AVG Samples Bit 0 - See table 3 spec

    #define INA3221_CONFIG_VBUS_CT2                 (0x0100)  // VBUS bit 2 Conversion time - See table 4 spec
    #define INA3221_CONFIG_VBUS_CT1                 (0x0080)  // VBUS bit 1 Conversion time - See table 4 spec
    #define INA3221_CONFIG_VBUS_CT0                 (0x0040)  // VBUS bit 0 Conversion time - See table 4 spec

    #define INA3221_CONFIG_VSH_CT2                  (0x0020)  // Vshunt bit 2 Conversion time - See table 5 spec
    #define INA3221_CONFIG_VSH_CT1                  (0x0010)  // Vshunt bit 1 Conversion time - See table 5 spec
    #define INA3221_CONFIG_VSH_CT0                  (0x0008)  // Vshunt bit 0 Conversion time - See table 5 spec

    #define INA3221_CONFIG_MODE_2                   (0x0004)  // Operating Mode bit 2 - See table 6 spec
    #define INA3221_CONFIG_MODE_1                   (0x0002)  // Operating Mode bit 1 - See table 6 spec
    #define INA3221_CONFIG_MODE_0                   (0x0001)  // Operating Mode bit 0 - See table 6 spec

/*=========================================================================*/

/*=========================================================================
    SHUNT VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
    #define INA3221_REG_SHUNTVOLTAGE_1                (0x01)
/*=========================================================================*/

/*=========================================================================
    BUS VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
    #define INA3221_REG_BUSVOLTAGE_1                  (0x02)

/*=========================================================================*/

/**************************************************************************/
/*!
    @brief  Sensor driver for the Adafruit INA3221 ADC breakout.
*/
/**************************************************************************/



struct I2C_INA3221 {
  sl_i2cspm_t *i2cspm;
  uint8_t m_i2cAddress;      ///< the I2C address
  float shunt_resistor_value;
};


void sl_ina3221_init(struct I2C_INA3221 * sensor, uint8_t i2c_addr, float shunt_resistor_value);

double INA3221_getCurrentA(struct I2C_INA3221 * sensor, uint8_t channel);
double INA3221_getBusVoltageV(struct I2C_INA3221 * sensor, uint8_t channel);
double INA3221_getShuntVoltageV(struct I2C_INA3221 * sensor, uint8_t channel);




#endif /* INA3221_H_ */
