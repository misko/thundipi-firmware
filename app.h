/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H


#include "sl_bluetooth.h"


typedef struct {
    bd_addr device_address;
    uint8_t address_type;
    uint8_t connection_handle;
} connection;


enum RELAY_STATE {
  RELAY_OFF = 0,
  RELAY_ON = 1,
  RELAY_TOGGLE = 2,
  RELAY_IGNORE = 3
};

enum APP_STATE {
  IDLE,
  RELAY_SERVE,
  SWITCH_I2C_WAIT,
  SWITCH_SCAN,
  SWITCH_GET_SERVICE,
  SWITCH_GET_CHAR
};

#define PI_DEBUG 1
#define SIGNAL_AMP_NOTIFY 0x1
#define SIGNAL_RELAY_NOTIFY 0x2
#define SIGNAL_SWITCH_TOGGLE 0x4
#define SIGNAL_I2C_CHECK 0x8
#define SIGNAL_PASSKEY_CHECK 0x10

#define NRELAYS 3
#define NBUTTONS 1
#define T_RELAY 0
#define T_SWITCH 1

#define T_TYPE 0

#define PIN_SET 0
#define PIN_UNSET 1

#define RELAY_DLAY_MSEC 20
#define BUTTON_DLAY_MSEC 150
#define AMP_DLAY_MSEC 1000
#define I2C_DLAY_MSEC 5000
#define PASSKEY_DLAY_MSEC 1000
#define PASSKEY_CHECKS 10


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);





#endif // APP_H
