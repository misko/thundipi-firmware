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
  RELAY_PAIR,
  RELAY_CONFIRM,
  SWITCH_I2C_WAIT,
  SWITCH_SCAN,
  SWITCH_GET_SERVICE,
  SWITCH_GET_CHAR,
  SWITCH_SERVE,
};

enum I2C_THUNDI_STATE {
	I2C_THUNDI_CONNECTED,
	I2C_THUNDI_DISCONNECTED
};

#define PI_DEBUG 1
#define SIGNAL_AMP_NOTIFY 0x1
#define SIGNAL_RELAY_NOTIFY 0x2
#define SIGNAL_SWITCH_TOGGLE 0x4
#define SIGNAL_I2C_CHECK 0x8
#define SIGNAL_PASSKEY_CHECK 0x10
#define SIGNAL_MONITOR 0x20
#define SIGNAL_NVM_SAVE 0x40
#define SIGNAL_PRESS_HOLD 0x80
#define SIGNAL_SETUP_TIMEOUT 0x0100
#define SIGNAL_PASSKEY_ACCEPT 0x0200
#define SIGNAL_DFU_HOLD 0x0400

#define NRELAYS 3
#define NBUTTONS 1
#define T_RELAY 0
#define T_SWITCH 1

#define T_TYPE 0

#define PIN_SET 0
#define PIN_UNSET 1


#define PASSKEY_CHECKS 10

extern int app_state;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);
void set_discoverable();
void unset_discoverable();




#endif // APP_H
