/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
#include <ads1115.h>
#include "em_common.h"
#include "em_gpio.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_i2cspm.h"
#include "ads1115.h"
#include "gatt_db.h"
#include "app.h"
#include "em_i2c.h"
#include "sl_i2cspm_instances.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static struct I2C_ADS1115 ads1115_sensor;




/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  ///
  ///
  /// This works with existing board
  //GPIO_PinModeSet(gpioPortA,7,  gpioModeWiredOrPullDown,0);
  //GPIO_PinModeSet(gpioPortA,8,  gpioModeWiredOrPullDown,0);
  //GPIO_PinModeSet(gpioPortC,6,  gpioModeWiredOrPullDown,0);
  //GPIO_PinModeSet(gpioPortC,7,  gpioModeWiredOrPullDown,0);

  //Used as of March 26 2021
  //GPIO_PinModeSet(gpioPortA,7,  gpioModeWiredAnd,0);
  //GPIO_PinModeSet(gpioPortA,8,  gpioModeWiredAnd,0);
  //GPIO_PinModeSet(gpioPortC,6,  gpioModeWiredAnd,0);
  //GPIO_PinModeSet(gpioPortC,7,  gpioModeWiredAnd,0);

  GPIO_PinModeSet(gpioPortA,7,  gpioModeWiredAndPullUp,1);
  GPIO_PinModeSet(gpioPortA,8,  gpioModeWiredAndPullUp,1);
  GPIO_PinModeSet(gpioPortC,6,  gpioModeWiredAndPullUp,1);
  GPIO_PinModeSet(gpioPortC,7,  gpioModeWiredAndPullUp,1);

  sl_ads_init(&ads1115_sensor);

}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
float read_voltage(int channel) {
  uint16_t v0 = ads1115_readADC_SingleEnded(&ads1115_sensor, channel);
  return v0*(4.096/(1<<15));
}

float read_current(int channel) {
  float ratio = (2000.0+3300.0)/3300.0;
  float vout = read_voltage(channel)*ratio;
  float current = (vout-1.65)/0.044;
  printf("VOUT %f , current %f\r\n\n",vout,current);
}
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
  ///
  /*uint16_t v0 = ads1115_readADC_SingleEnded(&ads1115_sensor, 0);
  uint16_t v1 = ads1115_readADC_SingleEnded(&ads1115_sensor, 1);
  uint16_t v2 = ads1115_readADC_SingleEnded(&ads1115_sensor, 2);
  uint16_t v3 = ads1115_readADC_SingleEnded(&ads1115_sensor, 3);*/
  float v0 = read_voltage(0);
  float v1 = read_voltage(1);
  float v2 = read_voltage(2);
  float v3 = read_voltage(3);
  float c0 = read_current(0);
  //printf("%f %f %f %f\r\n\n",v0,v1,v2,v3);
}

static void aio_system_boot_cb(void)
{
  sl_status_t sc;
  uint8_t value_out = 1;
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_number_of_digitalsA, 0, 1, &value_out);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to write attribute value\n",
                (int)sc);
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_number_of_digitalsB, 0, 1, &value_out);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to write attribute value\n",
                (int)sc);
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_number_of_digitalsC, 0, 1, &value_out);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to write attribute value\n",
                (int)sc);
  sc = sl_bt_gatt_server_write_attribute_value(gattdb_number_of_digitalsD, 0, 1, &value_out);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to write attribute value\n",
                (int)sc);
}


static void aio_connection_opened_cb(sl_bt_evt_connection_opened_t *data)
{
  (void)data;
}

static void aio_connection_closed_cb(sl_bt_evt_connection_closed_t *data)
{
  (void)data;
}



static void aio_digital_out_read_cb(sl_bt_evt_gatt_server_user_read_request_t *data,int port, int pin)
{
  sl_status_t sc;
  uint8_t value =GPIO_PinOutGet(port,pin);
  sc = sl_bt_gatt_server_send_user_read_response(
    data->connection,
    data->characteristic,
    0,
    1,
    &value,
    NULL);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read response\n",
                (int)sc);
}


static void aio_digital_out_write_cb(sl_bt_evt_gatt_server_user_write_request_t *data,int port, int pin)
{
  sl_status_t sc;
  uint8_t att_errorcode = 0;

  if (data->value.len == 1) {
    if (data->value.data[0]==0) {
        GPIO_PinOutClear(port,pin);
    } else if (data->value.data[0]==1) {
        GPIO_PinOutSet(port,pin);
    } else {
        printf("Weird value to write to pin!\n");
    }
  } else {
    printf("Weird value to write to pin 2!\n");
    att_errorcode = 0x0D; // Invalid Attribute Value Length
  }

  sc = sl_bt_gatt_server_send_user_write_response(data->connection,
                                                  data->characteristic,
                                                  att_errorcode);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user write response\n",
                (int)sc);
}

static uint8_t _conn_handle = 0xFF;
static uint8_t _bonding_handle = 0xFF;
void sl_bt_aio_process(sl_bt_msg_t *evt) {
  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      aio_system_boot_cb();
      sl_bt_sm_configure(0x0F, sm_io_capability_displayyesno);
      sl_bt_sm_set_bondable_mode(1);
      break;

    case sl_bt_evt_sm_confirm_bonding_id:
      printf("CONFIRM BOND\n");
      sl_bt_sm_bonding_confirm(_conn_handle,1);
      break;

    case sl_bt_evt_sm_passkey_display_id:
      printf("Passkey: %06lu", evt->data.evt_sm_passkey_display.passkey);
      break;

    // Event raised by the security manager when a passkey needs to be confirmed
    case sl_bt_evt_sm_confirm_passkey_id:
      printf("Do you see this passkey on the other device: %06lu? (y/n)\r\n\n", evt->data.evt_sm_confirm_passkey.passkey);
      sl_bt_sm_passkey_confirm(_conn_handle, 1);
      //sl_bt_sm_passkey_confirm(_conn_handle, 0); //no
      break;

      // Event raised when bonding is successful
      case sl_bt_evt_sm_bonded_id:
        printf("Bonded\r\n\n");
        printf("--------------------------------------\r\n\n");
        printf("** SPP Mode ON **\r\n\n");
       // _main_state = STATE_SPP_MODE;

        //SLEEP_SleepBlockBegin(sleepEM2);  // disable sleeping when SPP mode active
        sl_bt_connection_close(_conn_handle);
        break;

      // Event raised when bonding failed
      case sl_bt_evt_sm_bonding_failed_id:
        printf("Bonding failed\r\n\n");
        printf("--------------------------------------\r\n\n");
        //gecko_cmd_sm_increase_security(_conn_handle);
        break;

    case sl_bt_evt_connection_opened_id:
      aio_connection_opened_cb(&evt->data.evt_connection_opened);
      _conn_handle = evt->data.evt_connection_opened.connection;
      _bonding_handle = evt->data.evt_connection_opened.bonding;
      printf("Connection open\n");
      break;

    case sl_bt_evt_connection_closed_id:
      aio_connection_closed_cb(&evt->data.evt_connection_closed);
      printf("Connection closed\n");
      break;

    case sl_bt_evt_gatt_server_user_read_request_id:
      switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
        case gattdb_digitalA:
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortA,8);
          break;
        case gattdb_digitalB:
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortA,7);
          break;
        case gattdb_digitalC:
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortC,6);
          break;
        case gattdb_digitalD:
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortC,7);
          break;
      }
      break;

    /*case sl_bt_evt_gatt_server_characteristic_status_id:
      if ((gattdb_aio_digital_in == evt->data.evt_gatt_server_characteristic_status.characteristic)
          && (gatt_server_client_config == (gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags)) {
        // client characteristic configuration changed by remote GATT client
        aio_digital_in_changed_cb(&evt->data.evt_gatt_server_characteristic_status);
      }
      break;*/

    case sl_bt_evt_gatt_server_user_write_request_id:
      switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
        case gattdb_digitalA:
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortA,8);
          break;
        case gattdb_digitalB:
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortA,7);
          break;
        case gattdb_digitalC:
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortC,6);
          break;
        case gattdb_digitalD:
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortC,7);
          break;
      }
      break;
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

    case sl_bt_evt_gatt_server_user_write_request_id:
        printf("WRITE\n");
        break;

    case sl_bt_evt_gatt_server_user_read_request_id:
        printf("READ\n");
        break;



    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
  sl_bt_aio_process(evt);
}
