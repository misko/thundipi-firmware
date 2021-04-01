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

  GPIO_PinModeSet(gpioPortA,7,  gpioModeWiredAndPullUp,0);
  GPIO_PinModeSet(gpioPortA,8,  gpioModeWiredAndPullUp,0);
  GPIO_PinModeSet(gpioPortC,6,  gpioModeWiredAndPullUp,0);
  GPIO_PinModeSet(gpioPortC,7,  gpioModeWiredAndPullUp,0);

  sl_ads_init(&ads1115_sensor);

}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/

void channel_to_port_and_pin(int channel, int * port, int * pin) {
  switch(channel) {
    case 0:
      *port=gpioPortA;
      *pin=7;
      break;
    case 1:
      *port=gpioPortA;
      *pin=8;
      break;
    case 2:
      *port=gpioPortC;
      *pin=7;
      break;
    case 3:
      *port=gpioPortC;
      *pin=6;
      break;
    default:
      sl_app_assert(1==0, "[E: 0x%04x] Invalid channel\n", channel);
  }
}
float read_voltage(int channel) {
  uint16_t v0 = ads1115_readADC_SingleEnded(&ads1115_sensor, channel);
  return v0*(4.096/(1<<15));
}

float read_current(int channel) {
  float ratio = (2000.0+3300.0)/3300.0;
  float vout = read_voltage(channel)*ratio;
  float current = (vout-1.65)/0.044;
  //printf("VOUT %f , current %f\r\n\n",vout,current);
  return current;
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
  //float v0 = read_voltage(0);
  //float v1 = read_voltage(1);
  //float v2 = read_voltage(2);
  //float v3 = read_voltage(3);
  //float c0 = read_current(0);
  //printf("%f %f %f %f\r\n\n",v0,v1,v2,v3);
}

static void aio_system_boot_cb(void)
{
  sl_status_t sc;
  uint8_t value_out = 1;
  /*sc = sl_bt_gatt_server_write_attribute_value(gattdb_number_of_digitalsA, 0, 1, &value_out);
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
                (int)sc);*/
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
  //TODO can batch it all into one value
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



static void aio_digital_out_read_cb_all(sl_bt_evt_gatt_server_user_read_request_t *data)
{
  sl_status_t sc;
  uint8_t s[4];
  for (int i=0; i<4; i++) {
      int pin=0; int port=0;
      channel_to_port_and_pin(i,&port,&pin);
      s[i]=GPIO_PinOutGet(port,pin);
      //printf("PORT %d pin %d out %d\r\n\n",port,pin,s[i]);
  }

  sc = sl_bt_gatt_server_send_user_read_response(
    data->connection,
    data->characteristic,
    0,
    4,
    s,
    NULL);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read response\n",
                (int)sc);
}


static void aio_analog_out_read_cb_all(sl_bt_evt_gatt_server_user_read_request_t *data)
{
  sl_status_t sc;
  int16_t s[4];
  s[0]=(int)(100*read_current(0));
  s[1]=(int)(100*read_current(1));
  s[2]=(int)(100*read_current(2));
  s[3]=(int)(100*read_current(3));

  sc = sl_bt_gatt_server_send_user_read_response(
    data->connection,
    data->characteristic,
    0,
    2*4,
    &s,
    NULL);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read response\n",
                (int)sc);
}

static void aio_analog_out_read_cb(sl_bt_evt_gatt_server_user_read_request_t *data,int channel)
{
  sl_status_t sc;
  int16_t current = (int)(100*read_current(channel));
  //printf("CURRENT ISXX %d\r\n\n",current);
  sc = sl_bt_gatt_server_send_user_read_response(
    data->connection,
    data->characteristic,
    0,
    2,
    &current,
    NULL);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read response\n",
                (int)sc);
}


static void aio_digital_out_write_cb_all(sl_bt_evt_gatt_server_user_write_request_t *data) {
  sl_status_t sc;
  uint8_t att_errorcode = 0;
  //printf("WTIING\r\n\n");
  for (int i=0; i<data->value.len; i++) {
      int port=0;
      int pin=0;
      channel_to_port_and_pin(i,&port,&pin);
        if (data->value.data[i]==0) {
            GPIO_PinOutClear(port,pin);
        } else if (data->value.data[i]==1) {
            GPIO_PinOutSet(port,pin);
        } else {
            printf("Weird value to write to pin! %d\n",data->value.data[i]);
        }
  }
  sc = sl_bt_gatt_server_send_user_write_response(data->connection,
                                                  data->characteristic,
                                                  att_errorcode);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user write response\n",
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
  //printf("EVENT %x\r\n\n",SL_BT_MSG_ID(evt->header));
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      aio_system_boot_cb();
      //sl_bt_sm_delete_bondings();
      sl_bt_sm_store_bonding_configuration(8, 2);
      sl_bt_sm_set_passkey(0);
      sl_bt_sm_configure(0x0B, sm_io_capability_displayyesno); //sm_io_capability_displayonly); //0x0F, sm_io_capability_displayyesno);// );
      sl_bt_sm_set_bondable_mode(1);
      printf("sl_bt_evt_system_boot_id\r\n\n");
      break;

    case sl_bt_evt_sm_confirm_bonding_id:
        printf("sl_bt_evt_sm_confirm_bonding_id\r\n\n");
        printf("    Event Parameters:\r\n\n");
        printf("        connection:     0x%02x\r\n\n", evt->data.evt_sm_confirm_bonding.connection);
        printf("        bonding_handle: %02d\r\n\n",   evt->data.evt_sm_confirm_bonding.bonding_handle);
       //sl_bt_sm_bonding_confirm(_conn_handle,1);
       sl_bt_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection,1);
       printf("Return from bonding\r\n\n");

      break;

    case sl_bt_evt_connection_parameters_id:
        printf("sl_bt_evt_connection_parameters_id\r\n\n");
        printf("    Event Parameters:\r\n");
        printf("        connection:    0x%02x\r\n\n", evt->data.evt_connection_parameters.connection);
        printf("        interval:      0x%04x\r\n\n", evt->data.evt_connection_parameters.interval);
        printf("        latency:       0x%04x\r\n\n", evt->data.evt_connection_parameters.latency);
        printf("        timeout:       0x%04x\r\n\n", evt->data.evt_connection_parameters.timeout);
        printf("        security_mode: %d\r\n\n",     evt->data.evt_connection_parameters.security_mode);
        printf("        txsize:        0x%04x\r\n\n", evt->data.evt_connection_parameters.txsize);
        uint8_t connection_handle = evt->data.evt_connection_parameters.connection;
        if(evt->data.evt_connection_parameters.security_mode < 2) {
          printf("Bonding Handle is: 0x%04X",_bonding_handle);
          if(_bonding_handle == 0xFF)
          {
            printf("Increasing security.\r\n");
            sl_bt_sm_increase_security(connection_handle);
            //connectionToIncreasSec = connection_handle;
          }
          else
          {
            printf("Increasing security..\r\n");
            sl_bt_sm_increase_security(connection_handle);
          }
        } else {
          printf("Clear increase sec timer.\r\n");
          //gecko_cmd_hardware_set_soft_timer(0, INCREASE_SEC_TMR_HANDLE, 1); //Clear timer
        }
        break;
        //sl_bt_sm_increase_security(_conn_handle); //not sure about this
        //sl_bt_sm_passkey_confirm(_conn_handle, 1);
        break;

    case sl_bt_evt_connection_phy_status_id:
        printf("sl_bt_evt_connection_phy_status_id\r\n\n");
        printf("    Event Parameters:\r\n\n");
        printf("        connection: 0x%02x\r\n\n", evt->data.evt_connection_phy_status.connection);
        printf("        phy:        0x%02x\r\n\n", evt->data.evt_connection_phy_status.phy);
        break;
    case sl_bt_evt_gatt_mtu_exchanged_id:
        printf("sl_bt_evt_gatt_mtu_exchanged_id\r\n\n");
        printf("    Event Parameters:\r\n\n");
        printf("        connection: 0x%02x\r\n\n", evt->data.evt_gatt_mtu_exchanged.connection);
        printf("        mtu:        0x%04x\r\n\n", evt->data.evt_gatt_mtu_exchanged.mtu);
        break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
      printf("sl_bt_evt_gatt_server_characteristic_status_id\r\n\n");
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
        // sl_bt_connection_close(_conn_handle); // not sure if we need this?
        break;

      // Event raised when bonding failed
        //reason codes are here, https://docs.silabs.com/mcu/latest/bgm13/group-sl-status
        //((sl_status_t)0x1006) ->  SL_STATUS_BT_CTRL_PIN_OR_KEY_MISSING   ((sl_status_t)0x1006)
      case sl_bt_evt_sm_bonding_failed_id:
        printf("Bonding failed\r\n\n");
        printf("--------------------------------------\r\n\n");
        //gecko_cmd_sm_increase_security(_conn_handle);        case sl_bt_evt_sm_bonding_failed_id:
        printf("sl_bt_evt_sm_bonding_failed_id\r\n\n");
        printf("    Event Parameters:\r\n");
        printf("        connection: 0x%02x\r\n", evt->data.evt_sm_bonding_failed.connection);
        printf("        reason:     0x%04x\r\n\n", evt->data.evt_sm_bonding_failed.reason);
        break;

    case sl_bt_evt_connection_opened_id:
        printf("sl_bt_evt_connection_opened_id\r\n\n");
        printf("    Event Parameters:\r\n");
        printf  ("        address:      %02x\r\n", evt->data.evt_connection_opened.address.addr[0]);
        printf  (":%02x", evt->data.evt_connection_opened.address.addr[1]);
        printf  (":%02x", evt->data.evt_connection_opened.address.addr[2]);
        printf  (":%02x", evt->data.evt_connection_opened.address.addr[3]);
        printf  (":%02x", evt->data.evt_connection_opened.address.addr[4]);
        printf(":%02x\r\n", evt->data.evt_connection_opened.address.addr[5]);
        printf("        address_type: 0x%02x\r\n", evt->data.evt_connection_opened.address_type);
        printf("        master:       0x%02x\r\n", evt->data.evt_connection_opened.master);
        printf("        connection:   0x%02x\r\n", evt->data.evt_connection_opened.connection);
        printf("        bonding:      0x%02x\r\n", evt->data.evt_connection_opened.bonding);
        printf("        advertiser:   0x%02x\r\n\n", evt->data.evt_connection_opened.advertiser);

      aio_connection_opened_cb(&evt->data.evt_connection_opened); // not sure why we need this

      _conn_handle = evt->data.evt_connection_opened.connection;
      _bonding_handle = evt->data.evt_connection_opened.bonding;

      if(_bonding_handle == 0xFF) {
        printf("Increasing security\r\n");
        sl_bt_sm_increase_security(_conn_handle);
      } else {
        printf("Already Bonded (ID: %d)\r\n", _bonding_handle);
      }
      printf("Connection open\n");
      break;

    case sl_bt_evt_connection_closed_id:
      aio_connection_closed_cb(&evt->data.evt_connection_closed);
      printf("Connection closed\n");
      break;

    case sl_bt_evt_gatt_server_user_read_request_id:
      //printf("sl_bt_evt_gatt_server_user_read_request_id\r\n\n");
      switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
        /*case gattdb_digitalA:
          printf("Reading from A\r\n\n");
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortA,7);
          break;
        case gattdb_digitalB:
          printf("Reading from B\r\n\n");
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortA,8);
          break;
        case gattdb_digitalC:
          printf("Reading from C\r\n\n");
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortC,6);
          break;
        case gattdb_digitalD:
          printf("Reading from D\r\n\n");
          aio_digital_out_read_cb(&evt->data.evt_gatt_server_user_read_request,gpioPortC,7);
          break;
        case gattdb_analogA:
          printf("Reading from AA\r\n\n");
          aio_analog_out_read_cb(&evt->data.evt_gatt_server_user_read_request,0);
          break;
        case gattdb_analogB:
          printf("Reading from AA\r\n\n");
          aio_analog_out_read_cb(&evt->data.evt_gatt_server_user_read_request,1);
          break;
        case gattdb_analogC:
          printf("Reading from AA\r\n\n");
          aio_analog_out_read_cb(&evt->data.evt_gatt_server_user_read_request,2);
          break;
        case gattdb_analogD:
          printf("Reading from AA\r\n\n");
          aio_analog_out_read_cb(&evt->data.evt_gatt_server_user_read_request,3);
          break;*/
        case gattdb_switch:
          //printf("Reading from switch\r\n\n");
          aio_digital_out_read_cb_all(&evt->data.evt_gatt_server_user_read_request);
          break;
        case gattdb_amps:
          ///printf("Reading from amps\r\n\n");
          aio_analog_out_read_cb_all(&evt->data.evt_gatt_server_user_read_request);
          break;
        default:
          printf("Reading from ? %d\r\n\n", evt->data.evt_gatt_server_user_read_request.characteristic);
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
        /*case gattdb_digitalA:
          printf("Writing to A\r\n\n");
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortA,7);
          break;
        case gattdb_digitalB:
          printf("Writing to B\r\n\n");
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortA,8);
          break;
        case gattdb_digitalC:
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortC,6);
          break;
        case gattdb_digitalD:
          aio_digital_out_write_cb(&evt->data.evt_gatt_server_user_write_request,gpioPortC,7);
          break;*/
        case gattdb_switch:
          aio_digital_out_write_cb_all(&evt->data.evt_gatt_server_user_write_request);
          break;
        default:
          printf("WRITE TO WTF\r\n\n");
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
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc); // restart advertising right after?
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
        //printf("WRITE\n");
        break;

    case sl_bt_evt_gatt_server_user_read_request_id:
        //printf("READ\n");
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
