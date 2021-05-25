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
#include <INA3221.h>
#include "em_common.h"
#include "em_gpio.h"
#include "em_core.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_i2cspm.h"
#include "ads1115.h"
#include "gatt_db.h"
#include "app.h"
#include "em_i2c.h"
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "em_cmu.h"
#include "em_gpio.h"
//#include "em_int.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
struct {
    bd_addr device_address;
    uint8_t address_type;
    uint8_t connection_handle;
} connections[8];
uint8_t live_connections = 0;

enum RELAY_STATE {
  RELAY_OFF = 0,
  RELAY_ON = 1,
  RELAY_TOGGLE = 2,
  RELAY_IGNORE = 3
};

enum APP_STATE {
  IDLE,
  RELAY_SERVE,
  SWITCH_CONNECT,
  SWITCH_GET_SERVICE,
  SWITCH_GET_CHAR
};

#define SIGNAL_AMP_NOTIFY 0x1
#define SIGNAL_RELAY_NOTIFY 0x2
#define SIGNAL_SWITCH_TOGGLE 0x4

#define NRELAYS 3
#define NBUTTONS 4
#define T_RELAY 0
#define T_SWITCH 1
#define TYPE 1

#define PIN_SET 0
#define PIN_UNSET 1

static const  uint8_t UUID_SERVICE[2] = {0x15,0x18}; // backwards
static const  uint8_t UUID_RELAY_CHAR[2] = {0x56,0x2A}; // backwards
static uint8_t relay_state[NRELAYS] = {0,0,0};
static uint8_t relay_changing[NRELAYS] = {0,0,0};
static uint8_t button_debounce_state[NRELAYS] = {0,0,0};

static int app_state = IDLE;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
static struct I2C_ADS1115 ads1115_sensor;
static struct I2C_INA3221 ina3221_sensor;

static uint32_t _service_handle=0;
static uint16_t _char_handle=0;
static uint8_t _conn_handle=0;
//static uint16_t _char_control_handle;
//static uint16_t _char_data_handle;

static sl_sleeptimer_timer_handle_t button_debounce_timers[NBUTTONS];
static sl_sleeptimer_timer_handle_t relay_timers[NRELAYS];
static sl_sleeptimer_timer_handle_t amp_notify_timer;
static uint32_t relay_delay_ticks = 0;
static uint32_t button_delay_ticks = 0;
static uint32_t amp_delay_ticks = 0;
#define RELAY_DLAY_MSEC 20
#define BUTTON_DLAY_MSEC 150
#define AMP_DLAY_MSEC 1000

static void relay_toggle(int idx);

static void relay_timer_callback (sl_sleeptimer_timer_handle_t *handle,
                                            void *data)
{
  int relay_idx = (int)data;
  int pin , port;
  channel_to_port_and_pin(relay_idx, PIN_UNSET, &port, &pin);
  GPIO_PinOutClear(port,pin);
  channel_to_port_and_pin(relay_idx, PIN_SET, &port, &pin);
  GPIO_PinOutClear(port,pin);
  relay_changing[relay_idx]=0;
}

static void button_debounce_timer_callback (sl_sleeptimer_timer_handle_t *handle,
                                            void *data)
{
  int button_idx = (int)data;
  button_debounce_state[button_idx]=0;
}

static void amp_notify_timer_callback (sl_sleeptimer_timer_handle_t *handle,
                                            void *data)
{
  //TODO optimize to only if current changes
  sl_bt_external_signal(SIGNAL_AMP_NOTIFY);
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
static void button_change(uint8_t idx)
{
  int port, pin;
     button_to_port_and_pin(idx, &port, &pin);
     int state = GPIO_PinInGet(port, pin);
     if (state==0) {
         //debounce it
         if (button_debounce_state[idx]==1) {
             return;
         }
         button_debounce_state[idx]=1;
         sl_sleeptimer_start_timer(button_debounce_timers+idx,
                                   button_delay_ticks,
                                   button_debounce_timer_callback,
                                   idx,
                                   0,
                                   0);
         switch(idx) {
           case 0:
             relay_toggle(0);
             break;
           case 1:
             relay_toggle(1);
             break;
           case 2:
             relay_toggle(2);
             break;
           case 3:

             sl_bt_external_signal(SIGNAL_SWITCH_TOGGLE);

             break;
           default:
             break;
         }
  }
}

SL_WEAK void app_init(void)
{
  int pin, port;
#if TYPE == T_RELAY

  for (int idx=0; idx<NRELAYS; idx++) {
      channel_to_port_and_pin(idx, PIN_SET, &port, &pin);
      GPIO_PinModeSet(port,pin,  gpioModePushPull,0);
      channel_to_port_and_pin(idx, PIN_UNSET, &port, &pin);
      GPIO_PinModeSet(port,pin,  gpioModePushPull,0);
  }


  ///sl_ads_init(&ads1115_sensor);
  sl_ina3221_init(&ina3221_sensor,INA3221_ADDRESS, 0.005); //LVK25 , 0.005 tol 0.5%


#elif TYPE == T_SWITCH
#endif

  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIOINT_Init();

  for (int idx=0; idx<NBUTTONS; idx++) {
      button_to_port_and_pin(idx, &port, &pin);
      printf("SETTING pin %d port %d\r\n\n",pin,port);
      GPIO_PinModeSet(port,pin,gpioModeInput,1);
      GPIOINT_CallbackRegister(idx, (GPIOINT_IrqCallbackPtr_t)button_change);
      GPIO_ExtIntConfig(port,pin,idx,false,true,true);
  }
  sl_sleeptimer_init();
  relay_delay_ticks = ((uint64_t)RELAY_DLAY_MSEC * sl_sleeptimer_get_timer_frequency()) / 1000;
  button_delay_ticks = ((uint64_t)BUTTON_DLAY_MSEC * sl_sleeptimer_get_timer_frequency()) / 1000;
  amp_delay_ticks = ((uint64_t)AMP_DLAY_MSEC * sl_sleeptimer_get_timer_frequency()) / 1000;

#if TYPE == T_RELAY
  sl_status_t sc= sl_sleeptimer_start_periodic_timer ( &amp_notify_timer,
                                                       amp_delay_ticks,
                                                       amp_notify_timer_callback,
  NULL,
  0,
  0
  );
#endif
}




/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/

void button_to_port_and_pin(int button, int * port, int * pin) {
  switch(button) {
    case 0:
      *port=gpioPortC;
      *pin=0;
      break;
    case 1:
      *port=gpioPortC;
      *pin=1;
      break;
    case 2:
      *port=gpioPortC;
      *pin=2;
      break;
    case 3:
      *port=gpioPortB;
      *pin=1;
      break;
    default:
      sl_app_assert(1==0, "[E: 0x%04x] Invalid button\n", button);
  }

}
void channel_to_port_and_pin(int channel, int pin_type, int * port, int * pin) {
  switch(channel) {
    case 0:
      if (pin_type==PIN_SET) {
        *port=gpioPortC;
        *pin=7;
      } else {
         *port=gpioPortC;
         *pin=6;
      }
      break;
    case 1:
      if (pin_type==PIN_SET) {
        *port=gpioPortA;
        *pin=7;
      } else {
         *port=gpioPortA;
         *pin=8;
      }
      break;
    case 2:
      if (pin_type==PIN_SET) {
        *port=gpioPortA;
        *pin=5;
      } else {
         *port=gpioPortA;
         *pin=6;
      }
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
  float current = (vout-1.64)/0.044;
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
  /*float v0 = read_voltage(0);
  float v1 = read_voltage(1);
  float v2 = read_voltage(2);
  float v3 = read_voltage(3);
  float c0 = read_current(0);
  printf("%f %f %f %f %f\r\n\n",v0,v1,v2,v3,c0);*/


  //INA3221
  /*for (int channel=1; channel<4; channel++) {
      double shuntA=INA3221_getCurrentA(&ina3221_sensor,  channel);
      double busV=INA3221_getBusVoltageV(&ina3221_sensor,  channel);
      double shuntV=INA3221_getShuntVoltageV(&ina3221_sensor,  channel);
      printf("%d CHANNEL, Bus %0.4lfV, Shunt %0.8lfV %0.6lfA\r\n\n",channel, busV , shuntV, shuntA );
  }*/
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

  sc = sl_bt_gatt_server_send_user_read_response(
    data->connection,
    data->characteristic,
    0,
    NRELAYS,
    &relay_state,
    NULL);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read response\n",
                (int)sc);
}


static void aio_analog_out_read_cb_all(sl_bt_evt_gatt_server_user_read_request_t *data)
{
  sl_status_t sc;
  double s[NRELAYS];
  s[0]=INA3221_getCurrentA(&ina3221_sensor,  1);
  s[1]=INA3221_getCurrentA(&ina3221_sensor,  2);
  s[2]=INA3221_getCurrentA(&ina3221_sensor,  3);
  printf("CURRENTS %0.6f %0.6f %0.6f\r\n\n",s[0],s[1],s[2]);

  sc = sl_bt_gatt_server_send_user_read_response(
    data->connection,
    data->characteristic,
    0,
    sizeof(double)*NRELAYS,
    &s,
    NULL);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read response\n",
                (int)sc);
}

static void aio_analog_out_read_cb(sl_bt_evt_gatt_server_user_read_request_t *data,int channel)
{
  sl_status_t sc;
  double current = INA3221_getCurrentA(&ina3221_sensor,  channel);
  //printf("CURRENT ISXX %d\r\n\n",current);
  sc = sl_bt_gatt_server_send_user_read_response(
    data->connection,
    data->characteristic,
    0,
    sizeof(double),
    &current,
    NULL);
  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to send user read response\n",
                (int)sc);
}


static void relay_off(int idx) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_ATOMIC();
  if (relay_changing[idx]==1) {
      CORE_EXIT_ATOMIC();
      return;
  }
  relay_changing[idx]=1;
  CORE_EXIT_ATOMIC();
  int port=0;
  int pin=0;
  channel_to_port_and_pin(idx,PIN_UNSET,&port,&pin);
  GPIO_PinOutSet(port,pin);
  sl_sleeptimer_start_timer(relay_timers+idx,
                            relay_delay_ticks,
                            relay_timer_callback,
                            idx,
                            0,
                            0);
  relay_state[idx]=0;

  sl_bt_external_signal(SIGNAL_RELAY_NOTIFY);
}
static void relay_on(int idx) {
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_ATOMIC();
  if (relay_changing[idx]==1) {
      CORE_EXIT_ATOMIC();
      return;
  }
  relay_changing[idx]=1;
  CORE_EXIT_ATOMIC();
  int port=0;
  int pin=0;
  channel_to_port_and_pin(idx,PIN_SET,&port,&pin);
  GPIO_PinOutSet(port,pin);
  sl_sleeptimer_start_timer(relay_timers+idx,
                            relay_delay_ticks,
                            relay_timer_callback,
                            idx,
                            0,
                            0);
  relay_state[idx]=1;


  sl_bt_external_signal(SIGNAL_RELAY_NOTIFY);
}
static void relay_toggle(int idx) {
  if (relay_state[idx]==1) {
      relay_off(idx);
  } else {
      relay_on(idx);
  }
}

static void aio_digital_out_write_cb_all(sl_bt_evt_gatt_server_user_write_request_t *data) {
  sl_status_t sc;
  uint8_t att_errorcode = 0;
  //printf("WTIING\r\n\n");
  for (int i=0; i<data->value.len; i++) {

        if (data->value.data[i]==RELAY_OFF) {
            relay_off(i);
        } else if (data->value.data[i]==RELAY_ON) {
            relay_on(i);
        } else if (data->value.data[i]==RELAY_TOGGLE) { //TOGGLE IT
            relay_toggle(i);
        } else if (data->value.data[i]==RELAY_IGNORE) { //IGNORE IT
            //GPIO_PinOutSet(port,pin);
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

//ary must be len 18 at least
void bd_addr_to_char(bd_addr addr, char* ary) {
  for(uint8_t i=0; i<6; i++) {
    /* Convert to hexadecimal (capital letters) with minimum width of 2 and '0' stuffing
     * More info on the sprintf parameters here: https://www.tutorialspoint.com/c_standard_library/c_function_sprintf.htm
     */
    sprintf(&(ary[i*3]), "%02X", addr.addr[i]);

    /* Add the ':' character to overwrite the null terminator added by sprintf (except on the last iteration */
    if(i<5) {
        ary[i*3+2] = ':';
    }
  }
  ary[17]='\0';
}

static int process_scan_response( sl_bt_evt_scanner_scan_report_t *response) {
  // Decoding advertising packets is done here. The list of AD types can be found
  // at: https://www.bluetooth.com/specifications/assigned-numbers/Generic-Access-Profile

  int ad_len;
  int ad_type;
  char name[32];
  char addr[18];
  bd_addr_to_char(response->address,addr);
  name[0]='\0';
  int found=0;
  //0x17  «Public Target Address» // TODO get the address too?
  //0x1B  «LE Bluetooth Device Address»

  for (int i=0; i < (response->data.len - 1); i+=ad_len+1) {
    ad_len = response->data.data[i];
    ad_type = response->data.data[i + 1];
    //printf("Adtype %x\r\n", ad_type);

    if (ad_type == 0x08 || ad_type == 0x09) {
      // Type 0x08 = Shortened Local Name
      // Type 0x09 = Complete Local Name
      memcpy(name, &(response->data.data[i + 2]), ad_len - 1);
      name[ad_len - 1] = 0;
      if (strcmp(name,"thundipi")==0)  {
          found=1;
      }
    }
  }
  /*if (found==1) {
    printf("|%s| |%s|\n",name,addr);
  }*/
  return found;
}


/*void sl_button_on_change(const sl_button_t *handle)
{
  (void)handle;
#ifdef SL_CATALOG_GATT_SERVICE_AIO_PRESENT
  //sl_gatt_service_aio_on_change();
#endif // SL_CATALOG_GATT_SERVICE_AIO_PRESENT
  if (sl_button_get_state(handle)==1) {
    if (handle==&sl_button_relaybutton0) {
        relay_toggle(0);
    }
    if (handle==&sl_button_relaybutton1) {
        relay_toggle(1);
    }
    if (handle==&sl_button_relaybutton2) {
        relay_toggle(2);
    }
  }
  if (_char_handle>0) {
      if (sl_button_get_state(handle)==1 ) {
          uint8_t toggle[4] = { 0x00, 0x00, 0x02, 0x00 };
          //int sc = sl_bt_gatt_write_characteristic_value(_conn_handle,_char_handle,4,toggle);
      }
      //printf("TOGGLE IS %d\r\n\n",sc);
  }

}*/


//static uint8_t _conn_handle = 0xFF;
static uint8_t _bonding_handle = 0xFF;
void sl_bt_aio_process(sl_bt_msg_t *evt) {
  // Handle stack events
  // printf("EVENT %x\r\n\n",SL_BT_MSG_ID(evt->header));
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      aio_system_boot_cb();

/*#if TYPE==T_SWITCH
      uint8_t switch_name[]={ 0x74,0x68,0x75,0x6E,0x64,0x69,0x73,0x77,0x69,0x74,0x63,0x68};
      uint16_t result = sl_bt_gatt_server_write_attribute_value(gattdb_device_name,0,sizeof(switch_name),switch_name);
#endif*/
      //sl_bt_sm_delete_bondings();
      sl_bt_sm_store_bonding_configuration(8, 2);
      sl_bt_sm_set_passkey(0);
      sl_bt_sm_configure(0x0B, sm_io_capability_displayyesno); //sm_io_capability_displayonly); //0x0F, sm_io_capability_displayyesno);// );


#if TYPE == T_RELAY
      sl_bt_sm_set_bondable_mode(1);
      printf("RELAY sl_bt_evt_system_boot_id\r\n\n");
      app_state=RELAY_SERVE;
#elif TYPE == T_SWITCH
      sl_bt_sm_set_bondable_mode(0); //TODO SHOULD THIS BE A 1?
      printf("SWITCH sl_bt_evt_system_boot_id\r\n\n");
      sl_bt_scanner_start(1, scanner_discover_generic);
      app_state=SWITCH_CONNECT;
#endif
      break;
    case sl_bt_evt_scanner_scan_report_id:
      //printf("sl_bt_evt_scanner_scan_report_id\r\n\n");
      //struct sl_bt_evt_scanner_scan_report_s report = evt->data.evt_scanner_scan_report;
      if (process_scan_response(&(evt->data.evt_scanner_scan_report))==1) {
          //found  a thundipi!
          // then stop scanning for a while
          int sc = sl_bt_scanner_stop();
          sl_app_assert(sc == SL_STATUS_OK,
                        "[E: 0x%04x] Failed to stop discovery\n",
                        (int)sc);
          // and connect to that device
          //if (active_connections_num < SL_BT_CONFIG_MAX_CONNECTIONS) {
            sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                       evt->data.evt_scanner_scan_report.address_type,
                                       gap_1m_phy,
                                       NULL);
            printf("open a connection!\r\n");

      }
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
        printf("sl_bt_evt_connection_parameters_id\r\n");
        printf("    Event Parameters:\r\n");
        printf("        connection:    0x%02x\r\n", evt->data.evt_connection_parameters.connection);
        printf("        interval:      0x%04x\r\n", evt->data.evt_connection_parameters.interval);
        printf("        latency:       0x%04x\r\n", evt->data.evt_connection_parameters.latency);
        printf("        timeout:       0x%04x\r\n", evt->data.evt_connection_parameters.timeout);
        printf("        security_mode: %d\r\n",     evt->data.evt_connection_parameters.security_mode);
        printf("        txsize:        0x%04x\r\n", evt->data.evt_connection_parameters.txsize);
        uint8_t connection_handle = evt->data.evt_connection_parameters.connection;
        if(evt->data.evt_connection_parameters.security_mode < 2) {
            sl_bt_sm_increase_security(connection_handle);
        } else { //security is >=3
#if TYPE == T_SWITCH
            if (app_state==SWITCH_CONNECT) {
                app_state=SWITCH_GET_SERVICE;
                printf("LETS GET A SERVICE!\r\n");
                //sl_bt_gatt_discover_primary_services(evt->data.evt_connection_parameters.connection);
                sl_bt_gatt_discover_primary_services_by_uuid(evt->data.evt_connection_parameters.connection,  2, UUID_SERVICE);
            }
#endif
          //gecko_cmd_hardware_set_soft_timer(0, INCREASE_SEC_TMR_HANDLE, 1); //Clear timer
        }
        break;
        //sl_bt_sm_increase_security(_conn_handle); //not sure about this
        //sl_bt_sm_passkey_confirm(_conn_handle, 1);
        //break;

    case sl_bt_evt_gatt_service_id:
      {
        //sl_bt_evt_gatt_service_t service =
        //sl_bt_evt_gatt_service_t service = evt->data.evt_gatt_service;
        if(2!= evt->data.evt_gatt_service.uuid.len || memcmp(UUID_SERVICE,evt->data.evt_gatt_service.uuid.data,2)!=0) {
            break;
        }
        _service_handle = evt->data.evt_gatt_service.service;
        printf("FOUND THE SERVICE!\r\n");
      }
      break;

    case sl_bt_evt_gatt_characteristic_id:
      if(2!= evt->data.evt_gatt_characteristic.uuid.len || memcmp(UUID_RELAY_CHAR,evt->data.evt_gatt_characteristic.uuid.data,2)!=0) {
          break;
      }
      _char_handle = evt->data.evt_gatt_characteristic.characteristic;
      _conn_handle = evt->data.evt_gatt_characteristic.connection;
      printf("FOUND THE CHAR!\r\n");
      break;

    case sl_bt_evt_gatt_procedure_completed_id:
      printf("COMPLETED GATT PROCED %d\r\n",evt->data.evt_gatt_procedure_completed.result);
      if (_service_handle!=0 && app_state==SWITCH_GET_SERVICE) {
          app_state=SWITCH_GET_CHAR;
          sl_bt_gatt_discover_characteristics_by_uuid(evt->data.evt_gatt_procedure_completed.connection, _service_handle, 2, UUID_RELAY_CHAR);
      }
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
      sl_bt_sm_passkey_confirm(evt->data.evt_sm_confirm_passkey.connection, 1);
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

      //_conn_handle = evt->data.evt_connection_opened.connection;
      //_bonding_handle = evt->data.evt_connection_opened.bonding;

      if( evt->data.evt_connection_opened.bonding == 0xFF) {
        printf("Increasing security\r\n");
        sl_bt_sm_increase_security(evt->data.evt_connection_opened.connection);
      } else {
        printf("Already Bonded (ID: %d)\r\n",  evt->data.evt_connection_opened.bonding);
      }
      printf("Connection open\n");
      connections[live_connections].device_address = evt->data.evt_connection_opened.address;
      connections[live_connections].address_type = evt->data.evt_connection_opened.address_type;
      connections[live_connections].connection_handle = evt->data.evt_connection_opened.connection;
      live_connections++;
      break;

    case sl_bt_evt_connection_closed_id:
      aio_connection_closed_cb(&evt->data.evt_connection_closed);
      printf("Connection closed\n");
      int i=0;
      for (; i<live_connections; i++) {
          //compare address!
          if (memcmp(connections[i].device_address.addr,evt->data.evt_connection_opened.address.addr,6)==0) {
              printf("FOUND CONNETION TO REMOVE\r\n\n");
              break;
          }
      }
      if (i==live_connections) {
          printf("FAILE TO FIND THE ONE TO REMOVE...\r\n\n");
      }
      for (int j=i+1; j<live_connections; j++) {
          connections[j-1]=connections[j];
      }
      live_connections--;
      break;

    case sl_bt_evt_gatt_server_user_read_request_id:
      //printf("sl_bt_evt_gatt_server_user_read_request_id\r\n\n");
      switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
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
      case sl_bt_evt_system_external_signal_id:
        if (evt->data.evt_system_external_signal.extsignals & SIGNAL_AMP_NOTIFY) {
            sl_status_t sc;
            double s[NRELAYS];
            s[0]=INA3221_getCurrentA(&ina3221_sensor,  1);
            s[1]=INA3221_getCurrentA(&ina3221_sensor,  2);
            s[2]=INA3221_getCurrentA(&ina3221_sensor,  3);
            printf("CURRENTS %0.6f %0.6f %0.6f\r\n\n",s[0],s[1],s[2]);

            for (int i=0; i<live_connections; i++) {
              sc = sl_bt_gatt_server_send_characteristic_notification(
                //0xFF,
                  connections[i].connection_handle,
                gattdb_amps,
                sizeof(double)*NRELAYS,
                &s,
                NULL);
              //sl_app_assert(sc == SL_STATUS_OK,
              //              "[E: 0x%04x] Failed to send user notify\n",
              //              (int)sc);
            }
        } else if (evt->data.evt_system_external_signal.extsignals & SIGNAL_RELAY_NOTIFY) {
            for (int i=0; i<live_connections; i++) {
            sl_status_t sc = sl_bt_gatt_server_send_characteristic_notification(
                //0xFF,
                  connections[i].connection_handle,
              gattdb_switch,
              NRELAYS,
              &relay_state,
              NULL);
            /*sl_app_assert(sc == SL_STATUS_OK,
                          "[E: 0x%04x] Failed to send user notify\n",
                          (int)sc);*/
            }
        } else if (evt->data.evt_system_external_signal.extsignals & SIGNAL_SWITCH_TOGGLE) {
            if (_char_handle>0) {
              uint8_t toggle[NRELAYS] = { 0x03, 0x03, 0x02 };
              int sc = sl_bt_gatt_write_characteristic_value(_conn_handle,_char_handle,NRELAYS,toggle);

            }
        //printf("TOGGLE IS %d\r\n\n",sc);
        }
        break;
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
#if TYPE==T_SWITCH
      uint8_t switch_name[]={ 0x74,0x68,0x75,0x6E,0x64,0x69,0x73,0x77,0x69,0x74,0x63,0x68};
      uint16_t result = sl_bt_gatt_server_write_attribute_value(gattdb_device_name,0,sizeof(switch_name),switch_name);
#endif
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
#if TYPE==T_RELAY
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
#endif
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to restart advertising 1\n",
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
                    "[E: 0x%04x] Failed to restart advertising 2\n",
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
