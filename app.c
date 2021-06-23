#include "em_common.h"
#include "em_gpio.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"
#include "gatt_db.h"
#include "gpiointerrupt.h"
#include <INA3221.h>
#include <THUNDIPII2C.h>
#include "pin_config.h"
#include "app.h"
#include "button.h"
#include "timers.h"
#include "relay.h"
#include "stats.h"

static uint8_t live_connections = 0;
static connection connections[8];

static const uint8_t UUID_SERVICE[2] = { 0x15, 0x18 }; // backwards
static const uint8_t UUID_RELAY_CHAR[2] = { 0x56, 0x2A }; // backwards

int app_state = IDLE;
int i2c_thundi_state = I2C_THUNDI_DISCONNECTED;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static struct I2C_INA3221 ina3221_sensor;
static struct I2C_THUNDIPII2C thundipii2c_sensor;

static uint32_t _service_handle = 0;
static uint16_t _char_handle = 0;
static uint8_t _conn_handle = 0;

static uint16_t passkey_repeats = 0;

double currents[NRELAYS];
double voltages[NRELAYS];
static bool boot_to_dfu = false;
bd_addr address;

static void
aio_digital_out_read_cb_all(sl_bt_evt_gatt_server_user_read_request_t *data);
static void
aio_digital_out_write_cb_all(sl_bt_evt_gatt_server_user_write_request_t *data);
static void
aio_analog_out_read_cb_all(sl_bt_evt_gatt_server_user_read_request_t *data);

/**************************************************************************//**
 * App BT
 *****************************************************************************/
SL_WEAK void app_init(void) {
	printf("IN INIT x1\r\n\n");
	init_stats();
	int pin, port;
#if T_TYPE == T_RELAY
	for (int idx = 0; idx < NRELAYS; idx++) {
		channel_to_port_and_pin(idx, PIN_SET, &port, &pin);
		GPIO_PinModeSet(port, pin, gpioModePushPull, 0);
		channel_to_port_and_pin(idx, PIN_UNSET, &port, &pin);
		GPIO_PinModeSet(port, pin, gpioModePushPull, 0);
	}
	sl_ina3221_init(&ina3221_sensor, INA3221_ADDRESS, 0.005); //LVK25 , 0.005 tol 0.5%

	sl_thundipii2c_init(&thundipii2c_sensor, THUNDIPII2C_ADDRESS);

	//uint16_t thundi_id = thundipi_read_id(&thundipii2c_sensor);
	//printf("THUNID IID GOT ID %x\r\n\n", thundi_id);

	GPIO_PinModeSet(BUTTON1_LED_PORT, BUTTON1_LED_PIN, gpioModePushPull, 0);
#elif T_TYPE == T_SWITCH
	thundipi_slave_initI2C();
#endif

	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIOINT_Init();

	for (int idx = 0; idx < NBUTTONS; idx++) {
		button_to_port_and_pin(idx, &port, &pin);
		//printf("SETTING pin %d port %d\r\n\n",pin,port);
		GPIO_PinModeSet(port, pin, gpioModeInput, 1);
		GPIOINT_CallbackRegister(idx, (GPIOINT_IrqCallbackPtr_t) button_change);
		GPIO_ExtIntConfig(port, pin, idx, true, true, true);
	}

	GPIO_PinModeSet(BUTTON1_LED_PORT, BUTTON1_LED_PIN, gpioModePushPull, 0);

	init_timers();
}

SL_WEAK void app_process_action(void) {

}

static void aio_system_boot_cb(void) {
}

static void aio_connection_opened_cb(sl_bt_evt_connection_opened_t *data) {
	(void) data;
}

static void aio_connection_closed_cb(sl_bt_evt_connection_closed_t *data) {
	(void) data;
}

/**************************************************************************//**
 * AIO read and write
 *****************************************************************************/
static void aio_digital_out_read_cb_all(
		sl_bt_evt_gatt_server_user_read_request_t *data) {
	sl_status_t sc;

	sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0,
			NRELAYS, relay_state,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}

static void aio_analog_out_read_cb_all(
		sl_bt_evt_gatt_server_user_read_request_t *data) {
	sl_status_t sc;
	double s[NRELAYS];

	sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0, sizeof(double) * NRELAYS,
			(const uint8_t*) currents,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}

static void aio_digital_out_write_cb_all(
		sl_bt_evt_gatt_server_user_write_request_t *data) {
	sl_status_t sc;

	for (int i = 0; i < data->value.len; i++) {

		if (data->value.data[i] == RELAY_OFF) {
			relay_off(i);
		} else if (data->value.data[i] == RELAY_ON) {
			relay_on(i);
		} else if (data->value.data[i] == RELAY_TOGGLE) { //TOGGLE IT
			relay_toggle(i);
		} else if (data->value.data[i] == RELAY_IGNORE) { //IGNORE IT
		} else {
		}
	}
	uint8_t att_errorcode = 0;
	sc = sl_bt_gatt_server_send_user_write_response(data->connection,
			data->characteristic, att_errorcode);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user write response\n", (int )sc);

}

/**************************************************************************//**
 * Misc
 *****************************************************************************/
//ary must be len 18 at least
void bd_addr_to_char(uint8_t *addr, char *ary) {
	for (uint8_t i = 0; i < 6; i++) {
		/* Convert to hexadecimal (capital letters) with minimum width of 2 and '0' stuffing
		 * More info on the sprintf parameters here: https://www.tutorialspoint.com/c_standard_library/c_function_sprintf.htm
		 */
		sprintf(&(ary[i * 3]), "%02X", addr[i]);

		/* Add the ':' character to overwrite the null terminator added by sprintf (except on the last iteration */
		if (i < 5) {
			ary[i * 3 + 2] = ':';
		}
	}
	ary[17] = '\0';
}

static int process_scan_response(sl_bt_evt_scanner_scan_report_t *response) {
	// Decoding advertising packets is done here. The list of AD types can be found
	// at: https://www.bluetooth.com/specifications/assigned-numbers/Generic-Access-Profile
#if T_TYPE==T_SWITCH // switch only bonds over I2C
	if (memcmp(response->address.addr,thundipi_read_master_addr(),6)!=0) { // not the right address
		return 0;
	}
#endif
	int ad_len;
	int ad_type;
	char name[32];
	char addr[18];
	bd_addr_to_char(response->address.addr, addr);
	name[0] = '\0';
	int found = 0;
	//0x17  «Public Target Address» // TODO get the address too?
	//0x1B  «LE Bluetooth Device Address»

	for (int i = 0; i < (response->data.len - 1); i += ad_len + 1) {
		ad_len = response->data.data[i];
		ad_type = response->data.data[i + 1];
		//printf("Adtype %x\r\n", ad_type);

		if (ad_type == 0x08 || ad_type == 0x09) {
			// Type 0x08 = Shortened Local Name
			// Type 0x09 = Complete Local Name
			memcpy(name, &(response->data.data[i + 2]), ad_len - 1);
			name[ad_len - 1] = 0;
			if (strcmp(name, "thundipi") == 0) {
				printf("FOUDN THUNDIPI!\r\n\n");
				found = 1;
			}
		}
	}
	/*if (found==1) {
	 printf("|%s| |%s|\n",name,addr);
	 }*/
	return found;
}

/**************************************************************************//**
 * Main BT handling loop
 *****************************************************************************/
static uint8_t system_id[8];
uint32_t bt_bonding_passkey;
uint8_t bt_bonding_connection;
void sl_bt_aio_process(sl_bt_msg_t *evt) {
	// Handle stack events
	//printf("EVENT %x\r\n\n", SL_BT_MSG_ID(evt->header));
	switch (SL_BT_MSG_ID(evt->header)) {
	case sl_bt_evt_system_boot_id:
		aio_system_boot_cb();

#if T_TYPE==T_SWITCH
		uint8_t switch_name[] = { 0x74, 0x68, 0x75, 0x6E, 0x64, 0x69, 0x73,
				0x77, 0x69, 0x74, 0x63, 0x68 };
		sl_bt_gatt_server_write_attribute_value(
				gattdb_device_name, 0, sizeof(switch_name), switch_name);
#endif
		sl_bt_sm_delete_bondings();
		sl_bt_sm_store_bonding_configuration(8, 2);

		sl_bt_sm_configure(0x0B, sm_io_capability_displayyesno); //sm_io_capability_displayonly); //0x0F, sm_io_capability_displayyesno);// );

		sl_bt_sm_set_bondable_mode(1);


#if T_TYPE == T_RELAY
		sl_bt_sm_set_bondable_mode(1);
		printf("RELAY sl_bt_evt_system_boot_id\r\n\n");
		app_state = RELAY_SERVE;
#elif T_TYPE == T_SWITCH
		sl_bt_sm_set_bondable_mode(0); //TODO SHOULD THIS BE A 1?
		printf("SWITCH sl_bt_evt_system_boot_id\r\n\n");
		//sl_bt_scanner_start(1, scanner_discover_generic);
		app_state = SWITCH_I2C_WAIT;
#endif
		//sl_status_t sc = sl_bt_sm_set_passkey(1234);
		//sl_app_assert(sc == SL_STATUS_OK,
		//              "[E: 0x%04x] Failed to set using random passkeys\r\n",
		//              (int)sc);

		//printf("BLINK\r\n\n");
		//set_discoverable();
		//printf("REBOOT TO DFU\r\n\n");
		//sl_bt_system_reset(2); // TODO RESET this breaks
		break;
	case sl_bt_evt_scanner_scan_report_id:
		if (process_scan_response(&(evt->data.evt_scanner_scan_report)) == 1) {
			//found  a thundipi!
			// then stop scanning for a while
			int sc = sl_bt_scanner_stop();
			sl_app_assert(sc == SL_STATUS_OK,
					"[E: 0x%04x] Failed to stop discovery\n", (int )sc);

			sc = sl_bt_connection_open(
					evt->data.evt_scanner_scan_report.address,
					evt->data.evt_scanner_scan_report.address_type, gap_1m_phy,
					NULL);
			printf("open a connection!\r\n");

		}
		break;

	case sl_bt_evt_sm_confirm_bonding_id:
#ifdef PI_DEBUG
		printf("sl_bt_evt_sm_confirm_bonding_id\r\n\n");
		printf("    Event Parameters:\r\n\n");
		printf("        connection:     0x%02x\r\n\n",
				evt->data.evt_sm_confirm_bonding.connection);
		printf("        bonding_handle: %02d\r\n\n",
				evt->data.evt_sm_confirm_bonding.bonding_handle);
#endif
		sl_bt_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection,
				1);
		printf("Return from bonding\r\n\n");

		break;

	case sl_bt_evt_connection_parameters_id:
		printf("sl_bt_evt_connection_parameters_id\r\n");
		printf("    Event Parameters:\r\n");
		printf("        connection:    0x%02x\r\n",
				evt->data.evt_connection_parameters.connection);
		printf("        interval:      0x%04x\r\n",
				evt->data.evt_connection_parameters.interval);
		printf("        latency:       0x%04x\r\n",
				evt->data.evt_connection_parameters.latency);
		printf("        timeout:       0x%04x\r\n",
				evt->data.evt_connection_parameters.timeout);
		printf("        security_mode: %d\r\n",
				evt->data.evt_connection_parameters.security_mode);
		printf("        txsize:        0x%04x\r\n",
				evt->data.evt_connection_parameters.txsize);

		if (evt->data.evt_connection_parameters.security_mode < 2) {
			sl_bt_sm_increase_security(
					evt->data.evt_connection_parameters.connection);
			printf("INCREASE SECURITY\r\n");
		} else { //security is >=3
#if T_TYPE == T_SWITCH
			if (app_state == SWITCH_SCAN) {
				app_state = SWITCH_GET_SERVICE;
				printf("SEARCH FOR SERVICES!\r\n\n");
				sl_bt_gatt_discover_primary_services_by_uuid(
						evt->data.evt_connection_parameters.connection, 2,
						UUID_SERVICE);
			}
#endif
		}

		break;

	case sl_bt_evt_gatt_service_id: {
		if (2 != evt->data.evt_gatt_service.uuid.len
				|| memcmp(UUID_SERVICE, evt->data.evt_gatt_service.uuid.data, 2)
						!= 0) {
			break;
		}
		_service_handle = evt->data.evt_gatt_service.service;
	}
		printf("FOUND SERVIC %x %x \r\n\n",
				evt->data.evt_gatt_service.uuid.data[0],
				evt->data.evt_gatt_service.uuid.data[1]);
		break;

	case sl_bt_evt_gatt_characteristic_id:
		if (2 != evt->data.evt_gatt_characteristic.uuid.len
				|| memcmp(UUID_RELAY_CHAR,
						evt->data.evt_gatt_characteristic.uuid.data, 2) != 0) {
			break;
		}
		_char_handle = evt->data.evt_gatt_characteristic.characteristic;
		_conn_handle = evt->data.evt_gatt_characteristic.connection; // TODO if this handle closes then remove char handle
		break;

	case sl_bt_evt_gatt_procedure_completed_id:
		if (_service_handle != 0 && app_state == SWITCH_GET_SERVICE) {
			app_state = SWITCH_GET_CHAR;
			sl_bt_gatt_discover_characteristics_by_uuid(
					evt->data.evt_gatt_procedure_completed.connection,
					_service_handle, 2, UUID_RELAY_CHAR);
		}
		if (app_state == SWITCH_GET_CHAR) {
			//printf("GOT THE SERV AND CHAR\r\n\n");
		}
		break;

	case sl_bt_evt_connection_phy_status_id:
		printf("sl_bt_evt_connection_phy_status_id\r\n\n");
		printf("    Event Parameters:\r\n\n");
		printf("        connection: 0x%02x\r\n\n",
				evt->data.evt_connection_phy_status.connection);
		printf("        phy:        0x%02x\r\n\n",
				evt->data.evt_connection_phy_status.phy);
		break;
	case sl_bt_evt_gatt_mtu_exchanged_id:
		printf("sl_bt_evt_gatt_mtu_exchanged_id\r\n\n");
		printf("    Event Parameters:\r\n\n");
		printf("        connection: 0x%02x\r\n\n",
				evt->data.evt_gatt_mtu_exchanged.connection);
		printf("        mtu:        0x%04x\r\n\n",
				evt->data.evt_gatt_mtu_exchanged.mtu);
		break;

	case sl_bt_evt_gatt_server_characteristic_status_id:
		printf("sl_bt_evt_gatt_server_characteristic_status_id\r\n\n");
		break;

	case sl_bt_evt_sm_passkey_display_id:
		printf("Passkey: %06lu", evt->data.evt_sm_passkey_display.passkey);
		break;
		// Event raised by the security manager when a passkey needs to be confirmed
	case sl_bt_evt_sm_confirm_passkey_id:
		app_state = RELAY_CONFIRM;
		printf(
				"Do you see this passkey on the other device: %06lu? (y/n)\r\n\n",
				evt->data.evt_sm_confirm_passkey.passkey);
		bt_bonding_passkey = evt->data.evt_sm_confirm_passkey.passkey;
		bt_bonding_connection = evt->data.evt_sm_confirm_passkey.connection;

#if T_TYPE==T_REALY
		thundipi_write_passkey(&thundipii2c_sensor, bt_bonding_passkey);
#endif
#if T_TYPE==T_SWITCH
		thundipi_write_passkey_to_mem(bt_bonding_passkey);
#endif
		passkey_repeats = PASSKEY_CHECKS;
		start_passkey_timer();

		//sl_bt_sm_passkey_confirm(evt->data.evt_sm_confirm_passkey.connection,	1);
		break;

		// Event raised when bonding is successful
	case sl_bt_evt_sm_bonded_id:
		printf("Bonded\r\n\n");
#if T_TYPE == T_RELAY
		unset_discoverable();
		app_state = RELAY_SERVE;
#endif

		//printf("--------------------------------------\r\n\n");
		// sl_bt_connection_close(_conn_handle); // not sure if we need this?
		break;

		// Event raised when bonding failed
		//reason codes are here, https://docs.silabs.com/mcu/latest/bgm13/group-sl-status
		//((sl_status_t)0x1006) ->  SL_STATUS_BT_CTRL_PIN_OR_KEY_MISSING   ((sl_status_t)0x1006)
	case sl_bt_evt_sm_bonding_failed_id:
		printf("Bonding failed\r\n\n");
		//printf("--------------------------------------\r\n\n");
		//gecko_cmd_sm_increase_security(_conn_handle);        case sl_bt_evt_sm_bonding_failed_id:
		printf("sl_bt_evt_sm_bonding_failed_id\r\n\n");
		printf("    Event Parameters:\r\n");
		printf("        connection: 0x%02x\r\n",
				evt->data.evt_sm_bonding_failed.connection);
		printf("        reason:     0x%04x\r\n\n",
				evt->data.evt_sm_bonding_failed.reason);
		/* If the attempt at bonding/pairing failed, clear the bonded flag and display the reason */
		switch (evt->data.evt_sm_bonding_failed.reason) {
		case 0x0301:
			printf("The user input of passkey failed\r\n");
			break;

		case 0x0302:
			printf("Out of Band data is not available for authentication\r\n");
			break;

		case 0x0303:
			printf(
					"The pairing procedure cannot be performed as authentication requirements cannot be met due to IO capabilities of one or both devices\r\n");
			break;

		case 0x0304:
			printf(
					"The confirm value does not match the calculated compare value\r\n");
			break;

		case 0x0305:
			printf("Pairing is not supported by the device\r\n");
			break;

		case 0x0306:
			printf(
					"The resultant encryption key size is insufficient for the security requirements of this device\r\n");
			break;

		case 0x0307:
			printf(
					"The SMP command received is not supported on this device\r\n");
			break;

		case 0x0308:
			printf("Pairing failed due to an unspecified reason\r\n");
			break;

		case 0x0309:
			printf(
					"Pairing or authentication procedure is disallowed because too little time has elapsed since last pairing request or security request\r\n");
			break;

		case 0x030A:
			printf("Invalid Parameters\r\n");
			break;

		case 0x030B:
			printf("The bonding does not exist\r\n");
			break;

		case 0x0206:
			printf(
					"Pairing failed because of missing PIN, or authentication failed because of missing Key\r\n");
			printf(
					"Delete the Health Thermometer device from the Tablet to force bonding\r\n");
			break;

		case 0x0185:
			printf("Command or Procedure failed due to timeout\r\n");
			break;

		default:
			printf("Unknown error: 0x%X\r\n",
					evt->data.evt_sm_bonding_failed.reason);
			break;
		}

		break;

	case sl_bt_evt_connection_opened_id:
		printf("sl_bt_evt_connection_opened_id\r\n\n");
		printf("    Event Parameters:\r\n");
		printf("        address:      %02x\r\n",
				evt->data.evt_connection_opened.address.addr[0]);
		printf(":%02x", evt->data.evt_connection_opened.address.addr[1]);
		printf(":%02x", evt->data.evt_connection_opened.address.addr[2]);
		printf(":%02x", evt->data.evt_connection_opened.address.addr[3]);
		printf(":%02x", evt->data.evt_connection_opened.address.addr[4]);
		printf(":%02x\r\n", evt->data.evt_connection_opened.address.addr[5]);
		printf("        address_type: 0x%02x\r\n",
				evt->data.evt_connection_opened.address_type);
		printf("        master:       0x%02x\r\n",
				evt->data.evt_connection_opened.master);
		printf("        connection:   0x%02x\r\n",
				evt->data.evt_connection_opened.connection);
		printf("        bonding:      0x%02x\r\n",
				evt->data.evt_connection_opened.bonding);
		printf("        advertiser:   0x%02x\r\n\n",
				evt->data.evt_connection_opened.advertiser);

		aio_connection_opened_cb(&evt->data.evt_connection_opened); // not sure why we need this

		if (evt->data.evt_connection_opened.bonding == 0xFF) {
			printf("Increasing securityXX\r\n");
			sl_bt_sm_increase_security(
					evt->data.evt_connection_opened.connection);
		} else {
			printf("Already Bonded (ID: %d)\r\n",
					evt->data.evt_connection_opened.bonding);
		}
		printf("Connection open\n");
		connections[live_connections].device_address =
				evt->data.evt_connection_opened.address;
		connections[live_connections].address_type =
				evt->data.evt_connection_opened.address_type;
		connections[live_connections].connection_handle =
				evt->data.evt_connection_opened.connection;
		live_connections++;
		break;

	case sl_bt_evt_connection_closed_id:
		aio_connection_closed_cb(&evt->data.evt_connection_closed);
		printf("Connection closed %d\n", live_connections);
		int i = 0;
		for (; i < live_connections; i++) {
			//compare address!
			if (memcmp(connections[i].device_address.addr,
					evt->data.evt_connection_opened.address.addr, 6) == 0) {
				break;
			}
		}
		if (i == live_connections) {
			printf("FAILE TO FIND THE ONE TO REMOVE...\r\n\n");
		}
		for (int j = i + 1; j < live_connections; j++) {
			connections[j - 1] = connections[j];
		}
		live_connections--;

#if T_TYPE == T_SWITCH
		//sl_bt_scanner_start(1, scanner_discover_generic);
		app_state = SWITCH_I2C_WAIT;
#endif

		if (boot_to_dfu) {
			printf("BOOT TO DFU!!!\r\n\n");
			// Reset MCU and enter OTA DFU mode.
			//sl_bt_system_reset(2);
		}
		break;

	case sl_bt_evt_gatt_server_user_read_request_id:
		//printf("sl_bt_evt_gatt_server_user_read_request_id\r\n\n");
		switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
		case gattdb_switch:
			aio_digital_out_read_cb_all(
					&evt->data.evt_gatt_server_user_read_request);
			break;
		case gattdb_amps:
			///printf("Reading from amps\r\n\n");
			aio_analog_out_read_cb_all(
					&evt->data.evt_gatt_server_user_read_request);
			break;
		default:
			printf("Reading from ? %d\r\n\n",
					evt->data.evt_gatt_server_user_read_request.characteristic);
		}
		break;

	case sl_bt_evt_gatt_server_user_write_request_id:
		switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
		case gattdb_switch:
			aio_digital_out_write_cb_all(
					&evt->data.evt_gatt_server_user_write_request);

			break;
		case gattdb_ota_control:
			// Set flag to enter OTA mode.
			boot_to_dfu = true;
			printf("SET BOOT TO DFU!!!\r\n\n");
			sl_bt_system_reset(2);
			// Send response to user write request.
			uint8_t att_errorcode = 0;
			sl_status_t sc = sl_bt_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					evt->data.evt_gatt_server_user_write_request.characteristic,
					att_errorcode);
			sl_app_assert(sc == SL_STATUS_OK,
					"[E: 0x%04x] Failed to send response to user write request\n",
					(int)sc);
			// Close connection to enter to DFU OTA mode
			/*sl_status_t sc = sl_bt_connection_close(
					evt->data.evt_gatt_server_user_write_request.connection);
			sl_app_assert(sc == SL_STATUS_OK,
					"[E: 0x%04x] Failed to close connection to enter to DFU OTA mode\n",
					(int)sc);*/
			break;
		default:
			// printf("WRITE TO WTF\r\n\n");
			break;
		}

		break;
	case sl_bt_evt_system_external_signal_id:
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_AMP_NOTIFY) {
			//double s[NRELAYS];

			for (int i = 0; i < live_connections; i++) {
				sl_bt_gatt_server_send_characteristic_notification(
						connections[i].connection_handle,
						gattdb_amps, sizeof(double) * NRELAYS,
						(uint8_t*) &currents,
						NULL);
				//sl_app_assert(sc == SL_STATUS_OK,
				//		"[E: 0x%04x] Failed to send user notify\n", (int )sc);
			}
		}
		if (evt->data.evt_system_external_signal.extsignals
				& SIGNAL_RELAY_NOTIFY) {
			for (int i = 0; i < live_connections; i++) {
				sl_bt_gatt_server_send_characteristic_notification(
				//0xFF,
						connections[i].connection_handle,
						gattdb_switch,
						NRELAYS, (uint8_t*) &relay_state,
						NULL);
				//sl_app_assert(sc == SL_STATUS_OK,
				//		"[E: 0x%04x] Failed to send user notify\n", (int )sc);
			}
		}
		if (evt->data.evt_system_external_signal.extsignals
				& SIGNAL_SWITCH_TOGGLE) {
			if (_char_handle > 0) {
				uint8_t toggle[NRELAYS] = { 0x03, 0x03, 0x02 };
				sl_bt_gatt_write_characteristic_value(_conn_handle,
						_char_handle,
						NRELAYS, toggle);
				printf("TOGGLED!\r\n\n");

			}
			//printf("TOGGLE IS %d\r\n\n",sc);
		}
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_I2C_CHECK) {
#if T_TYPE == T_RELAY
			uint16_t thundi_id = thundipi_read_id(&thundipii2c_sensor);
			if (thundi_id == 0xF1E2
					&& i2c_thundi_state == I2C_THUNDI_DISCONNECTED) {
				i2c_thundi_state = I2C_THUNDI_CONNECTED;
				printf("Physically connected to thunidpi !\r\n\n");
				thundipi_write_address(&thundipii2c_sensor, address.addr);
				set_discoverable();
				//read the other side address
				uint8_t target_address[6];
				char target_address_str[20];
				thundipi_read_slave_address(&thundipii2c_sensor,
						target_address);
				bd_addr_to_char(target_address, target_address_str);
				printf("Remote ADDR %s\r\n\n", target_address_str);
			} else {
				i2c_thundi_state = I2C_THUNDI_DISCONNECTED;
			}
#endif

#if T_TYPE == T_SWITCH
			if (app_state==SWITCH_I2C_WAIT) {
				uint8_t master_indicator=thundipi_read_master_indicator();
				if (master_indicator==1) {
					printf("MASTER INDICATOR FOUND!\r\n\n");
				}
				app_state=SWITCH_SCAN;
				sl_bt_scanner_start(1, scanner_discover_generic);
			}
#endif
		}
		if (evt->data.evt_system_external_signal.extsignals
				& SIGNAL_PASSKEY_ACCEPT) {
			sl_bt_sm_passkey_confirm(bt_bonding_connection, 1);
			printf("CONRIMGED\r\n\n");
			passkey_repeats = 0;
			stop_passkey_timer();
		}
		if (evt->data.evt_system_external_signal.extsignals
				& SIGNAL_PASSKEY_CHECK) {
#if T_TYPE == T_RELAY
			uint32_t slave_passkey = thundipi_read_passkey(&thundipii2c_sensor);
			printf("READ PASSKEY FOM SLAVE %d vs %d \r\n\n", slave_passkey,
					bt_bonding_passkey);
			if (slave_passkey == bt_bonding_passkey) {

				printf("Trying to bonding\r\n\n", slave_passkey); //TODO PHONE BONDING HERE!
				sl_bt_sm_passkey_confirm(bt_bonding_connection, 1);
			} else if ((passkey_repeats--) > 0) {
				start_passkey_timer();
			}
#endif

#if T_TYPE == T_SWITCH
			//sl_bt_sm_passkey_confirm(evt->data.evt_sm_confirm_passkey.connection,1);
			uint32_t their_key = get_their_key();
			if (bt_bonding_passkey==their_key) {
				sl_bt_sm_passkey_confirm(bt_bonding_connection,1);
				printf("Trying to bonding!\r\n\n",bt_bonding_passkey,their_key);
			} else if ((passkey_repeats--)>0) {
				start_passkey_timer();
			}
#endif
		}
#if T_TYPE == T_RELAY
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_MONITOR) {
			INA3221_accumulate_mW(&ina3221_sensor, 1);
			INA3221_accumulate_mW(&ina3221_sensor, 2);
			INA3221_accumulate_mW(&ina3221_sensor, 3);

		}
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_NVM_SAVE) {
			update_stats();
			print_stats();

		}
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_PRESS_HOLD) {
			printf("HOLDING\r\n\n");

			//sl_bt_system_reset(2);
			set_discoverable();
		}
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_DFU_HOLD) {
			printf("DFU MOD!\r\n\n");

			sl_bt_system_reset(2);
		}

		if (evt->data.evt_system_external_signal.extsignals
				& SIGNAL_SETUP_TIMEOUT) {
			printf("SETUP TIMEOUT\r\n\n");
			unset_discoverable();
		}
#endif

#if T_TYPE == T_SWITCH
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_PRESS_HOLD) {
			printf("HOLDING\r\n\n");
			//set_discoverable();
		}
#endif

		break;
		break;
	}
}

void set_discoverable() {
	//start_setup_led_timer();
	app_state = RELAY_PAIR;
	start_setup_timer();
	// Set advertising interval to 100ms.
	sl_status_t sc = sl_bt_advertiser_set_timing(advertising_set_handle, 160, // min. adv. interval (milliseconds * 1.6)
			160, // max. adv. interval (milliseconds * 1.6)
			0,   // adv. duration
			0);  // max. num. adv. events
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to set advertising timing\n", (int )sc);
	// Start general advertising and enable connections.
	sc = sl_bt_advertiser_start(advertising_set_handle,
			advertiser_general_discoverable, advertiser_connectable_scannable);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to start advertising\n", (int )sc);
}

void unset_discoverable() {
	stop_setup_timer();
	sl_bt_advertiser_stop(advertising_set_handle);
	app_state = RELAY_SERVE;
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/

void sl_bt_on_event(sl_bt_msg_t *evt) {
	sl_status_t sc;
	uint8_t address_type;
	//uint8_t system_id[8];

	switch (SL_BT_MSG_ID(evt->header)) {
	// -------------------------------
	// This event indicates the device has started and the radio is ready.
	// Do not call any stack command before receiving this boot event!
	case sl_bt_evt_system_boot_id:

		// Extract unique ID from BT Address.
		sc = sl_bt_system_get_identity_address(&address, &address_type);
#if T_TYPE==T_SWITCH
		thundipi_write_slave_addr(address.addr);
#endif

		sl_app_assert(sc == SL_STATUS_OK,
				"[E: 0x%04x] Failed to get Bluetooth address\n", (int )sc);
#if T_TYPE==T_SWITCH
		uint8_t switch_name[] = { 0x74, 0x68, 0x75, 0x6E, 0x64, 0x69, 0x73,
				0x77, 0x69, 0x74, 0x63, 0x68 };
		uint16_t result = sl_bt_gatt_server_write_attribute_value(
		gattdb_device_name, 0, sizeof(switch_name), switch_name);
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

		sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id, 0,
				sizeof(system_id), system_id);
		sl_app_assert(sc == SL_STATUS_OK,
				"[E: 0x%04x] Failed to write attribute WTF\n", (int )sc);

		// Create an advertising set.
		sc = sl_bt_advertiser_create_set(&advertising_set_handle);
		sl_app_assert(sc == SL_STATUS_OK,
				"[E: 0x%04x] Failed to create advertising set\n", (int )sc);

		break;

		// -------------------------------
		// This event indicates that a new connection was opened.
	case sl_bt_evt_connection_opened_id:
		sc = sl_bt_advertiser_start(advertising_set_handle,
				advertiser_general_discoverable,
				advertiser_connectable_scannable);
		sl_app_assert(sc == SL_STATUS_OK,
				"[E: 0x%04x] Failed to restart advertising 1\n", (int )sc); // restart advertising right after?
		break;

		// -------------------------------
		// This event indicates that a connection was closed.
	case sl_bt_evt_connection_closed_id:
		/*/ Restart advertising after client has disconnected.
		 /sc = sl_bt_advertiser_start(advertising_set_handle,
		 advertiser_general_discoverable,
		 advertiser_connectable_scannable);
		 sl_app_assert(sc == SL_STATUS_OK,
		 "[E: 0x%04x] Failed to restart advertising 2\n", (int )sc);*/
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

