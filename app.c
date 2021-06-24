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
#include "ota.h"
#include "bt_get_set.h"

uint8_t live_connections = 0;
connection connections[8];

const uint8_t UUID_SERVICE[2] = { 0x15, 0x18 }; // backwards
const uint8_t UUID_RELAY_CHAR[2] = { 0x56, 0x2A }; // backwards

int app_state = IDLE;
int i2c_thundi_state = I2C_THUNDI_DISCONNECTED;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

struct I2C_INA3221 ina3221_sensor;
struct I2C_THUNDIPII2C thundipii2c_sensor;

static uint32_t _service_handle = 0;
static uint16_t _char_handle = 0;
static uint8_t _conn_handle = 0;

static uint16_t passkey_repeats = 0;

//double currents[NRELAYS];
//double voltages[NRELAYS];
//static bool boot_to_dfu = false;
bd_addr address;

/**************************************************************************//**
 * App BT
 *****************************************************************************/
SL_WEAK void app_init(void) {
	printf("IN INIT x1\r\n\n");
	//ota_init();

	init_stats();
	init_timers();
	int pin, port;
#if T_TYPE == T_RELAY
	for (int idx = 0; idx < NRELAYS; idx++) {
		channel_to_port_and_pin(idx, PIN_SET, &port, &pin);
		GPIO_PinModeSet(port, pin, gpioModePushPull, 0);
		channel_to_port_and_pin(idx, PIN_UNSET, &port, &pin);
		GPIO_PinModeSet(port, pin, gpioModePushPull, 0);
	}
	sl_ina3221_init(&ina3221_sensor, INA3221_ADDRESS, 0.005); //LVK25 , 0.005 tol 0.5%
	start_monitor_timer();
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

}

SL_WEAK void app_process_action(void) {

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
	if (memcmp(response->address.addr, thundipi_read_master_addr(), 6) != 0) { // not the right address
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

#if T_TYPE==T_SWITCH
	{
		uint8_t switch_name[] = { 0x74, 0x68, 0x75, 0x6E, 0x64, 0x69, 0x73,
				0x77, 0x69, 0x74, 0x63, 0x68 };
		sl_bt_gatt_server_write_attribute_value(
		gattdb_device_name, 0, sizeof(switch_name), switch_name);
	}
#endif
		sl_bt_sm_delete_bondings();
		sl_bt_sm_store_bonding_configuration(8, 2);

		sl_bt_sm_configure(0x0B, sm_io_capability_displayyesno); //sm_io_capability_displayonly); //0x0F, sm_io_capability_displayyesno);// );

#if T_TYPE == T_RELAY
		//sl_bt_sm_set_bondable_mode(1);
		sl_bt_sm_set_bondable_mode(0);
		start_advertising();
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
		sl_bt_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection,
				1);
		printf("Return from bonding\r\n\n");

		break;

	case sl_bt_evt_connection_parameters_id:
		printf("CONNETION LEVEL %d\r\n\n",evt->data.evt_connection_parameters.security_mode);
		if (evt->data.evt_connection_parameters.security_mode < 2) {
			/*if (app_state == RELAY_PAIR) {
				sl_bt_sm_increase_security(
						evt->data.evt_connection_parameters.connection);
				printf("INCREASE SECURITY\r\n");
			} else {
				printf("Not increasing security, not in bonding mode\r\n\n");
			}*/
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

	case sl_bt_evt_sm_passkey_display_id:
		printf("Passkey: %06lu", evt->data.evt_sm_passkey_display.passkey);
		break;
		// Event raised by the security manager when a passkey needs to be confirmed
	case sl_bt_evt_sm_confirm_passkey_id:
		printf(
				"Do you see this passkey on the other device: %06lu? (y/n)\r\n\n",
				evt->data.evt_sm_confirm_passkey.passkey);
		bt_bonding_passkey = evt->data.evt_sm_confirm_passkey.passkey;
		bt_bonding_connection = evt->data.evt_sm_confirm_passkey.connection;

#if T_TYPE==T_REALY
		app_state = RELAY_CONFIRM;
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

	case sl_bt_evt_connection_opened_id:
		if (evt->data.evt_connection_opened.bonding == 0xFF) {
			/*if (app_state == RELAY_PAIR) {
				printf("Increasing securityXX\r\n");
				sl_bt_sm_increase_security(
						evt->data.evt_connection_opened.connection);
			}*/
		} else {
			printf("Already Bonded (ID: %d)\r\n",
					evt->data.evt_connection_opened.bonding);
		}
		connections[live_connections].device_address =
				evt->data.evt_connection_opened.address;
		connections[live_connections].address_type =
				evt->data.evt_connection_opened.address_type;
		connections[live_connections].connection_handle =
				evt->data.evt_connection_opened.connection;
		live_connections++;
		{
			char ary[20];
			for (int i = 0; i < live_connections; i++) {
				//compare address!
				bd_addr_to_char(connections[i].device_address.addr, ary);
				printf(" > %s\r\n\n", ary);
			}
		}
		break;

	case sl_bt_evt_connection_closed_id:
		printf("Connection closed %d\n", live_connections);
		int found = -1;
		for (int i = 0; i < live_connections; i++) {
			if (evt->data.evt_connection_closed.connection
					== connections[i].connection_handle) {
				found = i;
			}
		}
		if (found == -1) {
			printf("FAILE TO FIND THE ONE TO REMOVE...\r\n\n");
		} else {
			for (int j = found + 1; j < live_connections; j++) {
				connections[j - 1] = connections[j];
			}
		}
		live_connections--;

#if T_TYPE == T_SWITCH
		//sl_bt_scanner_start(1, scanner_discover_generic);
		app_state = SWITCH_I2C_WAIT;
#endif

		break;

	case sl_bt_evt_gatt_server_user_read_request_id:
		//printf("sl_bt_evt_gatt_server_user_read_request_id\r\n\n");
		switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
		case gattdb_switch:
			bt_read_relay_state(&evt->data.evt_gatt_server_user_read_request);
			break;
		case gattdb_amps:
			///printf("Reading from amps\r\n\n");
			bt_read_currents(&evt->data.evt_gatt_server_user_read_request);
			break;
		case gattdb_lifetime_stats:
			bt_send_lifetime_stats(
					&evt->data.evt_gatt_server_user_read_request);
			break;
		case gattdb_trip_stats:
			bt_send_trip_stats(&evt->data.evt_gatt_server_user_read_request);
			break;
		case gattdb_voltages:
			bt_send_voltages(&evt->data.evt_gatt_server_user_read_request);
			break;
		case gattdb_version:
			bt_send_version(&evt->data.evt_gatt_server_user_read_request);
			break;
		default:
			printf("Reading from ? %d\r\n\n",
					evt->data.evt_gatt_server_user_read_request.characteristic);
		}
		break;

	case sl_bt_evt_gatt_server_user_write_request_id:
		switch (evt->data.evt_gatt_server_user_read_request.characteristic) {
		case gattdb_switch:
			bt_write_relay_state(&evt->data.evt_gatt_server_user_write_request);

			break;

		default:
			break;
		}
		break;

	case sl_bt_evt_system_external_signal_id:
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_AMP_NOTIFY) {
			for (int i = 0; i < live_connections; i++) {
				bt_send_amp_notify(connections[i].connection_handle);
				bt_send_voltage_notify(connections[i].connection_handle);
				bt_send_lifetime_notify(connections[i].connection_handle);
				bt_send_trip_notify(connections[i].connection_handle);
			}
		}
		if (evt->data.evt_system_external_signal.extsignals
				& SIGNAL_RELAY_NOTIFY) {
			for (int i = 0; i < live_connections; i++) {
				sl_bt_gatt_server_send_characteristic_notification(
						connections[i].connection_handle,
						gattdb_switch,
						NRELAYS, (uint8_t*) &relay_state,
						NULL);
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
			if (app_state == SWITCH_I2C_WAIT) {
				uint8_t master_indicator = thundipi_read_master_indicator();
				if (master_indicator == 1) {
					printf("MASTER INDICATOR FOUND!\r\n\n");
				}
				app_state = SWITCH_SCAN;
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
			if (bt_bonding_passkey == their_key) {
				sl_bt_sm_passkey_confirm(bt_bonding_connection, 1);
				printf("Trying to bonding!\r\n\n", bt_bonding_passkey,
						their_key);
			} else if ((passkey_repeats--) > 0) {
				start_passkey_timer();
			}
#endif
		}
#if T_TYPE == T_RELAY
		if (evt->data.evt_system_external_signal.extsignals & SIGNAL_MONITOR) {
			printf("SIGNAL MONITOR!!!\r\n\n");
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

			sl_bt_sm_delete_bondings();
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

void start_advertising() {
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

void stop_advertising() {
	sl_bt_advertiser_stop(advertising_set_handle);
}

void set_discoverable() {
	//start_setup_led_timer();
	app_state = RELAY_PAIR;
	sl_bt_sm_set_bondable_mode(1);
	start_setup_timer();
}

void unset_discoverable() {
	stop_setup_timer();
	sl_bt_sm_set_bondable_mode(0);
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

