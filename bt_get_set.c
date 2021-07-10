#include "app.h"
#include "INA3221.h"
#include "relay.h"
#include "stats.h"
#include "gatt_db.h"

#include "sl_app_assert.h"

void bt_read_relay_state(
		sl_bt_evt_gatt_server_user_read_request_t *data) {
	sl_status_t sc;

	sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0,
			NRELAYS, relay_state,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}

void bt_read_currents(
		sl_bt_evt_gatt_server_user_read_request_t *data) {
	sl_status_t sc;

	sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0, sizeof(double) * NRELAYS,
			(const uint8_t*) currents,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}
void bt_write_relay_state(
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


void bt_send_trip_stats(sl_bt_evt_gatt_server_user_read_request_t *data) {
	sl_status_t sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0, sizeof(double) * NRELAYS,
			(const uint8_t*) trip_accumulator,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}

void bt_send_lifetime_stats(sl_bt_evt_gatt_server_user_read_request_t *data) {
	sl_status_t sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0, sizeof(double) * NRELAYS,
			(const uint8_t*) life_accumulator,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}

void bt_send_voltages(sl_bt_evt_gatt_server_user_read_request_t *data) {
	sl_status_t sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0, sizeof(double) * NRELAYS,
			(const uint8_t*) voltages,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}

void bt_send_amp_notify(uint8_t connection_handle) {
	sl_bt_gatt_server_send_characteristic_notification(
			connection_handle,
			gattdb_amps, sizeof(double) * NRELAYS,
			(uint8_t*) &currents,
			NULL);
}
void bt_send_voltage_notify(uint8_t connection_handle) {
	sl_bt_gatt_server_send_characteristic_notification(
			connection_handle,
			gattdb_voltages, sizeof(double) * NRELAYS,
			(uint8_t*) voltages,
			NULL);
}
void bt_send_lifetime_notify(uint8_t connection_handle) {
	sl_bt_gatt_server_send_characteristic_notification(
			connection_handle,
			gattdb_lifetime_stats, sizeof(double) * NRELAYS,
			(uint8_t*) life_accumulator,
			NULL);
}
void bt_send_trip_notify(uint8_t connection_handle) {
	sl_bt_gatt_server_send_characteristic_notification(
			connection_handle,
			gattdb_trip_stats, sizeof(double) * NRELAYS,
			(uint8_t*) trip_accumulator,
			NULL);
}


void bt_send_version(sl_bt_evt_gatt_server_user_read_request_t *data) {
	const int32_t app_version = APP_VERSION;
	sl_status_t sc = sl_bt_gatt_server_send_user_read_response(data->connection,
			data->characteristic, 0, sizeof(int32_t),
			(const uint8_t*) &app_version,
			NULL);
	sl_app_assert(sc == SL_STATUS_OK,
			"[E: 0x%04x] Failed to send user read response\n", (int )sc);
}
