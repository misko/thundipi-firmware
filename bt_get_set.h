
void bt_read_relay_state(
		sl_bt_evt_gatt_server_user_read_request_t *data);
void bt_read_currents(
		sl_bt_evt_gatt_server_user_read_request_t *data);

void bt_write_relay_state(
		sl_bt_evt_gatt_server_user_write_request_t *data);

void bt_send_trip_stats(sl_bt_evt_gatt_server_user_read_request_t *data);
void bt_send_lifetime_stats(sl_bt_evt_gatt_server_user_read_request_t *data);
void bt_send_voltages(sl_bt_evt_gatt_server_user_read_request_t *data);
void bt_send_version(sl_bt_evt_gatt_server_user_read_request_t *data);


void bt_send_amp_notify(uint8_t connection_handle);
void bt_send_voltage_notify(uint8_t connection_handle);
void bt_send_lifetime_notify(uint8_t connection_handle);
void bt_send_trip_notify(uint8_t connection_handle);
