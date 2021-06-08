#include "timers.h"
#include "app.h"
#include "relay.h"
#include "sl_sleeptimer.h"
#include "button.h"
#include "em_gpio.h"
#include "sl_app_assert.h"

static sl_sleeptimer_timer_handle_t button_debounce_timers[NBUTTONS];
static sl_sleeptimer_timer_handle_t relay_timers[NRELAYS];
static sl_sleeptimer_timer_handle_t amp_notify_timer;
static sl_sleeptimer_timer_handle_t i2c_check_timer;
static sl_sleeptimer_timer_handle_t passkey_check_timer;

static uint32_t relay_delay_ticks = 0;
static uint32_t button_delay_ticks = 0;
static uint32_t amp_delay_ticks = 0;
static uint32_t i2c_delay_ticks = 0;
static uint32_t passkey_delay_ticks = 0;


void init_timers() {
	sl_sleeptimer_init();
	relay_delay_ticks = ((uint64_t) RELAY_DLAY_MSEC
			* sl_sleeptimer_get_timer_frequency()) / 1000;
	button_delay_ticks = ((uint64_t) BUTTON_DLAY_MSEC
			* sl_sleeptimer_get_timer_frequency()) / 1000;
	amp_delay_ticks = ((uint64_t) AMP_DLAY_MSEC
			* sl_sleeptimer_get_timer_frequency()) / 1000;
	i2c_delay_ticks = ((uint64_t) I2C_DLAY_MSEC
			* sl_sleeptimer_get_timer_frequency()) / 1000;
	passkey_delay_ticks = ((uint64_t) PASSKEY_DLAY_MSEC
			* sl_sleeptimer_get_timer_frequency()) / 1000;



#if T_TYPE == T_RELAY
	sl_sleeptimer_start_periodic_timer(&amp_notify_timer,
			amp_delay_ticks, amp_notify_timer_callback,
			NULL, 0, 0);
#endif

	sl_sleeptimer_start_periodic_timer(&i2c_check_timer,
			i2c_delay_ticks, i2c_check_timer_callback,
			NULL, 0, 0);
}

/*    */
void start_relay_timer(int idx) {
	sl_sleeptimer_start_timer(relay_timers + idx, relay_delay_ticks,
			relay_timer_callback, (void*) (int) idx, 0, 0);
}

void start_debounce_timer(int idx) {
sl_sleeptimer_start_timer(button_debounce_timers + idx,
		button_delay_ticks, button_debounce_timer_callback,
		(void*) (int) idx, 0, 0);
}

void start_passkey_timer() {
	sl_sleeptimer_start_timer(&passkey_check_timer, passkey_delay_ticks,passkey_check_timer_callback, (void*)0x0, 0, 0);

}
/**************************************************************************//**
 * Timers
 *****************************************************************************/
void relay_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data) {
	(void) handle;
	int relay_idx = (int) data;
	int pin, port;
	channel_to_port_and_pin(relay_idx, PIN_UNSET, &port, &pin);
	GPIO_PinOutClear(port, pin);
	channel_to_port_and_pin(relay_idx, PIN_SET, &port, &pin);
	GPIO_PinOutClear(port, pin);
	relay_changing[relay_idx] = 0;
}

void button_debounce_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data) {
	(void) handle;
	int button_idx = (int) data;
	button_debounce_state[button_idx] = 0;
}

void amp_notify_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data) {
	(void) data;
	(void) handle;
	//TODO optimize to only if current changes
	sl_bt_external_signal(SIGNAL_AMP_NOTIFY);
}

void i2c_check_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data) {
	(void) data;
	(void) handle;
	//TODO optimize to only if current changes
	sl_bt_external_signal(SIGNAL_I2C_CHECK);
}

void passkey_check_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data) {
	(void) data;
	(void) handle;
	//TODO optimize to only if current changes
	sl_bt_external_signal(SIGNAL_PASSKEY_CHECK);
}
