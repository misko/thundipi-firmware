#include "em_core.h"
#include "em_common.h"
#include "relay.h"
#include "app.h"
#include "button.h"
#include "timers.h"
#include "em_gpio.h"
#include "sl_app_assert.h"

uint8_t relay_state[NRELAYS] = { 0, 0, 0 };
uint8_t relay_changing[NRELAYS] = { 0, 0, 0 };

/**************************************************************************//**
 * Relay control
 *****************************************************************************/
void relay_off(int idx) {
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_ATOMIC();
	if (relay_changing[idx] == 1) {
		CORE_EXIT_ATOMIC();
		return;
	}
	relay_changing[idx] = 1;
	CORE_EXIT_ATOMIC();
	int port = 0;
	int pin = 0;
	channel_to_port_and_pin(idx, PIN_UNSET, &port, &pin);
	GPIO_PinOutSet(port, pin);
	start_relay_timer(idx);
	relay_state[idx] = 0;

	sl_bt_external_signal(SIGNAL_RELAY_NOTIFY);
}
void relay_on(int idx) {
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_ATOMIC();
	if (relay_changing[idx] == 1) {
		CORE_EXIT_ATOMIC();
		return;
	}
	relay_changing[idx] = 1;
	CORE_EXIT_ATOMIC();
	int port = 0;
	int pin = 0;
	channel_to_port_and_pin(idx, PIN_SET, &port, &pin);
	GPIO_PinOutSet(port, pin);

	start_relay_timer(idx);
	relay_state[idx] = 1;

	sl_bt_external_signal(SIGNAL_RELAY_NOTIFY);
}
void relay_toggle(int idx) {
	if (relay_state[idx] == 1) {
		relay_off(idx);
	} else {
		relay_on(idx);
	}
}
