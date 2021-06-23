#include "pin_config.h"
#include "button.h"
#include "relay.h"

#include "sl_app_assert.h"
#include "app.h"


uint8_t button_debounce_state[NRELAYS] = { 0, 0, 0 };
uint8_t button_state[NRELAYS] = { 255, 255, 255 };

/**************************************************************************//**
 * Button handling
 *****************************************************************************/

void button_to_port_and_pin(int button, int *port, int *pin) {
	switch (button) {
	case 0:
		*port = BUTTON1_PORT;
		*pin = BUTTON1_PIN;
		break;
	default:
		sl_app_assert(1 == 0, "[E: 0x%04x] Invalid button\n", button);
	}

}
void channel_to_port_and_pin(int channel, int pin_type, int *port, int *pin) {
	switch (channel) {
	case 0:
		if (pin_type == PIN_SET) {
			*port = RELAY1_SET_PORT;
			*pin = RELAY1_SET_PIN;
		} else {
			*port = RELAY1_UNSET_PORT;
			*pin = RELAY1_UNSET_PIN;
		}
		break;
	case 1:
		if (pin_type == PIN_SET) {
			*port = RELAY2_SET_PORT;
			*pin = RELAY2_SET_PIN;
		} else {
			*port = RELAY2_UNSET_PORT;
			*pin = RELAY2_UNSET_PIN;
		}
		break;
	case 2:
		if (pin_type == PIN_SET) {
			*port = RELAY3_SET_PORT;
			*pin = RELAY3_SET_PIN;
		} else {
			*port = RELAY3_UNSET_PORT;
			*pin = RELAY3_UNSET_PIN;
		}
		break;
	default:
		sl_app_assert(1 == 0, "[E: 0x%04x] Invalid channel\n", channel);
	}
}

void button_debounce_change(uint8_t idx) {
	if (button_state[idx] == 0) {
		if (app_state==RELAY_CONFIRM) {
			printf("CONFIRM the bonding\r\n\n");
			sl_bt_external_signal(SIGNAL_PASSKEY_ACCEPT);
			return;
		}
		start_press_hold_timer();
		start_dfu_hold_timer();

#if T_TYPE == T_RELAY
		switch (idx) {
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
#else
		sl_bt_external_signal(SIGNAL_SWITCH_TOGGLE);
		int state = GPIO_PinInGet(BUTTON1_LED_PORT, BUTTON1_LED_PIN);
		printf("STATE IS %d\r\n\n", state);
		if (state == 1) {
			GPIO_PinOutClear(BUTTON1_LED_PORT, BUTTON1_LED_PIN);
		} else {

			GPIO_PinOutSet(BUTTON1_LED_PORT, BUTTON1_LED_PIN);
		}
#endif
	} else {
		stop_press_hold_timer();
		stop_dfu_hold_timer();
	}
}

void button_change(uint8_t idx) {
	int port, pin;
	button_to_port_and_pin(idx, &port, &pin);
	int state = GPIO_PinInGet(port, pin);
	//printf("STATE IS %d\r\n\n",state);
	if (button_debounce_state[idx]!=1 || state!=button_state[idx]) {
		//start a debounce for this state
		stop_debounce_timer(idx);
		button_state[idx]=state;
		button_debounce_state[idx]=1;
		start_debounce_timer(idx);
	}
}
