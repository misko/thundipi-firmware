#include "pin_config.h"
#include "button.h"
#include "relay.h"

#include "sl_app_assert.h"
#include "app.h"
#include "timers.h"


uint8_t button_debounce_state[NRELAYS] = { 0, 0, 0 };
uint8_t button_state[NRELAYS] = { 0, 0, 0 };

/**************************************************************************//**
 * Button handling
 *****************************************************************************/

void button_to_port_and_pin(int button, int *port, int *pin) {
	switch (button) {

#if T_TYPE == T_RELAY
	case 0:
		*port = BUTTON1_PORT;
		*pin = BUTTON1_PIN;
		break;
	default:
		sl_app_assert(1 == 0, "[E: 0x%04x] Invalid button\n", button);
	}
#else
	case 0:
		*port = BUTTON1_PORT;
		*pin = BUTTON1_PIN;
		break;
	case 1:
		*port = BUTTON2_PORT;
		*pin = BUTTON2_PIN;
		break;
	case 2:
		*port = BUTTON3_PORT;
		*pin = BUTTON3_PIN;
		break;
	default:
		sl_app_assert(1 == 0, "[E: 0x%04x] Invalid button\n", button);
	}

#endif

}

void buttonled_to_port_and_pin(int button, int *port, int *pin) {
	switch (button) {

#if T_TYPE == T_RELAY
	case 0:
		*port = BUTTON1_LED_PIN;
		*pin = BUTTON1_LED_PORT;
		break;
	default:
		sl_app_assert(1 == 0, "[E: 0x%04x] Invalid button\n", button);
	}
#else
	case 0:
		*port = BUTTON1_LED_PIN;
		*pin = BUTTON1_LED_PORT;
		break;
	case 1:
		*port = BUTTON2_LED_PIN;
		*pin = BUTTON2_LED_PORT;
		break;
	case 2:
		*port = BUTTON3_LED_PIN;
		*pin = BUTTON3_LED_PORT;
		break;
	default:
		sl_app_assert(1 == 0, "[E: 0x%04x] Invalid button\n", button);
	}

#endif

}

#if T_TYPE == T_RELAY
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
#endif

void button_debounce_change(uint8_t idx) {
	if (button_state[idx] == 0) {
		start_dfu_hold_timer();

#if T_TYPE == T_RELAY

		if (app_state==RELAY_CONFIRM) {
			printf("CONFIRM the bonding\r\n\n");
			sl_bt_external_signal(SIGNAL_PASSKEY_ACCEPT);
			return;
		}
		start_press_hold_timer();
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

#endif
	} else {
		stop_press_hold_timer();
		stop_dfu_hold_timer();
	}

	int port,pin;
	buttonled_to_port_and_pin(idx,&port,&pin);

	//button_state[idx]=1-button_state[idx];

	printf("BUTTON STATE IS %d %d\r\n\n", idx,button_state[idx]);

	if (button_state[idx] == 0) {
		GPIO_PinOutClear(port, pin);
	} else {
		GPIO_PinOutSet(port, pin);
	}


	sl_bt_external_signal(SIGNAL_SWITCH_TOGGLE);

	button_debounce_state[idx]=0;
}

void button_change(uint8_t idx) {
	int port, pin;
	button_to_port_and_pin(idx, &port, &pin);
	int state = 1-GPIO_PinInGet(port, pin);
	for (int i=0; i<NRELAYS; i++) {
		printf("BUT %d/%d ", i, button_state[i]);
	}
	printf("\r\n\n");
	printf("STATE IS %d %d\r\n\n",idx ,state);
	if (button_debounce_state[idx]!=1) { // || state!=button_state[idx]) {
		//start a debounce for this state
		printf("set STATE IS %d %d\r\n\n",idx ,state);
		stop_debounce_timer(idx);
		button_state[idx]=state;
		button_debounce_state[idx]=1;
		start_debounce_timer(idx);
	}
}
