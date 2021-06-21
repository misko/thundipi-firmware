#include "app.h"

extern uint8_t button_debounce_state[NRELAYS];
void
button_change(uint8_t idx);
void
button_to_port_and_pin(int button, int *port, int *pin);
void
channel_to_port_and_pin(int channel, int pin_type, int *port, int *pin);

void button_debounce_change(uint8_t idx);
