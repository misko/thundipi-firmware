#include "app.h"

extern uint8_t relay_state[NRELAYS];
extern uint8_t relay_changing[NRELAYS];


void relay_off(int idx);
void relay_on(int idx);
void relay_toggle(int idx);

