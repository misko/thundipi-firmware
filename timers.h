#define RELAY_DLAY_MSEC 20
#define BUTTON_DLAY_MSEC 150
#define AMP_DLAY_MSEC 1000
#define I2C_DLAY_MSEC 5000
#define PASSKEY_DLAY_MSEC 1000
#include "sl_sleeptimer.h"

void
relay_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void
button_debounce_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void
amp_notify_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void
i2c_check_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void
passkey_check_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data);

void start_relay_timer(int idx);
void start_debounce_timer(int idx);
void start_passkey_timer();
void init_timers();
