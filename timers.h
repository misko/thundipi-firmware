#define RELAY_DLAY_MSEC 20
#define BUTTON_DLAY_MSEC 10
#define AMP_DLAY_MSEC 1000
#define I2C_DLAY_MSEC 5000
#define PASSKEY_DLAY_MSEC 1000
#define MONITOR_DLAY_SEC 0.5
#define MONITOR_DLAY_MSEC (MONITOR_DLAY_SEC*1000)
//#define NVM_SAVE_DLAY_MSEC (1000*60*30) //30 minutes
#define NVM_SAVE_DLAY_MSEC (1000*5) //5 second
#define PRESS_HOLD_DLAY_MSEC (1000*3) //3 second
#define DFU_HOLD_DLAY_MSEC (1000*6) //3 second
#define SETUP_DLAY_MSEC (1000*60) //3 second
#define SETUP_LED_DLAY_MSEC (250) //3 second
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
void
monitor_timer_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void nvm_save_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data);
void press_hold_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data);
void setup_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data);
void setup_led_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data);
void dfu_hold_timer_callback(sl_sleeptimer_timer_handle_t *handle,
		void *data) ;
void start_relay_timer(int idx);
void start_debounce_timer(int idx);
void stop_debounce_timer(int idx);
void start_passkey_timer();
void stop_passkey_timer();
void start_press_hold_timer();
void stop_press_hold_timer();
void start_dfu_hold_timer();
void stop_dfu_hold_timer();
void start_setup_led_timer();
void stop_setup_led_timer();
void start_setup_timer();
void stop_setup_timer();
void init_timers();
void start_monitor_timer();
void stop_monitor_timer();
