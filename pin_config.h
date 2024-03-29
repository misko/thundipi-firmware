#include "em_gpio.h"
#include "app.h"

#define INA3221_CRITICAL_PORT gpioPortA
#define INA3221_CRITICAL_PIN 0
#define INA3221_WARNING_PORT gpioPortA
#define INA3221_WARNING_PIN 4
#define INA3221_TC_PORT gpioPortA
#define INA3221_TC_PIN 5
#define INA3221_PV_PORT gpioPortB
#define INA3221_PV_PIN 3

#if T_TYPE == T_RELAY

#define BUTTON1_LED_PORT gpioPortA
#define BUTTON1_LED_PIN 6
#define BUTTON1_PORT gpioPortC
#define BUTTON1_PIN 3

#define RELAY1_SET_PORT gpioPortC
#define RELAY1_SET_PIN 7
#define RELAY1_UNSET_PORT gpioPortD
#define RELAY1_UNSET_PIN 1

#define RELAY2_SET_PORT gpioPortC
#define RELAY2_SET_PIN 0
#define RELAY2_UNSET_PORT gpioPortC
#define RELAY2_UNSET_PIN 1

#define RELAY3_SET_PORT gpioPortC
#define RELAY3_SET_PIN 6
#define RELAY3_UNSET_PORT gpioPortC
#define RELAY3_UNSET_PIN 2

#else

#define BUTTON1_LED_PORT gpioPortC
#define BUTTON1_LED_PIN 0
#define BUTTON1_PORT gpioPortB
#define BUTTON1_PIN 0

#define BUTTON2_LED_PORT gpioPortC
#define BUTTON2_LED_PIN 1
#define BUTTON2_PORT gpioPortB
#define BUTTON2_PIN 1

#define BUTTON3_LED_PORT gpioPortC
#define BUTTON3_LED_PIN 2
#define BUTTON3_PORT gpioPortB
#define BUTTON3_PIN 2

#endif





