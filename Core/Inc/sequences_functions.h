#ifndef SEQUENCES_FUNCTIONS_H_
#define SEQUENCES_FUNCTIONS_H_

#include "main.h"
#include "hardware_config.h"
#include "i2c_manager.h"
#include "pcf8574.h"
#include "adc121s021.h"
#include "mcp4552.h"
#include "can_app.h"
#include "pots_controller.h"
#include "servos_controller.h"
#include "voltage_outputs_controller.h"
#include "buttons_leds.h"

#include "tim.h"


typedef struct {
    uint8_t tim7_running;
    uint8_t tim8_pwm_running;
    uint8_t tim17_running;
} TimerStatus;


void power_on_seqence(void);
void wake_up_sequence(void);
void power_off_sequence(uint8_t additional_delay);
void bat_measurement_in_sleep_sequence(void);

void setup_pots(void);

void turn_off_all_LEDs(void);


#endif // SEQUENCES_FUNCTIONS_H_
