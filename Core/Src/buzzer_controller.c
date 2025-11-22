#include "buzzer_controller.h"


extern BuzzerController buzzer;


static void Buzzer_PWM_Start(uint16_t freq) {
    uint32_t timer_clk = 80000000;
    uint32_t psc = htim3.Instance->PSC + 1;
    uint32_t arr = (timer_clk / (psc * freq)) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr / 2);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}


static void Buzzer_PWM_Stop(void) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}


void Buzzer_Init(void) {
    buzzer.remaining_beeps = 0;
    buzzer.active = 0;
    buzzer.phase = 0;
}


void Buzzer_Start(uint8_t times, BuzzerSound type) {
    if(times == 0 || times > 4) return;

    buzzer.remaining_beeps = times;
    buzzer.active = 1;
    buzzer.phase = 0;
    buzzer.counter = 0;

    switch(type) {
        case BUZZ_LOW_SHORT:
            buzzer.freq = BUZZ_FREQ_LOW;
            buzzer.beep_duration = BUZZ_SHORT_DURATION;
            break;

        case BUZZ_LOW_LONG:
            buzzer.freq = BUZZ_FREQ_LOW;
            buzzer.beep_duration = BUZZ_LONG_DURATION;
            break;

        case BUZZ_HIGH_SHORT:
            buzzer.freq = BUZZ_FREQ_HIGH;
            buzzer.beep_duration = BUZZ_SHORT_DURATION;
            break;

        case BUZZ_HIGH_LONG:
            buzzer.freq = BUZZ_FREQ_HIGH;
            buzzer.beep_duration = BUZZ_LONG_DURATION;
            break;
    }

    buzzer.pause_duration = BUZZ_PAUSE_DURATION;
}


void Buzzer_Tick(void) {
    if(!buzzer.active) return;

    buzzer.counter++;

    if(buzzer.phase == 0) { 	// beep phase
        if(buzzer.counter == 1)
            Buzzer_PWM_Start(buzzer.freq);

        if(buzzer.counter >= buzzer.beep_duration) {
            Buzzer_PWM_Stop();
            buzzer.counter = 0;
            buzzer.phase = 1;
        }
    } else {
        if(buzzer.counter >= buzzer.pause_duration) {
            buzzer.counter = 0;
            buzzer.phase = 0;
            buzzer.remaining_beeps--;

            if(buzzer.remaining_beeps == 0) {
                buzzer.active = 0;
            }
        }
    }
}
