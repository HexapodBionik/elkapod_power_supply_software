#ifndef BUZZER_CONTROLLER_H
#define BUZZER_CONTROLLER_H

#include "tim.h"

typedef enum {
    BUZZ_LOW_SHORT  = 0,
    BUZZ_LOW_LONG   = 1,
    BUZZ_HIGH_SHORT = 2,
    BUZZ_HIGH_LONG  = 3
} BuzzerSound;

#define BUZZ_SHORT_DURATION   50
#define BUZZ_LONG_DURATION    150
#define BUZZ_PAUSE_DURATION   50

#define BUZZ_FREQ_LOW   2000
#define BUZZ_FREQ_HIGH  4000

typedef struct {
    uint8_t remaining_beeps;
    uint8_t active;
    uint16_t beep_duration;
    uint16_t pause_duration;
    uint16_t freq;
    uint16_t counter;
    uint8_t phase;   // 0 = beep, 1 = mute
} BuzzerController;


void Buzzer_Init(void);
void Buzzer_Start(uint8_t times, BuzzerSound type);
void Buzzer_Tick(void);

#endif //BUZZER_CONTROLLER_H
