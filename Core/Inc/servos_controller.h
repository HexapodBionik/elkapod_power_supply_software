#ifndef SERVOS_CONTROLLER_H
#define SERVOS_CONTROLLER_H

#include "main.h"
#include "hardware_config.h"


typedef struct {
    uint32_t target_mask;   // 18-bit mask: 1 = ON, 0 = OFF
    uint32_t current_mask;
    uint8_t done;           // 0 = working, 1 = request to send ACK, 2 = idle
} ServoControllerState;


void Servos_ApplyImmediateOff(uint32_t mask);
void Servos_SetTargetMask(uint32_t new_mask);

void Servos_Tick(void);

#endif
