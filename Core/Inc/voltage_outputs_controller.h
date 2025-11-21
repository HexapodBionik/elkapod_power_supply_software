#ifndef VOLTAGE_OUTPUTS_CONTROLLER_H
#define VOLTAGE_OUTPUTS_CONTROLLER_H

#include "hardware_config.h"
#include "main.h"


typedef struct {
    uint8_t current_mask;		// bit mask: 1 = ON, 0 = OFF
    uint8_t target_mask;
    uint8_t done;              	// 0 = working, 1 = request to send ACK, 2 = idle
} VoltageOutputsController;


void VoltageOutputs_ApplyImmediateAllOff(void);
void VoltageOutputs_SetTargetMask(uint8_t new_mask);

void VoltageOutputs_Tick(void);

#endif
