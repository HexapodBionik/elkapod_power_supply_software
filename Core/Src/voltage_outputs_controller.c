#include "voltage_outputs_controller.h"


extern VoltageOutputsController vouts;

static const GPIO_TypeDef* VOUT_PORT[VOUT_COUNT] = {
    V_OUT_EN1_GPIO_Port,
    V_OUT_EN2_GPIO_Port,
    V_OUT_EN3_GPIO_Port,
    V_OUT_EN4_GPIO_Port,
    V_OUT_EN5_GPIO_Port,
    V_OUT_EN6_GPIO_Port
};

static const uint16_t VOUT_PIN[VOUT_COUNT] = {
    V_OUT_EN1_Pin,
    V_OUT_EN2_Pin,
    V_OUT_EN3_Pin,
    V_OUT_EN4_Pin,
    V_OUT_EN5_Pin,
    V_OUT_EN6_Pin
};

static const uint8_t vout_active_high[VOUT_COUNT] = {
    1, 1, 1,    // V_OUT_EN1..3
    0, 0, 0     // V_OUT_EN4..6
};


void VoltageOutputs_ApplyImmediateAllOff(void) {
    for(uint8_t i = 0; i < VOUT_COUNT; i++) {
        if(vout_active_high[i])
            HAL_GPIO_WritePin(VOUT_PORT[i], VOUT_PIN[i], GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(VOUT_PORT[i], VOUT_PIN[i], GPIO_PIN_SET);
    }

    vouts.current_mask = 0;
    vouts.target_mask = 0;
    vouts.done = 2;
}


void VoltageOutputs_SetTargetMask(uint8_t new_mask) {
	vouts.target_mask = new_mask;
    vouts.done = 0;
}


void VoltageOutputs_Tick(void) {
    static uint8_t tick_counter = 0;

    if(vouts.done != 0) {
        tick_counter = 0;
        return;
    }

    if(tick_counter == VOUT_TICK_DELAY) {
        for(uint8_t i = 0; i < VOUT_COUNT; i++) {
            uint32_t mask = (1 << i);
            if((vouts.target_mask & mask) != (vouts.current_mask & mask)) {

                GPIO_PinState state;

                uint8_t want_on = (vouts.target_mask & mask) ? 1 : 0;

                if(vout_active_high[i]) {
                    state = want_on ? GPIO_PIN_SET : GPIO_PIN_RESET;
                } else {
                    state = want_on ? GPIO_PIN_RESET : GPIO_PIN_SET;
                }

                HAL_GPIO_WritePin(VOUT_PORT[i], VOUT_PIN[i], state);

                if(want_on) vouts.current_mask |= mask;
                else        vouts.current_mask &= ~mask;

                tick_counter = 0;
                return;
            }
        }
        vouts.done = 1;
    }
    tick_counter++;
}
