#include "servos_controller.h"


static GPIO_TypeDef* SERVO_PORT[SERVO_COUNT] = {
    SERVO1_EN_GPIO_Port, SERVO2_EN_GPIO_Port, SERVO3_EN_GPIO_Port, SERVO4_EN_GPIO_Port,
    SERVO5_EN_GPIO_Port, SERVO6_EN_GPIO_Port, SERVO7_EN_GPIO_Port, SERVO8_EN_GPIO_Port,
    SERVO9_EN_GPIO_Port, SERVO10_EN_GPIO_Port, SERVO11_EN_GPIO_Port, SERVO12_EN_GPIO_Port,
    SERVO13_EN_GPIO_Port, SERVO14_EN_GPIO_Port, SERVO15_EN_GPIO_Port, SERVO16_EN_GPIO_Port,
    SERVO17_EN_GPIO_Port, SERVO18_EN_GPIO_Port
};

static uint16_t SERVO_PIN[SERVO_COUNT] = {
    SERVO1_EN_Pin, SERVO2_EN_Pin, SERVO3_EN_Pin, SERVO4_EN_Pin,
    SERVO5_EN_Pin, SERVO6_EN_Pin, SERVO7_EN_Pin, SERVO8_EN_Pin,
    SERVO9_EN_Pin, SERVO10_EN_Pin, SERVO11_EN_Pin, SERVO12_EN_Pin,
    SERVO13_EN_Pin, SERVO14_EN_Pin, SERVO15_EN_Pin, SERVO16_EN_Pin,
    SERVO17_EN_Pin, SERVO18_EN_Pin
};


extern ServoControllerState servos;


void Servos_ApplyImmediateOff(uint32_t mask) {
    for(uint8_t i = 0; i < SERVO_COUNT; i++) {
        if(mask & (1 << i)) {
            HAL_GPIO_WritePin(SERVO_PORT[i], SERVO_PIN[i], GPIO_PIN_RESET);
            servos.current_mask &= ~(1 << i);
        }
    }
}


void Servos_SetTargetMask(uint32_t new_mask) {
    servos.target_mask = new_mask;
    servos.done = 0;

    uint32_t to_turn_off = (~new_mask) & servos.current_mask;
    Servos_ApplyImmediateOff(to_turn_off);
}


void Servos_Tick(void) {
	static uint8_t tick_counter = 0;

    if(servos.done != 0) {
    	tick_counter = 0;
        return;
    }

    if(tick_counter == EN_SERVOS_TICK_DELAY) {
    	for(uint8_t i = 0; i < SERVO_COUNT; i++) {
			uint32_t mask = (1 << i);
			if((servos.target_mask & mask) && !(servos.current_mask & mask)) {
				HAL_GPIO_WritePin(SERVO_PORT[i], SERVO_PIN[i], GPIO_PIN_SET);
				servos.current_mask |= mask;
				tick_counter = 0;
				return;
			}
		}
		servos.done = 1;
    }
    tick_counter++;
}
