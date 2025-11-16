#include "pots_controller.h"

extern PotChannel pots[4];


static inline void Delay_dependent_pot_value(uint16_t pot_value) {
	if(pot_value < 20) {
		HAL_Delay(5);
	} else if(pot_value >= 20 && pot_value <= 50) {
		HAL_Delay(2);
	} else {
		HAL_Delay(1);
	}
}


void Pots_Tick(void) {
	for(uint8_t i = 0; i < 4; i++) {
		PotChannel* pot = &pots[i];

		if(pot->done != 0) continue;

		if(pot->target_value > pot->current_value) {
			MCP4552_increment_volatile(pot->pot);
			pot->current_value++;
			Delay_dependent_pot_value(pot->current_value);
		} else if(pot->target_value < pot->current_value) {
			MCP4552_decrement_volatile(pot->pot);
			pot->current_value--;
			Delay_dependent_pot_value(pot->current_value);
		} else {
			MCP4552_write_volatile(pot->pot, pot->target_value);
			pot->done = 1;
		}
	}
}

