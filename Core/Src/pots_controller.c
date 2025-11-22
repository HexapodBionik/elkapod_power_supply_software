#include "pots_controller.h"

extern PotChannel pots[4];


static inline uint8_t get_delay_dependent_pot_value(uint16_t pot_value) {
	if(pot_value < 20) {
		return 20;
	} else if(pot_value >= 20 && pot_value <= 50) {
		return 8;
	} else {
		return 4;
	}
}


void Pots_ResetCurrnetValue(void) {
	pots[0].current_value = 128;
	pots[1].current_value = 128;
	pots[2].current_value = 128;
	pots[3].current_value = 128;
}


void Pots_Tick(void) {
	static uint8_t tick_counter[4] = {0};

	for(uint8_t i = 0; i < 4; i++) {
		PotChannel* pot = &pots[i];

		if(pot->done != 0) {
			tick_counter[i] = 0;
			continue;
		}

		tick_counter[i]++;

		if(tick_counter[i] == get_delay_dependent_pot_value(pot->current_value)) {
			if(pot->target_value > pot->current_value) {
				MCP4552_increment_volatile(pot->pot);
				pot->current_value++;
				tick_counter[i] = 0;
			} else if(pot->target_value < pot->current_value) {
				MCP4552_decrement_volatile(pot->pot);
				pot->current_value--;
				tick_counter[i] = 0;
			} else {
				MCP4552_write_volatile(pot->pot, pot->target_value);
				pot->done = 1;
				tick_counter[i] = 0;
			}
		}
	}
}

