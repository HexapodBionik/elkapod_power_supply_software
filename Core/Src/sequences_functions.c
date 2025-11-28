#include "sequences_functions.h"

extern I2C_Manager hi2c2_mgr;

extern PCF8574_HandleTypeDef expander1;
extern PCF8574_HandleTypeDef expander2;

extern MCP4552_HandleTypeDef pot1;
extern MCP4552_HandleTypeDef pot2;
extern MCP4552_HandleTypeDef pot3;
extern MCP4552_HandleTypeDef pot4;

extern ADC121S021_HandleTypeDef spi_adcs[SPI_ADC_COUNT];

extern PotChannel pots[4];
extern ServoControllerState servos;

extern uint8_t manip_conv_en;

extern uint16_t converter_voltage_to_pot_value(float voltage, float R_FB1,
		float R_FB2, float R_FB3,
		float R_pot, float R_pot_offset);


TimerStatus timers = {0};


void Safe_Start_TIM_Base_IT(TIM_HandleTypeDef *htim, uint8_t *flag) {
    if (*flag == 0) {
        HAL_TIM_Base_Start_IT(htim);
        *flag = 1;
    }
}


void Safe_Stop_TIM_Base_IT(TIM_HandleTypeDef *htim, uint8_t *flag) {
    if (*flag == 1) {
        HAL_TIM_Base_Stop_IT(htim);
        *flag = 0;
    }
}


void Safe_Start_TIM_PWM(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t *flag) {
    if (*flag == 0) {
        HAL_TIM_PWM_Start(htim, channel);
        *flag = 1;
    }
}


void Safe_Stop_TIM_PWM(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t *flag) {
    if (*flag == 1) {
        HAL_TIM_PWM_Stop(htim, channel);
        *flag = 0;
    }
}


void power_on_seqence(void) {
	HAL_GPIO_WritePin(LED_POWER_ON_GPIO_Port, LED_POWER_ON_Pin, GPIO_PIN_SET);

	// enable VOLTAGE_EN
	HAL_GPIO_WritePin(VOLTAGE_EN_GPIO_Port, VOLTAGE_EN_Pin, GPIO_PIN_RESET);

	// expanders setup
	if(PCF7485_init(&expander1, &hi2c2_mgr, EXPANDER1_ADDRESS) != HAL_OK) {
	  Error_Handler();
	}

	if(PCF7485_init(&expander2, &hi2c2_mgr, EXPANDER2_ADDRESS) != HAL_OK) {
	  Error_Handler();
	}

	// disable all converters, disable HVC, enable POTs
	PCF7485_write_buffer_blocking(&expander1, 0b01111111);
	// disable all SPI ADC CSs pins
	PCF7485_write_buffer_blocking(&expander2, 0b11111111);

	// disable all servos outputs
	Servos_ApplyImmediateOff(0x3FFFF);

	// turn off all LEDS
	turn_off_all_LEDs();

	// set temperatures MUX to position 0
	HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);

	// disable additional outputs
	VoltageOutputs_ApplyImmediateAllOff();

	HAL_Delay(150);

	// setup pots
	if(MCP4552_init(&pot1, &hi2c2_mgr, POT1_ADDRESS) != HAL_OK) {
	  Error_Handler();
	}
	if(MCP4552_init(&pot2, &hi2c2_mgr, POT2_ADDRESS) != HAL_OK) {
	  Error_Handler();
	}
	if(MCP4552_init(&pot3, &hi2c2_mgr, POT3_ADDRESS) != HAL_OK) {
	  Error_Handler();
	}
	if(MCP4552_init(&pot4, &hi2c2_mgr, POT4_ADDRESS) != HAL_OK) {
	  Error_Handler();
	}

	// SPI ADCs setup
	for (uint8_t i = 0; i < SPI_ADC_COUNT; i++) {
		ADC121S021_init(&spi_adcs[i], &hspi3, &expander2, i); // i - pin number on PCF
	}

	HAL_Delay(1500);

	// disable POTs
	PCF7485_write_pin_blocking(&expander1, EXPANDER1_POT_EN, GPIO_PIN_SET);
	// disable VOLTAGE_EN
	HAL_GPIO_WritePin(VOLTAGE_EN_GPIO_Port, VOLTAGE_EN_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LED_POWER_ON_GPIO_Port, LED_POWER_ON_Pin, GPIO_PIN_RESET);

}


void wake_up_sequence(void) {
	HAL_GPIO_WritePin(LED_POWER_ON_GPIO_Port, LED_POWER_ON_Pin, GPIO_PIN_SET);

	// enable VOLTAGE_EN
	HAL_GPIO_WritePin(VOLTAGE_EN_GPIO_Port, VOLTAGE_EN_Pin, GPIO_PIN_RESET);

	// disable all servos outputs
	Servos_ApplyImmediateOff(0x3FFFF);

	// disable additional outputs
	VoltageOutputs_ApplyImmediateAllOff();

	// set temperatures MUX to position 0
	HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);

	// enable converters 1-3 and 5, disable HVC, enable POTs
	PCF7485_write_buffer_blocking(&expander1, 0b01101000);

	ErrorManager_FlashLoad();
	ErrorManager_UpdateState();

	HAL_Delay(1500);

	// FAN PWM
	Safe_Start_TIM_PWM(&htim8, TIM_CHANNEL_4, &timers.tim8_pwm_running);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 500); // 50% duty (500 / 999)

	// STM ADCs timer start
	Safe_Start_TIM_Base_IT(&htim7, &timers.tim7_running);

	// SPI ADCs timer start
	Safe_Start_TIM_Base_IT(&htim17, &timers.tim17_running);

	Pots_ResetCurrnetValue();
	setup_pots();

	if(CAN_App_Init(CAN_ID_MIN, CAN_ID_MAX) != HAL_OK) {
		Error_Handler();
	}

	Buzzer_Start(2, BUZZ_HIGH_SHORT);

}

void power_off_sequence(void) {
	// stop timers
	Safe_Stop_TIM_PWM(&htim8, TIM_CHANNEL_4, &timers.tim8_pwm_running);
	Safe_Stop_TIM_Base_IT(&htim7, &timers.tim7_running);
	Safe_Stop_TIM_Base_IT(&htim17, &timers.tim17_running);

	// set manipulator disable flag
	manip_conv_en = 0;

	// disable all converters, disable HVC, disable POTs
	PCF7485_write_buffer_blocking(&expander1, 0b11111111);
	// disable all SPI ADC CSs pins
	PCF7485_write_buffer_blocking(&expander2, 0b11111111);

	// disable all servos outputs
	Servos_ApplyImmediateOff(0x3FFFF);

	// disable additional outputs
	VoltageOutputs_ApplyImmediateAllOff();

	// turn off all LEDS
	turn_off_all_LEDs();

	HAL_Delay(1500);

	// disable VOLTAGE_EN
	HAL_GPIO_WritePin(VOLTAGE_EN_GPIO_Port, VOLTAGE_EN_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(LED_POWER_ON_GPIO_Port, LED_POWER_ON_Pin, GPIO_PIN_RESET);
}


void setup_pots(void) {
	pots[0].target_value = converter_voltage_to_pot_value(CONVERTER1_SETUP_VOLTAGE,
			CONV1_3_RFB1, CONV1_3_RFB2, CONV1_3_RFB3, POT_RESISTANCE, POT_RESISTANCE_OFFSET);
	pots[0].done = 0;
	pots[1].target_value = converter_voltage_to_pot_value(CONVERTER2_SETUP_VOLTAGE,
			CONV1_3_RFB1, CONV1_3_RFB2, CONV1_3_RFB3, POT_RESISTANCE, POT_RESISTANCE_OFFSET);
	pots[1].done = 0;
	pots[2].target_value = converter_voltage_to_pot_value(CONVERTER3_SETUP_VOLTAGE,
			CONV1_3_RFB1, CONV1_3_RFB2, CONV1_3_RFB3, POT_RESISTANCE, POT_RESISTANCE_OFFSET);
	pots[2].done = 0;
	pots[3].target_value = converter_voltage_to_pot_value(CONVERTER4_SETUP_VOLTAGE,
			CONV4_RFB1, CONV4_RFB2, CONV4_RFB3, POT_RESISTANCE, POT_RESISTANCE_OFFSET);
	pots[3].done = 0;
}


void turn_off_all_LEDs(void) {
	HAL_GPIO_WritePin(LED_FATAL_ERROR_GPIO_Port, LED_FATAL_ERROR_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_STATUS2_GPIO_Port, LED_STATUS2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_STATUS3_GPIO_Port, LED_STATUS3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_STATUS4_GPIO_Port, LED_STATUS4_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_CONV1_GPIO_Port, LED_CONV1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_CONV2_GPIO_Port, LED_CONV2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_CONV3_GPIO_Port, LED_CONV3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_CONV4_GPIO_Port, LED_CONV4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_CONV5_GPIO_Port, LED_CONV5_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_BAT_INDICATOR1_GPIO_Port, LED_BAT_INDICATOR1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BAT_INDICATOR2_GPIO_Port, LED_BAT_INDICATOR2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BAT_INDICATOR3_GPIO_Port, LED_BAT_INDICATOR3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BAT_INDICATOR4_GPIO_Port, LED_BAT_INDICATOR4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_BAT_INDICATOR5_GPIO_Port, LED_BAT_INDICATOR5_Pin, GPIO_PIN_RESET);

}
