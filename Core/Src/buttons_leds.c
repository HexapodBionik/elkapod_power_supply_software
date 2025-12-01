#include "buttons_leds.h"

extern float U_bat_ADC;
extern uint8_t fatal_error;


#define BAT_INDICATOR_HYSTERESIS 50.0f
#define BAT_FILTER_SIZE 16

static const float bat_thresholds[] = {
    BAT_20_VALUE,
    BAT_40_VALUE,
    BAT_60_VALUE,
    BAT_80_VALUE,
    BAT_100_VALUE
};

#define LED_COUNT (sizeof(bat_thresholds) / sizeof(bat_thresholds[0]))

static uint8_t bat_led_state[LED_COUNT] = {0};

static GPIO_TypeDef* const bat_led_ports[LED_COUNT] = {
    LED_BAT_INDICATOR1_GPIO_Port,
    LED_BAT_INDICATOR2_GPIO_Port,
    LED_BAT_INDICATOR3_GPIO_Port,
    LED_BAT_INDICATOR4_GPIO_Port,
    LED_BAT_INDICATOR5_GPIO_Port
};

static const uint16_t bat_led_pins[LED_COUNT] = {
    LED_BAT_INDICATOR1_Pin,
    LED_BAT_INDICATOR2_Pin,
    LED_BAT_INDICATOR3_Pin,
    LED_BAT_INDICATOR4_Pin,
    LED_BAT_INDICATOR5_Pin
};


static void BatIndicator_FilterAndUpdate(float new_val) {
	static float sum = 0.0f;
	static uint8_t index = 0;

	sum += new_val;
	index++;

    if (index >= BAT_FILTER_SIZE) {
    	BatIndicator_Update(sum/(float)BAT_FILTER_SIZE);
    	sum = 0.0f;
    	index = 0;
    }
}


void ButtonsLEDs_Tick(void) {
    uint32_t now = HAL_GetTick();

    static uint8_t sw_func_pushed = 0;
    static uint32_t sw_func_pushed_time = 0;

    static uint8_t sw_clr_err_pushed = 0;
    static uint32_t sw_clr_err_pushed_time = 0;

    if(HAL_GPIO_ReadPin(SW_FUNC_GPIO_Port, SW_FUNC_Pin) == GPIO_PIN_RESET) {
        if(sw_func_pushed == 0) {
            sw_func_pushed = 1;
            sw_func_pushed_time = now;
        }
    } else {
        if(sw_func_pushed == 1 && (now - sw_func_pushed_time) > DEBOUNCE_MS) {
            ErrorManager_LED_ViewNext();
        }
        sw_func_pushed = 0;
    }

    if(HAL_GPIO_ReadPin(SW_CLR_ERR_GPIO_Port, SW_CLR_ERR_Pin) == GPIO_PIN_RESET) {
        if(sw_clr_err_pushed == 0) {
            sw_clr_err_pushed = 1;
            sw_clr_err_pushed_time = now;
        } else if(sw_clr_err_pushed == 1 && (now - sw_clr_err_pushed_time) > LONG_PRESS_MS) {
            ErrorManager_LED_DeleteCurrent();
            sw_clr_err_pushed = 2;
        }
    } else {
		sw_clr_err_pushed = 0;
    }


    uint8_t code = ErrorManager_LED_CurrentCode();

    HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, (code & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_STATUS2_GPIO_Port, LED_STATUS2_Pin, (code & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_STATUS3_GPIO_Port, LED_STATUS3_Pin, (code & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_STATUS4_GPIO_Port, LED_STATUS4_Pin, (code & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint8_t conv = ErrorManager_LED_CurrentConvMask();

    HAL_GPIO_WritePin(LED_CONV1_GPIO_Port, LED_CONV1_Pin, (conv & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CONV2_GPIO_Port, LED_CONV2_Pin, (conv & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CONV3_GPIO_Port, LED_CONV3_Pin, (conv & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CONV4_GPIO_Port, LED_CONV4_Pin, (conv & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CONV5_GPIO_Port, LED_CONV5_Pin, (conv & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    BatIndicator_FilterAndUpdate(U_bat_ADC);

    HAL_GPIO_WritePin(LED_FATAL_ERROR_GPIO_Port, LED_FATAL_ERROR_Pin, fatal_error ? GPIO_PIN_SET : GPIO_PIN_RESET);

}


void BatIndicator_Update(float U_bat) {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        float threshold = bat_thresholds[i];

        if (U_bat > (threshold + BAT_INDICATOR_HYSTERESIS)) {
        	bat_led_state[i] = 1;
        } else if (U_bat < (threshold - BAT_INDICATOR_HYSTERESIS)) {
        	bat_led_state[i] = 0;
        }

        HAL_GPIO_WritePin(
            bat_led_ports[i],
            bat_led_pins[i],
            bat_led_state[i] ? GPIO_PIN_SET : GPIO_PIN_RESET
        );
    }
}
