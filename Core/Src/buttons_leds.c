#include "buttons_leds.h"


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

}
