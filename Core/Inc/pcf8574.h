#ifndef PCF8574_EXPANDER_H
#define PCF8574_EXPANDER_H

#include "i2c.h"
#include "gpio.h"
#include "stm32l4xx_hal.h"

typedef void (*PCF8574_Callback_t)(uint8_t addr, HAL_StatusTypeDef status, void* user);
typedef void (*PCF8574_StartCallback_t)(void* user);

typedef struct{
    I2C_HandleTypeDef* hi2c;
    uint8_t addr;

    uint8_t write_buff;
    uint8_t read_buff;

    volatile uint8_t tx_busy;
	PCF8574_Callback_t tx_cb;
	void* tx_user;

	// optional one-shot start callback
	PCF8574_StartCallback_t start_cb;
	void* start_cb_user;
} PCF8574_HandleTypeDef;


HAL_StatusTypeDef PCF7485_init(PCF8574_HandleTypeDef* pcf, I2C_HandleTypeDef* hi2c, uint8_t addr);

HAL_StatusTypeDef PCF7485_write_buffer_blocking(PCF8574_HandleTypeDef* pcf, uint8_t data);
HAL_StatusTypeDef PCF7485_write_pin_blocking(PCF8574_HandleTypeDef* pcf, uint8_t pin, GPIO_PinState state);

HAL_StatusTypeDef PCF8574_write_buffer_async(PCF8574_HandleTypeDef* pcf, uint8_t data,
                                             PCF8574_Callback_t cb, void* user);
HAL_StatusTypeDef PCF8574_write_pin_async(PCF8574_HandleTypeDef* pcf, uint8_t pin, GPIO_PinState state,
                                          PCF8574_Callback_t cb, void* user);

HAL_StatusTypeDef PCF8574_set_start_callback(PCF8574_HandleTypeDef* pcf,
                                             PCF8574_StartCallback_t cb,
                                             void* user);
void PCF8574_clear_start_callback(PCF8574_HandleTypeDef* pcf);

GPIO_PinState PCF7485_read_pin_blocking(PCF8574_HandleTypeDef* pcf, uint8_t pin);


#endif //PCF8574_EXPANDER_H
