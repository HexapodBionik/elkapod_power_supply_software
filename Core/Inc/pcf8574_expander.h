#ifndef PCF8574_EXPANDER_H
#define PCF8574_EXPANDER_H

#include "i2c.h"
#include "gpio.h"

typedef struct{
    uint8_t addr;
    I2C_HandleTypeDef* hi2c;

    uint8_t write_buff;
    uint8_t read_buff;
} PCF7485_HandleTypeDef;


HAL_StatusTypeDef PCF7485_init(PCF7485_HandleTypeDef* pcf, I2C_HandleTypeDef* hi2c, uint8_t addr);
void PCF7485_write_pin(PCF7485_HandleTypeDef* pcf, uint8_t pin, GPIO_PinState state);
void PCF7485_write_buffer(PCF7485_HandleTypeDef* pcf, uint8_t write_buffer);

GPIO_PinState PCF7485_read_pin(PCF7485_HandleTypeDef* pcf, uint8_t pin);

#endif PCF8574_EXPANDER_H
