#ifndef ADC121S021_H
#define ADC121S021_H

#include "spi.h"
#include "gpio.h"

typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    uint8_t rx_buff[2];
    uint16_t value;
} ADC121S021_HandleTypeDef;

HAL_StatusTypeDef ADC121S021_init(ADC121S021_HandleTypeDef* adc,
                                  SPI_HandleTypeDef* hspi,
                                  GPIO_TypeDef* cs_port, uint16_t cs_pin);

uint16_t ADC121S021_read(ADC121S021_HandleTypeDef* adc);

#endif // ADC121S021_H
