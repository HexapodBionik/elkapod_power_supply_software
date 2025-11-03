#ifndef ADC121S021_H
#define ADC121S021_H

#include "spi.h"
#include "pcf8574.h"
#include "gpio.h"

typedef struct {
    SPI_HandleTypeDef* hspi;
    PCF8574_HandleTypeDef* pcf;

    uint8_t pcf_cs_pin;

    uint8_t rx_buff[2];
    uint16_t value;
} ADC121S021_HandleTypeDef;

HAL_StatusTypeDef ADC121S021_init(ADC121S021_HandleTypeDef* adc,
                                  SPI_HandleTypeDef* hspi,
								  PCF8574_HandleTypeDef* pcf,
                                  uint8_t pcf_cs_pin);

uint16_t ADC121S021_read(ADC121S021_HandleTypeDef* adc);

#endif // ADC121S021_H
