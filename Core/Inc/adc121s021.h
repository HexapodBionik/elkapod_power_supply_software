#ifndef ADC121S021_H
#define ADC121S021_H

#include "spi.h"
#include "pcf8574.h"
#include "gpio.h"


typedef void (*ADC121S021_Callback_t)(void* user, HAL_StatusTypeDef status, uint16_t value);


typedef struct {
    SPI_HandleTypeDef* hspi;
    PCF8574_HandleTypeDef* pcf;

    uint8_t pcf_cs_pin;

    uint8_t rx_buff[2];
    uint16_t value;

    volatile uint8_t busy;
	ADC121S021_Callback_t cb;
	void* user;
} ADC121S021_HandleTypeDef;

HAL_StatusTypeDef ADC121S021_init(ADC121S021_HandleTypeDef* adc,
                                  SPI_HandleTypeDef* hspi,
								  PCF8574_HandleTypeDef* pcf,
                                  uint8_t pcf_cs_pin);

uint16_t ADC121S021_read_blocking(ADC121S021_HandleTypeDef* adc);
HAL_StatusTypeDef ADC121S021_read_start(ADC121S021_HandleTypeDef* adc, ADC121S021_Callback_t cb, void* user);

#endif // ADC121S021_H
