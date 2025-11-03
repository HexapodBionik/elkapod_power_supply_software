#include "adc121s021.h"

static inline void cs_low(ADC121S021_HandleTypeDef* adc) {
	PCF7485_write_pin(adc->pcf, adc->pcf_cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(ADC121S021_HandleTypeDef* adc) {
	PCF7485_write_pin(adc->pcf, adc->pcf_cs_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef ADC121S021_init(ADC121S021_HandleTypeDef* adc,
                                  SPI_HandleTypeDef* hspi,
								  PCF8574_HandleTypeDef* pcf,
								  uint8_t pcf_cs_pin) {
    adc->hspi = hspi;
    adc->pcf = pcf;
    adc->pcf_cs_pin = pcf_cs_pin;

    adc->rx_buff[0] = 0x00;
    adc->rx_buff[1] = 0x00;
    adc->value = 0;

    cs_high(adc);
    return HAL_OK;
}

uint16_t ADC121S021_read(ADC121S021_HandleTypeDef* adc) {
    while(HAL_SPI_GetState(adc->hspi) != HAL_SPI_STATE_READY);

    cs_low(adc);
    HAL_Delay(1);

    if (HAL_SPI_Receive_DMA(adc->hspi, adc->rx_buff, 2) != HAL_OK) {
        cs_high(adc);
        return 0xFFFF;
    }

    while(HAL_SPI_GetState(adc->hspi) != HAL_SPI_STATE_READY);
//
    HAL_Delay(1);
    cs_high(adc);

    uint16_t raw = ((uint16_t)adc->rx_buff[0] << 8) | adc->rx_buff[1];
    adc->value = raw & 0x0FFF;

    return adc->value;
}
