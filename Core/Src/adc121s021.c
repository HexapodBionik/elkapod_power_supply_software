#include "adc121s021.h"
#include "main.h"

#ifndef ADC_MAX_INSTANCES
#define ADC_MAX_INSTANCES 8
#endif


static ADC121S021_HandleTypeDef* adc_registry[ADC_MAX_INSTANCES] = {0};


static int8_t adc_register(ADC121S021_HandleTypeDef* adc){
    for(uint8_t i = 0; i < ADC_MAX_INSTANCES; i++){
        if(adc_registry[i] == NULL) {
        	adc_registry[i] = adc;
        	return 0;
        }
    }
    return -1;
}


static ADC121S021_HandleTypeDef* find_busy_adc_by_spi(SPI_HandleTypeDef* hspi){
    for(uint8_t i = 0; i < ADC_MAX_INSTANCES; i++) {
        ADC121S021_HandleTypeDef* adc = adc_registry[i];
        if(adc && adc->hspi == hspi && adc->busy) {
        	return adc;
        }
    }
    return NULL;
}


static inline void cs_low_blocking(ADC121S021_HandleTypeDef* adc) {
	PCF7485_write_pin_blocking(adc->pcf, adc->pcf_cs_pin, GPIO_PIN_RESET);
}


static inline void cs_high_blocking(ADC121S021_HandleTypeDef* adc) {
	PCF7485_write_pin_blocking(adc->pcf, adc->pcf_cs_pin, GPIO_PIN_SET);
}


HAL_StatusTypeDef ADC121S021_init(ADC121S021_HandleTypeDef* adc,
                                  SPI_HandleTypeDef* hspi,
								  PCF8574_HandleTypeDef* pcf,
								  uint8_t pcf_cs_pin) {
	if(!adc || !hspi || !pcf) return HAL_ERROR;
    adc->hspi = hspi;
    adc->pcf = pcf;
    adc->pcf_cs_pin = pcf_cs_pin;

    adc->rx_buff[0] = 0x00;
    adc->rx_buff[1] = 0x00;
    adc->value = 0;

    adc->busy = 0;
	adc->cb = NULL;
	adc->user = NULL;

    cs_high_blocking(adc);
    adc_register(adc);
	return HAL_OK;
}


uint16_t ADC121S021_read_blocking(ADC121S021_HandleTypeDef* adc) {
    while(HAL_SPI_GetState(adc->hspi) != HAL_SPI_STATE_READY);

    cs_low_blocking(adc);
    HAL_Delay(1);

    if (HAL_SPI_Receive_DMA(adc->hspi, adc->rx_buff, 2) != HAL_OK) {
        cs_high_blocking(adc);
        return 0xFFFF;
    }

    while(HAL_SPI_GetState(adc->hspi) != HAL_SPI_STATE_READY);

    HAL_Delay(1);
    cs_high_blocking(adc);

    uint16_t raw = ((uint16_t)adc->rx_buff[0] << 8) | adc->rx_buff[1];
    adc->value = raw & 0x0FFF;

    return adc->value;
}


static void pcf_cs_low_done(uint8_t addr, HAL_StatusTypeDef status, void* user);
static void pcf_cs_high_done(uint8_t addr, HAL_StatusTypeDef status, void* user);

HAL_StatusTypeDef ADC121S021_read_start(ADC121S021_HandleTypeDef* adc,
                                        ADC121S021_Callback_t cb,
                                        void* user)
{
    if(!adc) return HAL_ERROR;
    if(adc->busy) return HAL_BUSY;

    __disable_irq();
    adc->busy = 1;
    adc->cb = cb;
    adc->user = user;
    __enable_irq();

    return PCF8574_write_pin_async(adc->pcf, adc->pcf_cs_pin,
                                   GPIO_PIN_RESET, pcf_cs_low_done, adc);
}


static void pcf_cs_low_done(uint8_t addr, HAL_StatusTypeDef status, void* user)
{
    ADC121S021_HandleTypeDef* adc = (ADC121S021_HandleTypeDef*)user;
    if(!adc) return;

    if(status != HAL_OK){
        adc->busy = 0;
        if(adc->cb) adc->cb(adc->user, HAL_ERROR, 0);
        return;
    }

    if(HAL_SPI_Receive_DMA(adc->hspi, adc->rx_buff, 2) != HAL_OK){
        PCF8574_write_pin_async(adc->pcf, adc->pcf_cs_pin,
                                GPIO_PIN_SET, pcf_cs_high_done, adc);
    }
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ADC121S021_HandleTypeDef* adc = find_busy_adc_by_spi(hspi);
    if(!adc) return;

    PCF8574_write_pin_async(adc->pcf, adc->pcf_cs_pin,
                            GPIO_PIN_SET, pcf_cs_high_done, adc);
}


static void pcf_cs_high_done(uint8_t addr, HAL_StatusTypeDef status, void* user)
{
    ADC121S021_HandleTypeDef* adc = (ADC121S021_HandleTypeDef*)user;
    if(!adc) return;

    if(status != HAL_OK){
        adc->busy = 0;
        if(adc->cb) adc->cb(adc->user, HAL_ERROR, 0);
        return;
    }

    uint16_t raw = ((uint16_t)adc->rx_buff[0] << 8) | adc->rx_buff[1];
    adc->value = raw & 0x0FFF;
    adc->busy = 0;

    if(adc->cb) adc->cb(adc->user, HAL_OK, adc->value);
}


