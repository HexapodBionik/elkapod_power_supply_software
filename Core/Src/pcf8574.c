#include "pcf8574.h"
#include "main.h"

#ifndef PCF_MAX_INSTANCES
#define PCF_MAX_INSTANCES 8
#endif

static PCF8574_HandleTypeDef* pcf_registry[PCF_MAX_INSTANCES] = {0};


static int8_t register_pcf(PCF8574_HandleTypeDef* pcf){
    for(uint8_t i = 0; i < PCF_MAX_INSTANCES; i++){
        if(pcf_registry[i] == NULL){
            pcf_registry[i] = pcf;
            return 0;
        }
    }
    return -1;
}


static PCF8574_HandleTypeDef* find_busy_pcf_by_i2c(I2C_HandleTypeDef* hi2c){
    for(uint8_t i = 0; i < PCF_MAX_INSTANCES; i++){
        PCF8574_HandleTypeDef* pcf = pcf_registry[i];
        if(pcf && pcf->hi2c == hi2c && pcf->tx_busy){
            return pcf;
        }
    }
    return NULL;
}


HAL_StatusTypeDef PCF7485_init(PCF8574_HandleTypeDef* pcf, I2C_HandleTypeDef* hi2c, uint8_t addr){
	if(!pcf || !hi2c) return HAL_ERROR;
    pcf->hi2c = hi2c;
    pcf->addr = addr;
    pcf->write_buff = 0xFF;
    pcf->read_buff = 0xFF;

    pcf->tx_busy = 0;
	pcf->tx_cb = NULL;
	pcf->tx_user = NULL;

	if(register_pcf(pcf) != 0) return HAL_ERROR;
    return HAL_I2C_IsDeviceReady(pcf->hi2c, pcf->addr, 5, 100);

}


HAL_StatusTypeDef PCF7485_write_buffer_blocking(PCF8574_HandleTypeDef* pcf, uint8_t data){
	if(!pcf) return HAL_ERROR;
    pcf->write_buff = data;
    while(HAL_I2C_GetState(pcf->hi2c) != HAL_I2C_STATE_READY);
	return HAL_I2C_Master_Transmit_DMA(pcf->hi2c, pcf->addr, &pcf->write_buff, sizeof(pcf->write_buff));
}


HAL_StatusTypeDef PCF7485_write_pin_blocking(PCF8574_HandleTypeDef* pcf, uint8_t pin, GPIO_PinState state){
	if(!pcf) return HAL_ERROR;
    if(state == GPIO_PIN_SET){
        pcf->write_buff |= (0x01 << pin);
    }
    else{
        pcf->write_buff &= ~(0x01 << pin);
    }
    while(HAL_I2C_GetState(pcf->hi2c) != HAL_I2C_STATE_READY);
	return HAL_I2C_Master_Transmit_DMA(pcf->hi2c, pcf->addr, &pcf->write_buff, sizeof(pcf->write_buff));
}


GPIO_PinState PCF7485_read_pin_blocking(PCF8574_HandleTypeDef* pcf, uint8_t pin){
	while(HAL_I2C_GetState(pcf->hi2c) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Receive_DMA(pcf->hi2c, pcf->addr + 0x01, &pcf->read_buff, sizeof(pcf->write_buff));

	while(HAL_I2C_GetState(pcf->hi2c) != HAL_I2C_STATE_READY);
    if(pcf->read_buff & (0x01 << pin)){
        return GPIO_PIN_SET;
    }
    return GPIO_PIN_RESET;
}


HAL_StatusTypeDef PCF8574_write_buffer_async(PCF8574_HandleTypeDef* pcf, uint8_t data,
                                             PCF8574_Callback_t cb, void* user){
    if(!pcf) return HAL_ERROR;
    if(HAL_I2C_GetState(pcf->hi2c) != HAL_I2C_STATE_READY){
        return HAL_BUSY;
    }

    __disable_irq();
    pcf->write_buff = data;
    pcf->tx_busy = 1;
    pcf->tx_cb = cb;
    pcf->tx_user = user;
    __enable_irq();

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(pcf->hi2c, pcf->addr, &pcf->write_buff, 1);
    if(status != HAL_OK){
        __disable_irq();
        pcf->tx_busy = 0;
        pcf->tx_cb = NULL;
        pcf->tx_user = NULL;
        __enable_irq();
    }
    return status;
}


HAL_StatusTypeDef PCF8574_write_pin_async(PCF8574_HandleTypeDef* pcf, uint8_t pin, GPIO_PinState state,
                                          PCF8574_Callback_t cb, void* user){
    if(!pcf) return HAL_ERROR;
    __disable_irq();
    if(state == GPIO_PIN_SET) {
    	pcf->write_buff |= (0x01 << pin);
    } else {
    	pcf->write_buff &= ~(0x01 << pin);
    }
    __enable_irq();

    return PCF8574_write_buffer_async(pcf, pcf->write_buff, cb, user);
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    PCF8574_HandleTypeDef* pcf = find_busy_pcf_by_i2c(hi2c);
    if(pcf){
        PCF8574_Callback_t cb;
        void* user;
        __disable_irq();
        pcf->tx_busy = 0;
        cb = pcf->tx_cb;
        user = pcf->tx_user;
        pcf->tx_cb = NULL;
        pcf->tx_user = NULL;
        __enable_irq();

        if(cb) cb(pcf->addr, HAL_OK, user);
    }
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
    PCF8574_HandleTypeDef* pcf = find_busy_pcf_by_i2c(hi2c);
    if(pcf){
        PCF8574_Callback_t cb;
        void* user;
        __disable_irq();
        pcf->tx_busy = 0;
        cb = pcf->tx_cb;
        user = pcf->tx_user;
        pcf->tx_cb = NULL;
        pcf->tx_user = NULL;
        __enable_irq();

        if(cb) cb(pcf->addr, HAL_ERROR, user);
    }
}
