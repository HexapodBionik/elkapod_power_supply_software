#include "pcf8574.h"

static inline HAL_StatusTypeDef i2c_dma_write(PCF8574_HandleTypeDef* pcf){
    while(HAL_I2C_GetState(pcf->hi2c) != HAL_I2C_STATE_READY);
    return HAL_I2C_Master_Transmit_DMA(pcf->hi2c, pcf->addr, &pcf->write_buff, sizeof(pcf->write_buff));
}

inline void i2c_dma_read(PCF8574_HandleTypeDef* pcf){
    while(HAL_I2C_GetState(pcf->hi2c) != HAL_I2C_STATE_READY);

    HAL_I2C_Master_Receive_DMA(pcf->hi2c, pcf->addr + 0x01, &pcf->read_buff, sizeof(pcf->write_buff));
}

HAL_StatusTypeDef PCF7485_init(PCF8574_HandleTypeDef* pcf, I2C_HandleTypeDef* hi2c, uint8_t addr){
    // Init PCF7485_HandleTypeDef struct
    pcf->hi2c = hi2c;
    pcf->addr = addr;
    pcf->write_buff = 0b11111111;
    pcf->read_buff = 0x00;

    // Check if the device is available
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(pcf->hi2c, pcf->addr << 1, 5, 100);
    if(status != HAL_OK){
        return status;
    }

    // Set low level to all expander's gpios
//    status = i2c_dma_write(pcf);
    return HAL_OK;
}

void PCF7485_write_pin(PCF8574_HandleTypeDef* pcf, uint8_t pin, GPIO_PinState state){
    if(state == GPIO_PIN_SET){
        pcf->write_buff |= (0x01 << pin);
    }
    else{
        pcf->write_buff &= ~(0x01 << pin);
    }
    i2c_dma_write(pcf);
}

void PCF7485_write_buffer(PCF8574_HandleTypeDef* pcf, uint8_t write_buffer){
    pcf->write_buff = write_buffer;
    i2c_dma_write(pcf);
}

GPIO_PinState PCF7485_read_pin(PCF8574_HandleTypeDef* pcf, uint8_t pin){
    i2c_dma_read(pcf);

    if(pcf->read_buff & (0x01 << pin)){
        return GPIO_PIN_SET;
    }
    return GPIO_PIN_RESET;
}
