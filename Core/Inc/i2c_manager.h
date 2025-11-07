#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "stm32l4xx_hal.h"


typedef void (*I2C_ReadyCallback_t)(void* arg);

typedef struct {
    I2C_HandleTypeDef *hi2c;
    volatile uint8_t busy;
    I2C_ReadyCallback_t pending_cb;
    void* pending_arg;
} I2C_Manager_t;


void I2C_Manager_Init(I2C_Manager_t* mgr, I2C_HandleTypeDef* hi2c);

uint8_t I2C_Manager_TryLock(I2C_Manager_t* mgr, I2C_ReadyCallback_t on_free, void* arg);
void I2C_Manager_Unlock(I2C_Manager_t* mgr);

#endif // I2C_MANAGER_H
