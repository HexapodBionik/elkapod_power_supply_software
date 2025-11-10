#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "stm32l4xx_hal.h"

typedef void (*I2C_StartCallback_t)(void* ctx);
typedef void (*I2C_DoneCallback_t)(void* ctx);

typedef HAL_StatusTypeDef (*I2C_TaskFunc_t)(void* ctx, I2C_DoneCallback_t done_cb, void* done_cb_ctx);

#define I2C_QUEUE_LEN 4

typedef struct {
    I2C_TaskFunc_t func;
    void* ctx;
    I2C_StartCallback_t on_start_func;
    void* on_start_ctx;
} I2C_QueuedTask;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    I2C_QueuedTask queue[I2C_QUEUE_LEN];
    uint8_t head;
    uint8_t tail;
    uint8_t busy;
} I2C_Manager;

HAL_StatusTypeDef I2C_Manager_Init(I2C_Manager* mgr, I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef I2C_Manager_TryEnqueue(I2C_Manager* mgr, I2C_TaskFunc_t func,
										 void* ctx, I2C_StartCallback_t on_start_func,
										 void* on_start_ctx);
void I2C_Manager_Release(I2C_Manager* mgr);
void I2C_Manager_Process(I2C_Manager* mgr);
HAL_StatusTypeDef I2C_Manager_LockAndTransmit_Blocking(I2C_Manager* mgr,
                                                       uint16_t address,
                                                       uint8_t* data, uint16_t size);
void I2C_Manager_FlushQueue(I2C_Manager* mgr);

I2C_Manager* I2C_Manager_FindByHi2C(I2C_HandleTypeDef* hi2c);

#endif

