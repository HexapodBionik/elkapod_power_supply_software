#include "i2c_manager.h"

#ifndef I2C_MANAGER_MAX
#define I2C_MANAGER_MAX 2
#endif

static I2C_Manager* i2c_manager_registry[I2C_MANAGER_MAX] = {0};


static void I2C_Manager_Register(I2C_Manager* mgr) {
    for (uint8_t i = 0; i < I2C_MANAGER_MAX; i++) {
        if (i2c_manager_registry[i] == NULL) {
            i2c_manager_registry[i] = mgr;
            return;
        }
    }
}


I2C_Manager* I2C_Manager_FindByHi2C(I2C_HandleTypeDef* hi2c) {
    for (uint8_t i = 0; i < I2C_MANAGER_MAX; i++) {
        if (i2c_manager_registry[i] && i2c_manager_registry[i]->hi2c == hi2c)
            return i2c_manager_registry[i];
    }
    return NULL;
}


HAL_StatusTypeDef I2C_Manager_Init(I2C_Manager* mgr, I2C_HandleTypeDef* hi2c) {
	if(!hi2c) return HAL_ERROR;
    mgr->hi2c = hi2c;
    mgr->head = 0;
    mgr->tail = 0;
    mgr->busy = 0;
    I2C_Manager_Register(mgr);
    return HAL_OK;
}


static uint8_t queue_is_empty(I2C_Manager* mgr) {
    return mgr->head == mgr->tail;
}


static uint8_t queue_is_full(I2C_Manager* mgr) {
    return ((mgr->head + 1) % I2C_QUEUE_LEN) == mgr->tail;
}


HAL_StatusTypeDef I2C_Manager_TryEnqueue(I2C_Manager* mgr, I2C_TaskFunc_t func,
										 void* ctx, I2C_StartCallback_t on_start_func,
										 void* on_start_ctx) {
	if(!mgr || !func) return HAL_ERROR;

	    __disable_irq();
	    uint8_t busy = mgr->busy;
	    __enable_irq();

	    if(!busy && HAL_I2C_GetState(mgr->hi2c) == HAL_I2C_STATE_READY) {
	        __disable_irq();
	        mgr->busy = 1;
	        __enable_irq();

	        if(on_start_func) on_start_func(on_start_ctx);
	        HAL_StatusTypeDef status = func(ctx, (I2C_DoneCallback_t)I2C_Manager_Release, mgr);
	        if(status != HAL_OK) {
	            __disable_irq();
	            mgr->busy = 0;
	            __enable_irq();
	        }
	        return status;
	    }

	    if(queue_is_full(mgr)) return HAL_BUSY;

	    __disable_irq();
	    mgr->queue[mgr->head].func = func;
	    mgr->queue[mgr->head].ctx = ctx;
	    mgr->queue[mgr->head].on_start_func = on_start_func;
	    mgr->queue[mgr->head].on_start_ctx = on_start_ctx;
	    mgr->head = (mgr->head + 1) % I2C_QUEUE_LEN;
	    __enable_irq();

	    return HAL_BUSY;
}


void I2C_Manager_Release(I2C_Manager* mgr) {
	__disable_irq();
	mgr->busy = 0;
	__enable_irq();
	I2C_Manager_Process(mgr);
}


void I2C_Manager_Process(I2C_Manager* mgr) {
	if(queue_is_empty(mgr) || mgr->busy) return;

	__disable_irq();
	I2C_QueuedTask task = mgr->queue[mgr->tail];
	mgr->tail = (mgr->tail + 1) % I2C_QUEUE_LEN;
	mgr->busy = 1;
	__enable_irq();

	if(task.on_start_func) task.on_start_func(task.on_start_ctx);
	task.func(task.ctx, (I2C_DoneCallback_t)I2C_Manager_Release, mgr);
}


HAL_StatusTypeDef I2C_Manager_LockAndTransmit_Blocking(I2C_Manager* mgr,
                                                       uint16_t address,
                                                       uint8_t* data, uint16_t size) {
    while (!queue_is_empty(mgr) || mgr->busy == 1 || HAL_I2C_GetState(mgr->hi2c) != HAL_I2C_STATE_READY);

    __disable_irq();
    mgr->busy = 1;
    __enable_irq();

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(mgr->hi2c, address, data, size);

    while (HAL_I2C_GetState(mgr->hi2c) != HAL_I2C_STATE_READY);

    __disable_irq();
    mgr->busy = 0;
    __enable_irq();

    return status;
}


HAL_StatusTypeDef I2C_Manager_LockAndReceive_Blocking(I2C_Manager* mgr,
													  uint16_t address,
                                                      uint8_t* data, uint16_t size) {
    while (!queue_is_empty(mgr) || mgr->busy == 1 || HAL_I2C_GetState(mgr->hi2c) != HAL_I2C_STATE_READY);

    __disable_irq();
    mgr->busy = 1;
    __enable_irq();

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive_DMA(mgr->hi2c, address, data, size);


    while (HAL_I2C_GetState(mgr->hi2c) != HAL_I2C_STATE_READY);

    __disable_irq();
    mgr->busy = 0;
    __enable_irq();

    return status;
}


HAL_StatusTypeDef I2C_Manager_LockAndMemRead_Blocking(I2C_Manager* mgr,
													  uint16_t address,
                                                      uint16_t mem_addr, uint16_t mem_addr_size,
                                                      uint8_t* data, uint16_t size) {
    while (!queue_is_empty(mgr) || mgr->busy == 1 || HAL_I2C_GetState(mgr->hi2c) != HAL_I2C_STATE_READY);

    __disable_irq();
    mgr->busy = 1;
    __enable_irq();

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(mgr->hi2c, address,
                                                    mem_addr, mem_addr_size,
                                                    data, size);

    while (mgr->busy == 1);

    __disable_irq();
    mgr->busy = 0;
    __enable_irq();

    return status;
}


void I2C_Manager_FlushQueue(I2C_Manager* mgr) {
    __disable_irq();
    mgr->head = 0;
    mgr->tail = 0;
    mgr->busy = 0;
    __enable_irq();
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    I2C_Manager* mgr = I2C_Manager_FindByHi2C(hi2c);
    if (mgr) I2C_Manager_Release(mgr);
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    I2C_Manager* mgr = I2C_Manager_FindByHi2C(hi2c);
    if (mgr) I2C_Manager_Release(mgr);
}
