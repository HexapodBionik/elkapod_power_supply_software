#include "i2c_manager.h"


void I2C_Manager_Init(I2C_Manager_t* mgr, I2C_HandleTypeDef* hi2c) {
    mgr->hi2c = hi2c;
    mgr->busy = 0;
    mgr->pending_cb = NULL;
    mgr->pending_arg = NULL;
}

uint8_t I2C_Manager_TryLock(I2C_Manager_t* mgr, I2C_ReadyCallback_t on_free, void* arg) {
    uint8_t granted = 0;

    __disable_irq();
    if(!mgr->busy) {
        mgr->busy = 1;
        granted = 1;
    } else if(on_free) {
        mgr->pending_cb = on_free;
        mgr->pending_arg = arg;
    }
    __enable_irq();

    return granted;
}

void I2C_Manager_Unlock(I2C_Manager_t* mgr) {
    __disable_irq();
    mgr->busy = 0;

    if(mgr->pending_cb) {
        I2C_ReadyCallback_t cb = mgr->pending_cb;
        void* arg = mgr->pending_arg;
        mgr->pending_cb = NULL;
        mgr->pending_arg = NULL;

        __enable_irq();
        cb(arg);
        return;
    }
    __enable_irq();
}
