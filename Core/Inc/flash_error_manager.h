#ifndef FLASH_ERROR_MANAGER_H
#define FLASH_ERROR_MANAGER_H

#include "error_manager.h"

#define FLASH_ERR_PAGE_ADDR  0x0807E800
#define FLASH_ERR_PAGE_SIZE  2048


typedef struct __attribute__((packed)) {
    uint32_t size;
    uint32_t pad;
} FlashHeader;

typedef struct __attribute__((packed)) {
    uint8_t code;
    uint8_t conv_mask;
    uint32_t servo_mask;
    uint8_t crc;
    uint8_t pad;  // padding to 8B
} FlashErrorEntry;


uint8_t crc8_XOR(const uint8_t* data, uint8_t len);

void ErrorManager_FlashLoad(void);
uint8_t ErrorManager_FlashNeedsUpdate(void);
void ErrorManager_FlashSave(void);


#endif //FLASH_ERROR_MANAGER_H
