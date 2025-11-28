#include "flash_error_manager.h"
#include <string.h>

extern ErrorManager errm;


uint8_t crc8_XOR(const uint8_t* data, uint8_t len) {
    uint8_t c = 0;
    for(uint8_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}


static inline uint32_t flash_page_from_addr(uint32_t addr) {
    return (addr - FLASH_BASE) / FLASH_ERR_PAGE_SIZE;
}


static void flash_clear_error_flags(void) {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
}


static int flash_wait_ready(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
        if ((HAL_GetTick() - start) > timeout_ms) return 0;
    }
    return 1;
}


static void flash_disable_caches(void) {
	#if (__DCACHE_PRESENT == 1U)
		SCB_DisableDCache();
	#endif
	#if (__ICACHE_PRESENT == 1U)
		SCB_DisableICache();
	#endif
}


static void flash_enable_caches(void) {
	#if (__ICACHE_PRESENT == 1U)
		SCB_EnableICache();
	#endif
	#if (__DCACHE_PRESENT == 1U)
		SCB_EnableDCache();
	#endif
}



static int flash_program_doubleword_checked(uint32_t address, uint64_t data) {
    if((address & 0x7) != 0) {
        return -1;
    }

    flash_clear_error_flags();

    if(!flash_wait_ready(500)) {
        return -2;
    }

    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);

    if(status != HAL_OK) {
        uint32_t err = HAL_FLASH_GetError();
        return (int)err;
    }

    if(!flash_wait_ready(500)) {
        return -3;
    }

    return 0;
}


static int flash_erase_page_checked(uint32_t page_index) {
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;
    uint32_t address = FLASH_BASE + page_index * FLASH_ERR_PAGE_SIZE;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = page_index;
    erase.NbPages = 1;

    flash_clear_error_flags();
    if(!flash_wait_ready(500)) {
    	return -1;
    }

    if(HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        uint32_t err = HAL_FLASH_GetError();
        return (int)err;
    }

    if(!flash_wait_ready(500)) {
    	return -2; //
    }

    volatile uint64_t *p_data = (volatile uint64_t *)address;

    if (*p_data != 0xFFFFFFFFFFFFFFFFULL) {
        return -10;
    }

    return 0;
}


void ErrorManager_FlashLoad(void) {
    FlashHeader *hdr = (FlashHeader*)FLASH_ERR_PAGE_ADDR;

    if(hdr->size > ERROR_MAX_QUEUE) {
        errm.queue_size = 0;
        return;
    }

    FlashErrorEntry *flash_entries = (FlashErrorEntry*)(FLASH_ERR_PAGE_ADDR + 8);

    for(uint8_t i = 0; i < hdr->size; ++i) {
        FlashErrorEntry fe = flash_entries[i];

        uint8_t crc = crc8_XOR((uint8_t*)&fe, 6);

        if(crc != fe.crc) {
            errm.queue_size = 0;
            return;
        }

        errm.queue[i].code       = fe.code;
        errm.queue[i].conv_mask  = fe.conv_mask;
        errm.queue[i].servo_mask = fe.servo_mask;
        errm.queue[i].timestamp  = HAL_GetTick();
    }

    errm.queue_size = hdr->size;


    if(errm.queue_size > 0) {
        errm.view_index_led = 0;
        errm.view_index_can = 0;
    } else {
        errm.view_index_led = -1;
        errm.view_index_can = -1;
    }
}


uint8_t ErrorManager_FlashNeedsUpdate(void) {
	FlashHeader *hdr = (FlashHeader*)FLASH_ERR_PAGE_ADDR;

	if(hdr->size != errm.queue_size) {
		return 1;
	}

    FlashErrorEntry *flash_entries = (FlashErrorEntry*)(FLASH_ERR_PAGE_ADDR + 8);

    for(uint8_t i = 0; i < errm.queue_size; i++) {

        FlashErrorEntry expected;

        expected.code = errm.queue[i].code;
        expected.conv_mask = errm.queue[i].conv_mask;
        expected.servo_mask = errm.queue[i].servo_mask;
        expected.crc = crc8_XOR((uint8_t*)&expected, 6);

        if(memcmp(&expected, &flash_entries[i], sizeof(FlashErrorEntry)) != 0)
            return 1;
    }

    return 0;
}


void ErrorManager_FlashSave(void) {
    if(!ErrorManager_FlashNeedsUpdate())
        return;

    FlashHeader hdr;
    hdr.size = (uint32_t)errm.queue_size;
    hdr.pad  = 0xFFFFFFFFUL;

    FlashErrorEntry entries[ERROR_MAX_QUEUE];
    for(uint8_t i = 0; i < errm.queue_size; ++i) {
        entries[i].code = errm.queue[i].code;
        entries[i].conv_mask = errm.queue[i].conv_mask;
        entries[i].servo_mask = errm.queue[i].servo_mask;
        entries[i].crc = crc8_XOR((const uint8_t*)&entries[i], 1 + 1 + 4);
        entries[i].pad = 0xFF;
    }

    flash_disable_caches();

    HAL_FLASH_Unlock();

    uint32_t page_index = flash_page_from_addr(FLASH_ERR_PAGE_ADDR);
    int rc = flash_erase_page_checked(page_index);
    if(rc != 0) {
        HAL_FLASH_Lock();
        flash_enable_caches();
        return;
    }

    uint64_t packed = 0;
    memcpy(&packed, &hdr, sizeof(hdr));
    rc = flash_program_doubleword_checked(FLASH_ERR_PAGE_ADDR, packed);
    if(rc != 0) {
        HAL_FLASH_Lock();
        flash_enable_caches();
        return;
    }

    uint32_t base = FLASH_ERR_PAGE_ADDR + 8;
    for(uint8_t i = 0; i < errm.queue_size; ++i) {
        uint64_t p = 0;
        memcpy(&p, &entries[i], sizeof(entries[i]));
        rc = flash_program_doubleword_checked(base + (i * 8), p);
        if(rc != 0) {
            HAL_FLASH_Lock();
            flash_enable_caches();
            return;
        }
    }

    HAL_FLASH_Lock();
    flash_enable_caches();
}



