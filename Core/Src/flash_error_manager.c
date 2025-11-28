#include "flash_error_manager.h"
#include <string.h>

extern ErrorManager errm;


uint8_t crc8_XOR(const uint8_t* data, uint8_t len) {
    uint8_t c = 0;
    for(uint8_t i = 0; i < len; i++)
        c ^= data[i];
    return c;
}


void ErrorManager_FlashLoad(void) {
	FlashHeader *hdr = (FlashHeader*)FLASH_ERR_PAGE_ADDR;

	if(hdr->size > ERROR_MAX_QUEUE) {
		errm.queue_size = 0;
		return;
	}

	FlashErrorEntry *flash_entries = (FlashErrorEntry*)(FLASH_ERR_PAGE_ADDR + 8);

	for(uint8_t i = 0; i < hdr->size; i++) {

		FlashErrorEntry e = flash_entries[i];

		uint8_t crc = crc8_XOR((uint8_t*)&e, 6);

		if(crc != e.crc) {
			errm.queue_size = 0;
			return;
		}

		errm.queue[i].code       = e.code;
		errm.queue[i].conv_mask  = e.conv_mask;
		errm.queue[i].servo_mask = e.servo_mask;
		errm.queue[i].timestamp  = HAL_GetTick();
	}

	errm.queue_size = hdr->size;
	errm.view_index_led = 0;
	errm.view_index_can = 0;
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


static void Flash_WriteHeader(uint8_t qsize) {
    FlashHeader hdr;
    hdr.size = (uint32_t)qsize;
    hdr.pad  = 0xFFFFFFFF;

    uint64_t packed = 0;
    memcpy(&packed, &hdr, sizeof(hdr));

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                      FLASH_ERR_PAGE_ADDR,
                      packed);
}



void ErrorManager_FlashSave(void) {
    if(!ErrorManager_FlashNeedsUpdate())
        return;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = (FLASH_ERR_PAGE_ADDR - FLASH_BASE) / FLASH_ERR_PAGE_SIZE;
    erase.NbPages = 1;

    HAL_FLASHEx_Erase(&erase, &page_error);

    Flash_WriteHeader(errm.queue_size);

    uint32_t base = FLASH_ERR_PAGE_ADDR + 8;
	uint64_t buf;

	for(uint8_t i = 0; i < errm.queue_size; i++) {

		FlashErrorEntry e;

		e.code       = errm.queue[i].code;
		e.conv_mask  = errm.queue[i].conv_mask;
		e.servo_mask = errm.queue[i].servo_mask;
		e.crc        = crc8_XOR((uint8_t*)&e, 6);
		e.pad        = 0xFF;

		memcpy(&buf, &e, sizeof(e));

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
			base + (i * 8),
			buf);
	}

    HAL_FLASH_Lock();
}



