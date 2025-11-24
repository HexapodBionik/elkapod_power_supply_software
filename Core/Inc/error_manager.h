#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include "main.h"
#include "hardware_config.h"
#include "buzzer_controller.h"
#include "servos_controller.h"
#include "pcf8574.h"

#define ERR_CONV_OVERCURRENT  	0x01
#define ERR_SERV_OVERCURRENT	0x02
#define ERR_CONV_OVERTEMP   	0x04
#define ERR_VIN_TOO_LOW     	0x08

#define ERROR_MAX_QUEUE 4

#define CONV_OC_COUNT_THRESHOLD 10
#define CONV_OC_MAX_DELAY_MS 2000
#define CONV_TOGGLE_STATE_BLOCKING_TICKS 3000
#define SERVO_OC_COUNT_THRESHOLD 10


typedef struct {
    uint8_t code;
    uint8_t conv_mask;
    uint32_t servo_mask;
    uint32_t timestamp;
} ErrorEntry;


typedef struct {
    ErrorEntry queue[ERROR_MAX_QUEUE];
    uint8_t queue_size;

    int8_t view_index_led;
	int8_t view_index_can;
	int8_t can_last_viwed;

	uint8_t error_pending_can;
	uint8_t can_delete_error_done;

	uint8_t update_state_request;

    uint8_t conv_oc_counters[CONVERTER_COUNT];
    uint32_t last_conv_oc_tick[CONVERTER_COUNT];
    uint32_t last_conv_toggle_state_tick[CONVERTER_COUNT];

    uint8_t servo_oc_counters[SERVO_COUNT];

    uint32_t servo_block_mask;
    uint8_t  conv_block_mask;

} ErrorManager;


void ErrorManager_Init(void);
void ErrorManager_Tick(void);

void ErrorManager_ConvOC_Notify(uint16_t GPIO_Pin);
void ErrorManager_CheckServoCurrents(float *servo_currents);

void ErrorManager_Push(uint8_t code, uint8_t conv_mask, uint32_t servo_mask);
void ErrorManager_DeleteCurrent(void);

uint8_t ErrorManager_LED_CurrentCode(void);
uint8_t ErrorManager_LED_CurrentConvMask(void);
uint32_t ErrorManager_LED_CurrentServoMask(void);

void ErrorManager_CAN_ViewNext(void);
void ErrorManager_CAN_DeleteLastViewed(void);

void ErrorManager_LED_ViewNext(void);
void ErrorManager_LED_DeleteCurrent(void);

uint8_t ErrorManager_BuildErrorPayload(uint8_t *buff);

void ErrorManager_RegistryConverterToggleState(uint8_t converter_number);
void ErrorManager_UpdateState(void);

void ErrorManager_ClearAll(void);

#endif //ERROR_MANAGER_H
