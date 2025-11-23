#include "error_manager.h"
#include <string.h>


extern ErrorManager errm;

static const uint8_t servo_to_conv[SERVO_COUNT] = {
    0,0,0,0,0,0,   // servo 1..6 -> conv1
    1,1,1,1,1,1,   // servo 7..12 -> conv2
    2,2,2,2,2,2    // servo 13..18 -> conv3
};

static const uint16_t CONV_OC_Pin[CONVERTER_COUNT] = {
	CONV1_OC_Pin,
	CONV2_OC_Pin,
	CONV3_OC_Pin,
	CONV4_OC_Pin,
	CONV5_OC_Pin
};


void ErrorManager_Init(void) {
	memset(&errm, 0, sizeof(errm));
	errm.view_index_led = -1;
	errm.view_index_can = -1;
	errm.error_pending_can = 0;
}


void ErrorManager_ConvOC_Notify(uint16_t GPIO_Pin) {
    uint32_t now = HAL_GetTick();

    for(uint8_t i = 0; i < 5; i++) {
        if(GPIO_Pin == CONV_OC_Pin[i]) {
            if(now - errm.last_conv_oc_tick[i] < CONV_OC_MAX_DELAY_MS) {
                if(errm.conv_oc_counters[i] < 255)
                    errm.conv_oc_counters[i]++;
            } else {
                errm.conv_oc_counters[i] = 1;
            }
            errm.last_conv_oc_tick[i] = now;
        }
    }
}


void ErrorManager_CheckServoCurrents(float *servo_currents) {
    for(uint8_t i = 0; i < SERVO_COUNT; i++) {
        if(servo_currents[i] > I_SERVO_MAX) {
            if(errm.servo_oc_counters[i] < 255)
                errm.servo_oc_counters[i]++;
        } else {
            errm.servo_oc_counters[i] = 0;
        }

        if(errm.servo_oc_counters[i] >= SERVO_OC_COUNT_THRESHOLD) {
			uint8_t conv_mask = 1 << servo_to_conv[i];
			uint32_t servo_mask = (1u << i);
			ErrorManager_Push(ERR_SERV_OVERCURRENT, conv_mask, servo_mask);
        }
    }
}


void ErrorManager_Push(uint8_t code, uint8_t conv_mask, uint32_t servo_mask) {
    uint32_t now = HAL_GetTick();

    Buzzer_Start(2, BUZZ_HIGH_LONG);

    for(uint8_t i = 0; i < errm.queue_size; i++) {
        if(errm.queue[i].code == code) {
        	uint8_t new_conv_mask = errm.queue[i].conv_mask | conv_mask;
			uint32_t new_servo_mask = errm.queue[i].servo_mask | servo_mask;

        	if(errm.queue[i].conv_mask != new_conv_mask || errm.queue[i].servo_mask != new_servo_mask) {
        		errm.error_pending_can = 1;
        	}

            errm.queue[i].conv_mask = new_conv_mask;
            errm.queue[i].servo_mask = new_servo_mask;

            errm.queue[i].timestamp = now;

            ErrorEntry temp = errm.queue[i];
            for(int j = i; j > 0; j--) {
                errm.queue[j] = errm.queue[j-1];
            }
            errm.queue[0] = temp;

            errm.view_index_led = 0;
            errm.view_index_can = 0;

            return;
        }
    }

    if(errm.queue_size < ERROR_MAX_QUEUE) {
        for(uint8_t i = errm.queue_size; i > 0; i--) {
            errm.queue[i] = errm.queue[i-1];
        }
        errm.queue_size++;
    } else {
        for(uint8_t i = ERROR_MAX_QUEUE - 1; i > 0; i--) {
            errm.queue[i] = errm.queue[i-1];
        }
    }

    errm.queue[0].code = code;
    errm.queue[0].conv_mask = conv_mask;
    errm.queue[0].servo_mask = servo_mask;
    errm.queue[0].timestamp = now;

    errm.view_index_led = 0;
    errm.view_index_can = 0;

    errm.error_pending_can = 1;
}


static void ErrorManager_DeleteAtIndex(int8_t idx) {
    if(idx < 0 || idx >= errm.queue_size) return;

    for(uint8_t i = idx; i < errm.queue_size - 1; i++) {
        errm.queue[i] = errm.queue[i+1];
    }
    errm.queue_size--;

    if(errm.queue_size == 0) {
        errm.view_index_led = -1;
        errm.view_index_can = -1;
    } else {
        if(errm.view_index_led >= errm.queue_size) {
            errm.view_index_led = errm.queue_size - 1;
        }

        if(errm.view_index_can >= errm.queue_size) {
            errm.view_index_can = errm.queue_size - 1;
        }
    }

    Buzzer_Start(1, BUZZ_LOW_LONG);
}


void ErrorManager_LED_DeleteCurrent(void) {
    ErrorManager_DeleteAtIndex(errm.view_index_led);
}


void ErrorManager_CAN_DeleteLastViewed(void) {
	if(errm.can_delete_error_done != 0 || errm.queue_size == 0) {
		return;
	}
    ErrorManager_DeleteAtIndex(errm.can_last_viwed);
    errm.can_delete_error_done = 1;
}


uint8_t ErrorManager_LED_CurrentCode(void) {
    if(errm.view_index_led < 0 || errm.view_index_led >= errm.queue_size)
        return 0;
    return errm.queue[errm.view_index_led].code;
}

uint8_t ErrorManager_LED_CurrentConvMask(void) {
    if(errm.view_index_led < 0 || errm.view_index_led >= errm.queue_size)
        return 0;
    return errm.queue[errm.view_index_led].conv_mask;
}

uint32_t ErrorManager_LED_CurrentServoMask(void) {
    if(errm.view_index_led < 0 || errm.view_index_led >= errm.queue_size)
        return 0;
    return errm.queue[errm.view_index_led].servo_mask;
}


void ErrorManager_LED_ViewNext(void) {
    if(errm.queue_size == 0) return;
    if(errm.view_index_led < errm.queue_size - 1) {
        errm.view_index_led++;
    } else if(errm.view_index_led == errm.queue_size - 1) {
    	errm.view_index_led = 0;
    }
}


void ErrorManager_CAN_ViewNext(void) {
    if(errm.queue_size == 0) return;
    if(errm.view_index_can < errm.queue_size - 1) {
        errm.view_index_can++;
    } else if(errm.view_index_can == errm.queue_size - 1) {
    	errm.view_index_can = 0;
    }
}


void ErrorManager_Tick(void) {
    uint32_t now = HAL_GetTick();

    // Converters
    for(uint8_t i = 0; i < CONVERTER_COUNT; i++) {
        if(errm.conv_oc_counters[i] >= CONV_OC_COUNT_THRESHOLD) {
			ErrorManager_Push(ERR_CONV_OVERCURRENT, (1u << i), 0);
        }

        if(now - errm.last_conv_oc_tick[i] > CONV_OC_MAX_DELAY_MS)
            errm.conv_oc_counters[i] = 0;
    }
}


uint8_t ErrorManager_BuildErrorPayload(uint8_t *buff) {
    if(errm.queue_size == 0 || errm.view_index_can < 0)
        return 0;

    ErrorEntry *error = &errm.queue[errm.view_index_can];

    errm.can_delete_error_done = 0;
    errm.can_last_viwed = errm.view_index_can;
    ErrorManager_CAN_ViewNext();

    buff[0] = error->code;

    switch(error->code) {
        case ERR_CONV_OVERCURRENT:
            buff[1] = error->conv_mask;
            return 2;

        case ERR_SERV_OVERCURRENT:
            buff[1] = (error->servo_mask >> 16) & 0xFF;
            buff[2] = (error->servo_mask >> 8) & 0xFF;
            buff[3] = error->servo_mask & 0xFF;
            return 4;

        case ERR_CONV_OVERTEMP:
        	return 1;

        case ERR_VIN_TOO_LOW:
        	return 1;

        default:
            return 1;
    }
}


void ErrorManager_ClearAll(void) {
    ErrorManager_Init();
}
