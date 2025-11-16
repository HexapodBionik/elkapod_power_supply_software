#ifndef CAN_APP_H_
#define CAN_APP_H_

#include "main.h"
#include "can.h"
#include "can_logic.h"


HAL_StatusTypeDef CAN_App_Init(uint16_t can_id_min, uint16_t can_id_max);
HAL_StatusTypeDef CAN_App_SendFrame(uint32_t id, uint8_t *data, uint8_t len);

#endif
