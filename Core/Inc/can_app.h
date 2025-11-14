#ifndef CAN_APP_H_
#define CAN_APP_H_

#include "main.h"
#include "can.h"
#include "can_logic.h"


HAL_StatusTypeDef CAN_App_Init(void);
HAL_StatusTypeDef CAN_App_SendResponse(uint8_t opcode, uint8_t *payload, uint8_t len);

#endif
