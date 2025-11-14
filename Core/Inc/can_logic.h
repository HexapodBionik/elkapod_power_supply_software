#ifndef CAN_LOGIC_H_
#define CAN_LOGIC_H_

#include "main.h"
#include "can_app.h"



void CAN_Logic_ProcessFrame(uint8_t *data, uint8_t len);
void CAN_Logic_SendAck(uint8_t opcode, uint8_t *payload, uint8_t len);

void CAN_Logic_Handle_GetServoCurrents_1_3(void);

#endif
