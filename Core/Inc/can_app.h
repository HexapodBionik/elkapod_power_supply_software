#ifndef CAN_APP_H_
#define CAN_APP_H_

#include "main.h"
#include "can.h"

#define CAN_RX_BUF_SIZE 8  //max 8B

extern uint8_t CAN_LastRxData[CAN_RX_BUF_SIZE];
extern uint8_t CAN_LastRxLen;
extern uint32_t CAN_LastRxID;
extern CAN_RxHeaderTypeDef CAN_LastRxHeader;

void CAN_App_Init(void);
void CAN_App_SendTest(void);

#endif
