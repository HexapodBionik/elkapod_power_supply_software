#include "can_app.h"


HAL_StatusTypeDef CAN_App_Init(void) {
    CAN_FilterTypeDef filter;
    HAL_StatusTypeDef status;

    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    filter.FilterIdHigh     = (CAN_ID << 5);
    filter.FilterIdLow      = 0;
    filter.FilterMaskIdHigh = (0x7FF << 5);
    filter.FilterMaskIdLow  = 0;

    status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if(status != HAL_OK) return status;
    status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    if(status != HAL_OK) return status;
    return HAL_CAN_Start(&hcan1);
}


HAL_StatusTypeDef CAN_App_SendResponse(uint8_t opcode, uint8_t *payload, uint8_t len) {
    CAN_TxHeaderTypeDef tx;
    uint32_t mailbox;

    uint8_t buf[8] = {0};
    buf[0] = opcode;

    for(uint8_t i = 0; i < len; i++)
        buf[1+i] = payload[i];

    tx.StdId = CAN_ID;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = len + 1;

    return HAL_CAN_AddTxMessage(&hcan1, &tx, buf, &mailbox);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) != HAL_OK)
        return;

    CAN_Logic_ProcessFrame(data, hdr.DLC);
}
