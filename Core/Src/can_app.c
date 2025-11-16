#include "can_app.h"


static inline uint16_t msb_diff_pos(uint16_t x) {
    if (x == 0) return 11; // all bits the same
    uint16_t pos = 0;
    while (x >>= 1) pos++;
    return pos;
}


HAL_StatusTypeDef CAN_App_Init(uint16_t can_id_min, uint16_t can_id_max) {
    CAN_FilterTypeDef filter;
    HAL_StatusTypeDef status;

    uint16_t diff = can_id_min ^ can_id_max;
	uint16_t mask;

	if (diff == 0) {
		mask = 0x7FF;
	} else {
		uint16_t msb = msb_diff_pos(diff);
		mask = (~((1 << (msb + 1)) - 1)) & 0x7FF;
	}

	uint16_t id = can_id_min & mask;

    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    filter.FilterIdHigh   = (id & 0x7FF) << 5;
	filter.FilterIdLow    = 0;
	filter.FilterMaskIdHigh = (mask & 0x7FF) << 5;
	filter.FilterMaskIdLow  = 0;

    status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if(status != HAL_OK) return status;
    status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    if(status != HAL_OK) return status;
    return HAL_CAN_Start(&hcan1);
}


HAL_StatusTypeDef CAN_App_SendFrame(uint32_t id, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef txHeader = {0};
    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = len;

    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &mailbox);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
        return;

    CAN_Logic_HandleFrame(rxHeader.StdId, data, rxHeader.DLC);
}
