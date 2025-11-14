#include "can_app.h"

uint8_t CAN_LastRxData[CAN_RX_BUF_SIZE];
uint8_t CAN_LastRxLen = 0;
uint32_t CAN_LastRxID = 0;
CAN_RxHeaderTypeDef CAN_LastRxHeader;


void CAN_App_Init(void) {
    CAN_FilterTypeDef filter;

    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    // FILTR: akceptujemy tylko ID = 100
    // dla maski: id = 100, maska = 0x7FF (pełne sprawdzanie)
    filter.FilterIdHigh     = (100 << 5);
    filter.FilterIdLow      = 0;
    filter.FilterMaskIdHigh = (0x7FF << 5);
    filter.FilterMaskIdLow  = 0;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }

    HAL_CAN_ActivateNotification(&hcan1,
                                 CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan1);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    CAN_LastRxID = rxHeader.StdId;
    CAN_LastRxHeader = rxHeader;
    CAN_LastRxLen = rxHeader.DLC;

    for (uint8_t i = 0; i < CAN_LastRxLen; i++)
        CAN_LastRxData[i] = rxData[i];

    __NOP();
}


void CAN_App_SendTest(void)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t TxMailbox;
    uint8_t data[1] = {0x55};

    txHeader.StdId = 100;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 1;

    HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &TxMailbox);
}
