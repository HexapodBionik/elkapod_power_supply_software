#include "can_logic.h"

extern float servo_currents[18];


void CAN_Logic_ProcessFrame(uint8_t *data, uint8_t len)
{
    if (len == 0) return;

    uint8_t opcode = data[0];

    switch(opcode) {
        case CAN_CMD_GET_SERVO_CURRENT_1_3_REQ:
        	CAN_Logic_Handle_GetServoCurrents_1_3();
            break;

        default:
            break;
    }
}


void CAN_Logic_SendAck(uint8_t opcode, uint8_t *payload, uint8_t len) {
    CAN_App_SendResponse(opcode, payload, len);
}


void CAN_Logic_Handle_GetServoCurrents_1_3(void) {
    uint16_t servo1_current;
    uint16_t servo2_current;
    uint16_t servo3_current;

    servo1_current = (uint16_t)servo_currents[0];
    servo2_current = (uint16_t)servo_currents[1];
    servo3_current = (uint16_t)servo_currents[2];

    uint8_t payload[6];
    payload[0] = servo1_current >> 8;
    payload[1] = servo1_current & 0xFF;
    payload[2] = servo2_current >> 8;
    payload[3] = servo2_current & 0xFF;
    payload[4] = servo3_current >> 8;
    payload[5] = servo3_current & 0xFF;

    CAN_Logic_SendAck(CAN_CMD_GET_SERVO_CURRENT_1_3_ACK, payload, 6);
}
