#include "can_logic.h"

extern float servo_currents[18];
extern float I_manip;
extern float I_5V_pow;
extern float I_3V3_pow;
extern float I_standby;

extern float U_converter1;
extern float U_converter2;
extern float U_converter3;
extern float U_converter4;
extern float U_converter5;
extern float I_supply;
extern float U_supply;
extern float U_bat_ADC;


static void CAN_Logic_ReadServoGroup(uint8_t start_idx, uint8_t opcode_ack)
{
    uint16_t current1 = (uint16_t)servo_currents[start_idx];
    uint16_t current2 = (uint16_t)servo_currents[start_idx + 1];
    uint16_t current3 = (uint16_t)servo_currents[start_idx + 2];

    uint8_t payload[6];
    payload[0] = current1 >> 8;
    payload[1] = current1 & 0xFF;
    payload[2] = current2 >> 8;
    payload[3] = current2 & 0xFF;
    payload[4] = current3 >> 8;
    payload[5] = current3 & 0xFF;

    CAN_Logic_SendAck(opcode_ack, payload, 6);
}


void CAN_Logic_ProcessFrame(uint8_t *data, uint8_t len)
{
    if (len == 0) return;

    uint8_t opcode = data[0];

    switch(opcode) {
        case CAN_CMD_GET_SERVO_CURRENT_1_3_REQ:
        	CAN_Logic_Handle_GetCurrents_Servos_1_3();
            break;
        case CAN_CMD_GET_SERVO_CURRENT_4_6_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_4_6();
			break;
        case CAN_CMD_GET_SERVO_CURRENT_7_9_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_7_9();
			break;
        case CAN_CMD_GET_SERVO_CURRENT_10_12_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_10_12();
			break;
        case CAN_CMD_GET_SERVO_CURRENT_13_15_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_13_15();
			break;
        case CAN_CMD_GET_SERVO_CURRENT_16_18_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_16_18();
			break;

        case CAN_CMD_GET_I_MANIP_I5V_POW_I3V3_POW_REQ:
        	CAN_Logic_Handle_GetCurrents_I_MANIP_I_5V_POW_I_3V3_POW();
			break;
        case CAN_CMD_GET_I_STANDBY_I_SUPPLY_REQ:
        	CAN_Logic_Handle_GetCurrents_I_STANDBY_I_SUPPLY();
			break;

        case CAN_CMD_GET_U_CONVETERS_1_3_REQ:
        	CAN_Logic_Handle_GetVoltages_Converters_1_3();
			break;
        case CAN_CMD_GET_U_CONVETERS_4_5_REQ:
			CAN_Logic_Handle_GetVoltages_Converters_4_5();
			break;
        case CAN_CMD_GET_U_SUPPLY_U_BAT_REQ:
        	CAN_Logic_Handle_GetVoltages_U_SUPPLY_U_BAT();
			break;

        default:
            break;
    }
}


void CAN_Logic_SendAck(uint8_t opcode, uint8_t *payload, uint8_t len) {
    CAN_App_SendResponse(opcode, payload, len);
}


void CAN_Logic_Handle_GetCurrents_Servos_1_3(void) {
    CAN_Logic_ReadServoGroup(0, CAN_CMD_GET_SERVO_CURRENT_1_3_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_4_6(void) {
    CAN_Logic_ReadServoGroup(3, CAN_CMD_GET_SERVO_CURRENT_4_6_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_7_9(void) {
    CAN_Logic_ReadServoGroup(6, CAN_CMD_GET_SERVO_CURRENT_7_9_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_10_12(void) {
    CAN_Logic_ReadServoGroup(9, CAN_CMD_GET_SERVO_CURRENT_10_12_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_13_15(void) {
    CAN_Logic_ReadServoGroup(12, CAN_CMD_GET_SERVO_CURRENT_13_15_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_16_18(void) {
    CAN_Logic_ReadServoGroup(15, CAN_CMD_GET_SERVO_CURRENT_16_18_ACK);
}


void CAN_Logic_Handle_GetCurrents_I_MANIP_I_5V_POW_I_3V3_POW(void) {
	uint16_t I_manip_current;
	uint16_t I_5V_pow_current;
	uint16_t I_3V3_pow_current;

	I_manip_current = (uint16_t)I_manip;
	I_5V_pow_current = (uint16_t)I_5V_pow;
	I_3V3_pow_current = (uint16_t)I_3V3_pow;

	uint8_t payload[6];
	payload[0] = I_manip_current >> 8;
	payload[1] = I_manip_current & 0xFF;
	payload[2] = I_5V_pow_current >> 8;
	payload[3] = I_5V_pow_current & 0xFF;
	payload[4] = I_3V3_pow_current >> 8;
	payload[5] = I_3V3_pow_current & 0xFF;

	CAN_Logic_SendAck(CAN_CMD_GET_I_MANIP_I5V_POW_I3V3_POW_ACK, payload, 6);
}


void CAN_Logic_Handle_GetCurrents_I_STANDBY_I_SUPPLY(void) {
	uint16_t I_standby_current;
	uint16_t I_supply_current;

	I_standby_current = (uint16_t)I_standby;
	I_supply_current = (uint16_t)I_supply;

	uint8_t payload[4];
	payload[0] = I_standby_current >> 8;
	payload[1] = I_standby_current & 0xFF;
	payload[2] = I_supply_current >> 8;
	payload[3] = I_supply_current & 0xFF;

	CAN_Logic_SendAck(CAN_CMD_GET_I_STANDBY_I_SUPPLY_ACK, payload, 4);
}


void CAN_Logic_Handle_GetVoltages_Converters_1_3(void) {
	uint16_t converter1_voltage;
	uint16_t converter2_voltage;
	uint16_t converter3_voltage;

	converter1_voltage = (uint16_t)U_converter1;
	converter2_voltage = (uint16_t)U_converter2;
	converter3_voltage = (uint16_t)U_converter3;

	uint8_t payload[6];
	payload[0] = converter1_voltage >> 8;
	payload[1] = converter1_voltage & 0xFF;
	payload[2] = converter2_voltage >> 8;
	payload[3] = converter2_voltage & 0xFF;
	payload[4] = converter3_voltage >> 8;
	payload[5] = converter3_voltage & 0xFF;

	CAN_Logic_SendAck(CAN_CMD_GET_U_CONVETERS_1_3_ACK, payload, 6);
}


void CAN_Logic_Handle_GetVoltages_Converters_4_5(void) {
	uint16_t converter4_voltage;
	uint16_t converter5_voltage;

	converter4_voltage = (uint16_t)U_converter4;
	converter5_voltage = (uint16_t)U_converter5;

	uint8_t payload[4];
	payload[0] = converter4_voltage >> 8;
	payload[1] = converter4_voltage & 0xFF;
	payload[2] = converter5_voltage >> 8;
	payload[3] = converter5_voltage & 0xFF;

	CAN_Logic_SendAck(CAN_CMD_GET_U_CONVETERS_4_5_ACK, payload, 4);
}


void CAN_Logic_Handle_GetVoltages_U_SUPPLY_U_BAT(void) {
	uint16_t U_supply_voltage;
	uint16_t U_bat_voltage;

	U_supply_voltage = (uint16_t)U_supply;
	U_bat_voltage = (uint16_t)U_bat_ADC;

	uint8_t payload[4];
	payload[0] = U_supply_voltage >> 8;
	payload[1] = U_supply_voltage & 0xFF;
	payload[2] = U_bat_voltage >> 8;
	payload[3] = U_bat_voltage & 0xFF;

	CAN_Logic_SendAck(CAN_CMD_GET_U_SUPPLY_U_BAT_ACK, payload, 4);
}


