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

extern float temperatures[3];

extern PotChannel pots[4];
extern ServoControllerState servos;
extern VoltageOutputsController vouts;
extern uint8_t manip_conv_en;
extern uint8_t manip_conv_en_done;

extern uint8_t turn_off_supply_request;
extern volatile uint8_t pressed_off_button;

extern BuzzerController buzzer;
extern ErrorManager errm;


const uint16_t CONV1_3_MAX_VOLTAGE =
		CALC_CONVERTER_VOLTAGE(CONV1_3_RFB1, CONV1_3_RFB2, CONV1_3_RFB3, POT_RESISTANCE_OFFSET);

const uint16_t CONV1_3_MIN_VOLTAGE =
		CALC_CONVERTER_VOLTAGE(CONV1_3_RFB1, CONV1_3_RFB2, CONV1_3_RFB3, POT_RESISTANCE + POT_RESISTANCE_OFFSET);

const uint16_t CONV4_MAX_VOLTAGE =
		CALC_CONVERTER_VOLTAGE(CONV4_RFB1, CONV4_RFB2, CONV4_RFB3, POT_RESISTANCE_OFFSET);

const uint16_t CONV4_MIN_VOLTAGE =
		CALC_CONVERTER_VOLTAGE(CONV4_RFB1, CONV4_RFB2, CONV4_RFB3, POT_RESISTANCE + POT_RESISTANCE_OFFSET);


extern uint16_t converter_voltage_to_pot_value(float voltage, float R_FB1,
		float R_FB2, float R_FB3,
		float R_pot, float R_pot_offset);


static inline double calculate_converter_voltage_from_resistances_mV(
		float R_F1, float R_F2, float R_F3, float R_p) {
	return (R_F2 / ((R_F1 * (R_p + R_F3)) / (R_F1 + R_p + R_F3)) + 1.0f) * 1.215f * 1000.0f;
}


static void CAN_Logic_ReadServoGroup(uint8_t start_idx, uint32_t ack_id) {
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

	CAN_App_SendFrame(ack_id, payload, 6);
}


void CAN_Logic_HandleFrame(uint32_t id, uint8_t *data, uint8_t len) {
	switch(id) {
		case CAN_ID_GET_SERVO_CURRENT_1_3_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_1_3(len);
			break;
		case CAN_ID_GET_SERVO_CURRENT_4_6_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_4_6(len);
			break;
		case CAN_ID_GET_SERVO_CURRENT_7_9_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_7_9(len);
			break;
		case CAN_ID_GET_SERVO_CURRENT_10_12_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_10_12(len);
			break;
		case CAN_ID_GET_SERVO_CURRENT_13_15_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_13_15(len);
			break;
		case CAN_ID_GET_SERVO_CURRENT_16_18_REQ:
			CAN_Logic_Handle_GetCurrents_Servos_16_18(len);
			break;


		case CAN_ID_GET_I_MANIP_I_5V_POW_I_3V3_POW_REQ:
			CAN_Logic_Handle_GetCurrents_I_MANIP_I_5V_POW_I_3V3_POW(len);
			break;
		case CAN_ID_GET_I_STANDBY_I_SUPPLY_REQ:
			CAN_Logic_Handle_GetCurrents_I_STANDBY_I_SUPPLY(len);
			break;


		case CAN_ID_GET_U_CONVETERS_1_3_REQ:
			CAN_Logic_Handle_GetVoltages_Converters_1_3(len);
			break;
		case CAN_ID_GET_U_CONVETERS_4_5_REQ:
			CAN_Logic_Handle_GetVoltages_Converters_4_5(len);
			break;
		case CAN_ID_GET_U_SUPPLY_U_BAT_REQ:
			CAN_Logic_Handle_GetVoltages_U_SUPPLY_U_BAT(len);
			break;


		case CAN_ID_GET_TEMPERATURES_REQ:
			CAN_Logic_Handle_GetTemperatures(len);


		case CAN_ID_SET_CONVERTER1_VOLTAGE_REQ:
			CAN_Logic_Handle_SetVoltage_Converters1_3(data, len, 0);
			break;
		case CAN_ID_SET_CONVERTER2_VOLTAGE_REQ:
			CAN_Logic_Handle_SetVoltage_Converters1_3(data, len, 1);
			break;
		case CAN_ID_SET_CONVERTER3_VOLTAGE_REQ:
			CAN_Logic_Handle_SetVoltage_Converters1_3(data, len, 2);
			break;
		case CAN_ID_SET_CONVERTER4_VOLTAGE_REQ:
			CAN_Logic_Handle_SetVoltage_Converter4(data, len, 3);
			break;

		case CAN_ID_SET_SERVOS_STATES_REQ:
			CAN_Logic_Handle_SetServosStates(data, len);
			break;
		case CAN_ID_SET_MANIP_STATE_REQ:
			CAN_Logic_Handle_SetManipState(data, len);
			break;
		case CAN_ID_SET_VOLTAGE_OUTPUTS_STATES_REQ:
			CAN_Logic_Handle_SetVoltageOutputsStates(data, len);
			break;

		case CAN_ID_TURN_OFF_REQ:
			CAN_Logic_Handle_TurnOffPowerSupply(len);
			break;

		case CAN_ID_USE_BUZZER:
			CAN_Logic_Handle_Buzzer(data, len);
			break;

		case CAN_ID_GET_ERROR_CODES_REQ:
			CAN_Logic_Handle_GetErrorCodes(len);
			break;

		case CAN_ID_SERVICE_CLR_ERROR_REQ:
			CAN_Logic_Handle_Service_ClearCurrentError(len);
			break;

		default:
			break;
	}
}


void CAN_Logic_Handle_GetCurrents_Servos_1_3(uint8_t len) {
	if(len != 0) return;
	CAN_Logic_ReadServoGroup(0, CAN_ID_GET_SERVO_CURRENT_1_3_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_4_6(uint8_t len) {
	if(len != 0) return;
	CAN_Logic_ReadServoGroup(3, CAN_ID_GET_SERVO_CURRENT_4_6_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_7_9(uint8_t len) {
	if(len != 0) return;
	CAN_Logic_ReadServoGroup(6, CAN_ID_GET_SERVO_CURRENT_7_9_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_10_12(uint8_t len) {
	if(len != 0) return;
	CAN_Logic_ReadServoGroup(9, CAN_ID_GET_SERVO_CURRENT_10_12_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_13_15(uint8_t len) {
	if(len != 0) return;
	CAN_Logic_ReadServoGroup(12, CAN_ID_GET_SERVO_CURRENT_13_15_ACK);
}


void CAN_Logic_Handle_GetCurrents_Servos_16_18(uint8_t len) {
	if(len != 0) return;
	CAN_Logic_ReadServoGroup(15, CAN_ID_GET_SERVO_CURRENT_16_18_ACK);
}


void CAN_Logic_Handle_GetCurrents_I_MANIP_I_5V_POW_I_3V3_POW(uint8_t len) {
	if(len != 0) return;
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

	CAN_App_SendFrame(CAN_ID_GET_I_MANIP_I_5V_POW_I_3V3_POW_ACK, payload, 6);
}


void CAN_Logic_Handle_GetCurrents_I_STANDBY_I_SUPPLY(uint8_t len) {
	if(len != 0) return;
	uint16_t I_standby_current;
	uint16_t I_supply_current;

	I_standby_current = (uint16_t)I_standby;
	I_supply_current = (uint16_t)I_supply;

	uint8_t payload[4];
	payload[0] = I_standby_current >> 8;
	payload[1] = I_standby_current & 0xFF;
	payload[2] = I_supply_current >> 8;
	payload[3] = I_supply_current & 0xFF;

	CAN_App_SendFrame(CAN_ID_GET_I_STANDBY_I_SUPPLY_ACK, payload, 4);
}


void CAN_Logic_Handle_GetVoltages_Converters_1_3(uint8_t len) {
	if(len != 0) return;
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

	CAN_App_SendFrame(CAN_ID_GET_U_CONVETERS_1_3_ACK, payload, 6);
}


void CAN_Logic_Handle_GetVoltages_Converters_4_5(uint8_t len) {
	if(len != 0) return;
	uint16_t converter4_voltage;
	uint16_t converter5_voltage;

	converter4_voltage = (uint16_t)U_converter4;
	converter5_voltage = (uint16_t)U_converter5;

	uint8_t payload[4];
	payload[0] = converter4_voltage >> 8;
	payload[1] = converter4_voltage & 0xFF;
	payload[2] = converter5_voltage >> 8;
	payload[3] = converter5_voltage & 0xFF;

	CAN_App_SendFrame(CAN_ID_GET_U_CONVETERS_4_5_ACK, payload, 4);
}


void CAN_Logic_Handle_GetVoltages_U_SUPPLY_U_BAT(uint8_t len) {
	if(len != 0) return;
	uint16_t U_supply_voltage;
	uint16_t U_bat_voltage;

	U_supply_voltage = (uint16_t)U_supply;
	U_bat_voltage = (uint16_t)U_bat_ADC;

	uint8_t payload[4];
	payload[0] = U_supply_voltage >> 8;
	payload[1] = U_supply_voltage & 0xFF;
	payload[2] = U_bat_voltage >> 8;
	payload[3] = U_bat_voltage & 0xFF;

	CAN_App_SendFrame(CAN_ID_GET_U_SUPPLY_U_BAT_ACK, payload, 4);
}


void CAN_Logic_Handle_GetTemperatures(uint8_t len) {
	if(len != 0) return;
	int16_t temp1;
	int16_t temp2;
	int16_t temp3;

	temp1 = (int16_t)temperatures[0];
	temp2 = (int16_t)temperatures[1];
	temp3 = (int16_t)temperatures[2];

	uint8_t payload[6];
	payload[0] = temp1 >> 8;
	payload[1] = temp1 & 0xFF;
	payload[2] = temp2 >> 8;
	payload[3] = temp2 & 0xFF;
	payload[4] = temp3 >> 8;
	payload[5] = temp3 & 0xFF;

	CAN_App_SendFrame(CAN_ID_GET_TEMPERATURES_ACK, payload, 6);
}


void CAN_Logic_Handle_SetVoltage_Converters1_3(uint8_t* data, uint8_t len, uint8_t pot_index) {
	if(len != 2) return;
	uint16_t voltage_mV = (data[0] << 8) | data[1];
	if(voltage_mV > CONV1_3_MAX_VOLTAGE || voltage_mV < CONV1_3_MIN_VOLTAGE) {
		return;
	}
	pots[pot_index].target_value = converter_voltage_to_pot_value((float)voltage_mV/1000.0,
			CONV1_3_RFB1, CONV1_3_RFB2, CONV1_3_RFB3, POT_RESISTANCE, POT_RESISTANCE_OFFSET);
	pots[pot_index].done = 0;
}


void CAN_Logic_Handle_SetVoltage_Converter4(uint8_t* data, uint8_t len, uint8_t pot_index) {
	if(len != 2) return;
	uint16_t voltage_mV = (data[0] << 8) | data[1];
	if(voltage_mV > CONV4_MAX_VOLTAGE || voltage_mV < CONV4_MIN_VOLTAGE) {
		return;
	}
	pots[pot_index].target_value = converter_voltage_to_pot_value((float)voltage_mV/1000.0,
			CONV4_RFB1, CONV4_RFB2, CONV4_RFB3, POT_RESISTANCE, POT_RESISTANCE_OFFSET);
	pots[pot_index].done = 0;
}


void CAN_Logic_Handle_SetServosStates(uint8_t* data, uint8_t len) {
	if(len != 3) return;

	uint32_t mask =
			(data[0] << 16) |
			(data[1] << 8)  |
			data[2];

	mask &= 0x3FFFF;

	Servos_SetIntentMask(mask);
}


void CAN_Logic_Handle_SetVoltageOutputsStates(uint8_t* data, uint8_t len) {
	if(len != 2) return;
	uint8_t mask = (data[0] & 0x07)        // bits 0–2 - VOUT1–3
			| ((data[1] & 0x07) << 3); // bits 3–5 - VOUT4–6

	VoltageOutputs_SetTargetMask(mask);
}


void CAN_Logic_Handle_SetManipState(uint8_t* data, uint8_t len) {
	if(len != 1) return;
	manip_conv_en = data[0] & 0x01;
	manip_conv_en_done = 0;
}


void CAN_Logic_Handle_TurnOffPowerSupply(uint8_t len) {
	if(len != 0) return;
	turn_off_supply_request = 1;

	Buzzer_Start(2, BUZZ_HIGH_SHORT);
	CAN_App_SendFrame(CAN_ID_TURN_OFF_ACK, NULL, 0);
}


void CAN_Logic_Handle_Buzzer(uint8_t* data, uint8_t len) {
    if(len != 2) return;

    uint8_t times = data[0] & 0x07;
    uint8_t type  = data[1] & 0x03;

    if(times == 0 || times > 4 || buzzer.active) return;

    Buzzer_Start(times, (BuzzerSound)type);
}


void CAN_Logic_Handle_GetErrorCodes(uint8_t len) {
	if(len != 0) return;
	errm.error_pending_can = 1;
}


void CAN_Logic_Handle_Service_ClearCurrentError(uint8_t len) {
	if(len != 0) return;
	ErrorManager_CAN_DeleteLastViewed();
}


void CAN_Logic_Tick(void) {
	// sending ACK for SET_CONVERTER commends
	for(uint8_t i = 0; i < 3; i++) {
		if(pots[i].done == 1) {
			uint8_t payload[2];
			float Rp = ((float)pots[i].current_value/256.0) * POT_RESISTANCE + POT_RESISTANCE_OFFSET;
			uint16_t voltage_mV = (uint16_t)calculate_converter_voltage_from_resistances_mV(
					CONV1_3_RFB1, CONV1_3_RFB2, CONV1_3_RFB3, Rp);

			payload[0] = voltage_mV >> 8;
			payload[1] = voltage_mV & 0xFF;

			switch(i) {
			case 0:
				CAN_App_SendFrame(CAN_ID_SET_CONVERTER1_VOLTAGE_ACK, payload, 2);
				break;
			case 1:
				CAN_App_SendFrame(CAN_ID_SET_CONVERTER2_VOLTAGE_ACK, payload, 2);
				break;
			case 2:
				CAN_App_SendFrame(CAN_ID_SET_CONVERTER3_VOLTAGE_ACK, payload, 2);
				break;
			}
			pots[i].done = 2;
		}
	}

	if(pots[3].done == 1) {
		uint8_t payload[2];
		float Rp = ((float)pots[3].current_value/256.0) * POT_RESISTANCE + POT_RESISTANCE_OFFSET;
		uint16_t voltage_mV = (uint16_t)calculate_converter_voltage_from_resistances_mV(
				CONV4_RFB1, CONV4_RFB2, CONV4_RFB3, Rp);

		payload[0] = voltage_mV >> 8;
		payload[1] = voltage_mV & 0xFF;

		CAN_App_SendFrame(CAN_ID_SET_CONVERTER4_VOLTAGE_ACK, payload, 2);
		pots[3].done = 2;
	}

	if(servos.done == 1) {
		uint8_t payload[3];
		payload[0] = (servos.current_mask >> 16) & 0xFF;
		payload[1] = (servos.current_mask >> 8) & 0xFF;
		payload[2] =  servos.current_mask & 0xFF;

		CAN_App_SendFrame(CAN_ID_SET_SERVOS_STATES_ACK, payload, 3);

		servos.done = 2;
	}

	if(vouts.done == 1) {
		uint8_t payload[3];
		payload[0] = vouts.current_mask & 0x07; //VOUT1-3
		payload[1] = (vouts.current_mask >> 3) & 0x07; //VOUT4-6

		CAN_App_SendFrame(CAN_ID_SET_VOLTAGE_OUTPUTS_STATES_ACK, payload, 2);
		vouts.done = 2;
	}

	if(manip_conv_en_done == 1) {
		uint8_t payload = manip_conv_en;
		CAN_App_SendFrame(CAN_ID_SET_MANIP_STATE_ACK, &payload, 1);
		manip_conv_en_done = 2;
	}

	if(pressed_off_button == 1) {
		CAN_App_SendFrame(CAN_ID_PRESSED_OFF_BUTTON, NULL, 0);
		pressed_off_button = 0;
	}

	if(errm.error_pending_can == 1) {
	    uint8_t payload[8];
	    uint8_t len = ErrorManager_BuildErrorPayload(payload);

	    CAN_App_SendFrame(CAN_ID_GET_ERROR_CODES_ACK, payload, len);
	    errm.error_pending_can = 0;
	}

	if(errm.can_delete_error_done == 1) {
		CAN_App_SendFrame(CAN_ID_SERVICE_CLR_ERROR_ACK, NULL, 0);
		errm.can_delete_error_done = 2;
	}
}

