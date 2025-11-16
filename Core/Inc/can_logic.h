#ifndef CAN_LOGIC_H_
#define CAN_LOGIC_H_

#include "main.h"
#include "can_app.h"
#include "pots_controller.h"

#define CALC_CONVERTER_VOLTAGE(RF1, RF2, RF3, RP) \
    ( (uint16_t)( ((RF2) / (((RF1) * ((RP) + (RF3))) / ((RF1) + (RP) + (RF3))) + 1.0f) * 1.215f * 1000.0f ) )


void CAN_Logic_HandleFrame(uint32_t id, uint8_t *data, uint8_t len);


void CAN_Logic_Handle_GetCurrents_Servos_1_3(uint8_t len);
void CAN_Logic_Handle_GetCurrents_Servos_4_6(uint8_t len);
void CAN_Logic_Handle_GetCurrents_Servos_7_9(uint8_t len);
void CAN_Logic_Handle_GetCurrents_Servos_10_12(uint8_t len);
void CAN_Logic_Handle_GetCurrents_Servos_13_15(uint8_t len);
void CAN_Logic_Handle_GetCurrents_Servos_16_18(uint8_t len);

void CAN_Logic_Handle_GetCurrents_I_MANIP_I_5V_POW_I_3V3_POW(uint8_t len);
void CAN_Logic_Handle_GetCurrents_I_STANDBY_I_SUPPLY(uint8_t len);

void CAN_Logic_Handle_GetVoltages_Converters_1_3(uint8_t len);
void CAN_Logic_Handle_GetVoltages_Converters_4_5(uint8_t len);
void CAN_Logic_Handle_GetVoltages_U_SUPPLY_U_BAT(uint8_t len);

void CAN_Logic_Handle_GetTemperatures(uint8_t len);

void CAN_Logic_Handle_SetVoltage_Converters1_3(uint8_t* data, uint8_t len, uint8_t pot_index);
void CAN_Logic_Handle_SetVoltage_Converter4(uint8_t* data, uint8_t len, uint8_t pot_index);

void CAN_Logic_Tick(void);

#endif
