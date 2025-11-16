#ifndef CAN_LOGIC_H_
#define CAN_LOGIC_H_

#include "main.h"
#include "can_app.h"



void CAN_Logic_HandleFrame(uint32_t id, uint8_t *data, uint8_t len);


void CAN_Logic_Handle_GetCurrents_Servos_1_3(void);
void CAN_Logic_Handle_GetCurrents_Servos_4_6(void);
void CAN_Logic_Handle_GetCurrents_Servos_7_9(void);
void CAN_Logic_Handle_GetCurrents_Servos_10_12(void);
void CAN_Logic_Handle_GetCurrents_Servos_13_15(void);
void CAN_Logic_Handle_GetCurrents_Servos_16_18(void);

void CAN_Logic_Handle_GetCurrents_I_MANIP_I_5V_POW_I_3V3_POW(void);
void CAN_Logic_Handle_GetCurrents_I_STANDBY_I_SUPPLY(void);

void CAN_Logic_Handle_GetVoltages_Converters_1_3(void);
void CAN_Logic_Handle_GetVoltages_Converters_4_5(void);
void CAN_Logic_Handle_GetVoltages_U_SUPPLY_U_BAT(void);

void CAN_Logic_Handle_GetTemperatures(void);

#endif
