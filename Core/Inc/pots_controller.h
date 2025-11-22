#ifndef POT_H
#define POT_H

#include "mcp4552.h"

typedef struct {
    MCP4552_HandleTypeDef* pot;
    uint16_t current_value;
    uint16_t target_value;
    uint8_t done;
} PotChannel;

void Pots_ResetCurrnetValue(void);
void Pots_Tick(void);

#endif
