#ifndef MCP45XX_H
#define MCP45XX_H


#include "i2c.h"
#include "i2c_manager.h"

// Volatile memory
#define MCP45xx_VOLATILE_WIPER_0 0x00
#define MCP45xx_TCON_REG 0x04
#define MCP45xx_STATUS_REG 0x05

// Commands
#define MCP45xx_WRITE_OP 0b00000000
#define MCP45xx_INCREMENT_OP 0b00000100
#define MCP45xx_DECREMENT_OP 0b00001000
#define MCP45xx_READ_OP 0b00001100


typedef struct{
    I2C_Manager* i2c_mgr;
    uint8_t addr;

    uint8_t tx_data[2];
    uint8_t rx_data[2];

} MCP4552_HandleTypeDef;

HAL_StatusTypeDef MCP4552_init(MCP4552_HandleTypeDef* mcp, I2C_Manager* i2c_mgr, uint8_t addr);
HAL_StatusTypeDef MCP4552_write_volatile(MCP4552_HandleTypeDef* mcp, uint16_t value);

HAL_StatusTypeDef MCP4552_increment_volatile(MCP4552_HandleTypeDef* mcp);
HAL_StatusTypeDef MCP4552_decrement_volatile(MCP4552_HandleTypeDef* mcp);

uint16_t MCP4552_read_volatile(MCP4552_HandleTypeDef* mcp);


#endif //MCP45XX_H
