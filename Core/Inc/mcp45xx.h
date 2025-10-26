#ifndef MCP45XX_H
#define MCP45XX_H

#include <pcf8574.h>

#include "i2c.h"
#include "gpio.h"

// Volatile memory
#define MCP45xx_VOLATILE_WIPER_0 0x00
#define MCP45xx_TCON_REG 0x04
#define MCP45xx_STATUS_REG 0x05

// Non-volatile memory
#define MCP45xx_NONVOLATILE_WIPER_0 0x02

// Commands
#define MCP45xx_WRITE_OP 0b00000000
#define MCP45xx_INCREMENT_OP 0b00000100
#define MCP45xx_DECREMENT_OP 0b00001000
#define MCP45xx_READ_OP 0b00001100


typedef struct{
    I2C_HandleTypeDef* hi2c;
    PCF8574_HandleTypeDef* pcf;

    uint8_t addr;

    uint8_t en_pin;
    uint8_t hvc_pin;
    uint8_t tx_data[2];
    uint8_t rx_data[2];

} MCP45xx_HandleTypeDef;

HAL_StatusTypeDef MCP45xx_init(MCP45xx_HandleTypeDef* mcp, PCF8574_HandleTypeDef* pcf, uint8_t en_pin, uint8_t hvc_pin,
                  I2C_HandleTypeDef* hi2c, uint8_t addr);
void MCP45xx_write_volatile(MCP45xx_HandleTypeDef* mcp, uint16_t n);
void MCP45xx_write_non_volatile(MCP45xx_HandleTypeDef* mcp, uint16_t n);

void MCP45xx_increment_volatile(MCP45xx_HandleTypeDef* mcp);
void MCP45xx_decrement_volatile(MCP45xx_HandleTypeDef* mcp);

uint16_t MCP45xx_read_volatile(MCP45xx_HandleTypeDef* mcp);
uint16_t MCP45xx_read_non_volatile(MCP45xx_HandleTypeDef* mcp);

#endif //MCP45XX_H
