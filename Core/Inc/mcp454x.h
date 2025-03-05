#ifndef MCP454X_H
#define MCP454X_H

#include "i2c.h"
#include "gpio.h"
#include "pcf8574_expander.h"

// Volatile memory
#define MCP454x_STATUS_REG 0x05
#define MCP454x_TCON_REG 0x04
#define MCP454x_VOLATILE_WIPER_0 0x00

// Non-volatile memory
#define MCP454x_NONVOLATILE_WIPER_0 0x02
#define MCP454x_DATA_EEPROM_0 0x06
#define MCP454x_DATA_EEPROM_1 0x07
#define MCP454x_DATA_EEPROM_2 0x08
#define MCP454x_DATA_EEPROM_3 0x09
#define MCP454x_DATA_EEPROM_4 0x0a
#define MCP454x_DATA_EEPROM_5 0x0b
#define MCP454x_DATA_EEPROM_6 0x0c
#define MCP454x_DATA_EEPROM_7 0x0d
#define MCP454x_DATA_EEPROM_8 0x0e
#define MCP454x_DATA_EEPROM_9 0x0f

// Commands
#define MCP454x_WRITE_OP 0b00000000
#define MCP454x_INCREMENT_OP 0b00000100
#define MCP454x_DECREMENT_OP 0b00001000
#define MCP454x_READ_OP 0b00001100


typedef struct{
    I2C_HandleTypeDef* hi2c;
    PCF7485_HandleTypeDef* pcf;

    uint8_t addr;

    uint8_t en_pin;
    uint8_t hvc_pin;

    uint8_t eeprom_value;
} MCP454x_HandleTypeDef;

void MCP454x_init(MCP454x_HandleTypeDef* mcp, PCF7485_HandleTypeDef* pcf, uint8_t en_pin, uint8_t hvc_pin,
                  I2C_HandleTypeDef* hi2c, uint8_t addr);
void MCP454x_write_volatile(MCP454x_HandleTypeDef* mcp, uint8_t n);
void MCP454x_write_non_volatile(MCP454x_HandleTypeDef* mcp, uint8_t n);

void MCP454x_increment_volatile(MCP454x_HandleTypeDef* mcp);
void MCP454x_decrement_volatile(MCP454x_HandleTypeDef* mcp);

uint8_t MCP454x_read_volatile(MCP454x_HandleTypeDef* mcp);
uint8_t MCP454x_read_non_volatile(MCP454x_HandleTypeDef* mcp);

#endif MCP454X_H
