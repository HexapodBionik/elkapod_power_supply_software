#include "mcp454x.h"

inline void MCP454x_I2C_Transmit_HVC(MCP454x_HandleTypeDef* mcp, uint8_t* data){
    // Set HVC high
    PCF7485_write_pin(mcp->pcf, mcp->hvc_pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    // Transmit Data
    while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr << 1, data, sizeof(data));
    HAL_Delay(1);

    // Set HVC low
    PCF7485_write_pin(mcp->pcf, mcp->hvc_pin, GPIO_PIN_SET);
}

static void MCP454x_disable_WL0(MCP454x_HandleTypeDef* mcp){
    uint8_t command_byte = 0x00;
    command_byte |= (MCP454x_NONVOLATILE_WIPER_0 << 4);
    command_byte |= MCP454x_INCREMENT_OP;

    MCP454x_I2C_Transmit_HVC(mcp, &command_byte);
}

static void MCP454x_disable_WP(MCP454x_HandleTypeDef* mcp){
    uint8_t command_byte = 0x00;
    command_byte |= (MCP454x_DATA_EEPROM_9 << 4);
    command_byte |= MCP454x_INCREMENT_OP;

    MCP454x_I2C_Transmit_HVC(mcp, &command_byte);
}


static void MCP454x_enable_WL0(MCP454x_HandleTypeDef* mcp){
    uint8_t command_byte = 0x00;
    command_byte |= (MCP454x_NONVOLATILE_WIPER_0 << 4);
    command_byte |= MCP454x_DECREMENT_OP;

    MCP454x_I2C_Transmit_HVC(mcp, &command_byte);
}

static void MCP454x_enable_WP(MCP454x_HandleTypeDef* mcp){
    uint8_t command_byte = 0x00;
    command_byte |= (MCP454x_DATA_EEPROM_9 << 4);
    command_byte |= MCP454x_DECREMENT_OP;

    MCP454x_I2C_Transmit_HVC(mcp, &command_byte);
}

inline void MCP454x_I2C_Transmit_Normal(MCP454x_HandleTypeDef* mcp, uint8_t* data){
    // Set WL0 and WP bits to 0
    MCP454x_disable_WP(mcp);
    MCP454x_disable_WL0(mcp);

    // Transmit Data
    while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, data, sizeof(data));

    MCP454x_enable_WL0(mcp);
    MCP454x_enable_WP(mcp);
}

inline uint16_t MCP454x_I2C_Read_Random(MCP454x_HandleTypeDef* mcp, uint8_t command){
    uint8_t data[2];
    // Read data
    while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
    HAL_I2C_Mem_Read_DMA(mcp->hi2c, mcp->addr << 1, (uint16_t)command, sizeof(data), data, sizeof(data));

    uint16_t read_value = data[1];
    read_value |= data[0] << 8;
    return read_value;
}

uint8_t MPC454x_read_non_volatile(MCP454x_HandleTypeDef* mcp);

void MCP454x_init(MCP454x_HandleTypeDef* mcp, PCF7485_HandleTypeDef* pcf, uint8_t en_pin, uint8_t hvc_pin,
                  I2C_HandleTypeDef* hi2c, uint8_t addr){
    mcp->hi2c = hi2c;
    mcp->addr = addr;

    mcp->pcf = pcf;
    mcp->en_pin = en_pin;
    mcp->hvc_pin = hvc_pin;


    // Set AD0 PIN state
    PCF7485_write_pin(mcp->pcf, mcp->en_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // POR even (power on)
    PCF7485_write_pin(mcp->pcf, mcp->en_pin, GPIO_PIN_SET);

    HAL_Delay(1);

    // Get the value from the Non-Volatile WIPER register
    mcp->eeprom_value = MPC454x_read_non_volatile(mcp);
}

void MCP454x_write_volatile(MCP454x_HandleTypeDef* mcp, uint8_t n){
    uint8_t data[2] ={ 0x00, n};
    data[0] |= (MCP454x_VOLATILE_WIPER_0 << 4);
    data[0] |= MCP454x_WRITE_OP;

    MCP454x_I2C_Transmit_Normal(mcp, data);
}

void MPC454x_write_non_volatile(MCP454x_HandleTypeDef* mcp, uint8_t n){
    uint8_t data[2] ={ 0x00, n};
    data[0]  |= (MCP454x_NONVOLATILE_WIPER_0 << 4);
    data[0]  |= MCP454x_WRITE_OP;

    MCP454x_I2C_Transmit_Normal(mcp, data);
}

void MCP454x_increment_volatile(MCP454x_HandleTypeDef* mcp){
    uint8_t data = 0x00;
    data  |= (MCP454x_VOLATILE_WIPER_0 << 4);
    data  |= MCP454x_INCREMENT_OP;

    MCP454x_I2C_Transmit_Normal(mcp, &data);
}

void MCP454x_decrement_volatile(MCP454x_HandleTypeDef* mcp){
    uint8_t data = 0x00;
    data  |= (MCP454x_VOLATILE_WIPER_0 << 4);
    data  |= MCP454x_DECREMENT_OP;

    MCP454x_I2C_Transmit_Normal(mcp, &data);
}


uint8_t MPC454x_read_volatile(MCP454x_HandleTypeDef* mcp){
    uint8_t command = 0x00;
    command  |= (MCP454x_VOLATILE_WIPER_0 << 4);
    command  |= MCP454x_READ_OP;

    uint16_t read_value = MCP454x_I2C_Read_Random(mcp, command);
    return (uint8_t)read_value;
}

uint8_t MPC454x_read_non_volatile(MCP454x_HandleTypeDef* mcp){
    uint8_t command = 0x00;
    command  |= (MCP454x_NONVOLATILE_WIPER_0 << 4);
    command  |= MCP454x_READ_OP;

    uint16_t read_value = MCP454x_I2C_Read_Random(mcp, command);
    return (uint8_t)read_value;
}
