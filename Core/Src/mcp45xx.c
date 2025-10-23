#include <mcp45xx.h>


inline void MCP45xx_I2C_Transmit_HVC(MCP45xx_HandleTypeDef* mcp, uint8_t* data){
    // Set HVC high
    PCF7485_write_pin(mcp->pcf, mcp->hvc_pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    // Transmit Data
    while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, data, sizeof(data));
    HAL_Delay(1);

    // Set HVC low
    PCF7485_write_pin(mcp->pcf, mcp->hvc_pin, GPIO_PIN_SET);
}


void MCP45xx_init(MCP45xx_HandleTypeDef* mcp, PCF8574_HandleTypeDef* pcf, uint8_t en_pin, uint8_t hvc_pin,
                  I2C_HandleTypeDef* hi2c, uint8_t addr){
    mcp->hi2c = hi2c;
    mcp->addr = addr;

    mcp->pcf = pcf;
    mcp->en_pin = en_pin;
    mcp->hvc_pin = hvc_pin;
}


void MCP45xx_write_volatile(MCP45xx_HandleTypeDef* mcp, uint16_t n){
	uint8_t data[2];
	data[1] = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_WRITE_OP;
	data[0] = (uint8_t)(n & 0xFF);

//	uint8_t data[2] ={0x00, (uint8_t)n};
//	data[0] |= (MCP45xx_VOLATILE_WIPER_0 << 4);
//	data[0] |= MCP45xx_WRITE_OP;

//	uint8_t data[2];
//	data[0] = 0b00000000;
//	data[1] = 0b00111111;

	while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	if (HAL_I2C_IsDeviceReady(mcp->hi2c, mcp->addr, 3, 1000) != HAL_OK) {
	    Error_Handler();
	}
	if(HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, data, 2) != HAL_OK) {
		Error_Handler();
	}
}


void MCP45xx_increment_volatile(MCP45xx_HandleTypeDef* mcp){
	uint8_t data = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_INCREMENT_OP;

	while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	if (HAL_I2C_IsDeviceReady(mcp->hi2c, mcp->addr, 3, 1000) != HAL_OK) {
		    Error_Handler();
		}
	HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, &data, sizeof(data));
}


void MCP45xx_decrement_volatile(MCP45xx_HandleTypeDef* mcp){
    uint8_t data = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_DECREMENT_OP;

    while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
    if (HAL_I2C_IsDeviceReady(mcp->hi2c, mcp->addr, 3, 1000) != HAL_OK) {
    	    Error_Handler();
    	}
    HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, &data, sizeof(data));
}


uint16_t MCP45xx_read_volatile(MCP45xx_HandleTypeDef* mcp) {
	uint8_t command = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_READ_OP;
	uint8_t data[2];


	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	if (HAL_I2C_IsDeviceReady(mcp->hi2c, mcp->addr, 3, 1000) != HAL_OK) {
		    Error_Handler();
		}
	if (HAL_I2C_Mem_Read_DMA(mcp->hi2c, mcp->addr, command,
			I2C_MEMADD_SIZE_8BIT, data, sizeof(data)) != HAL_OK) {
		return 0xFFFF;  // Read error
	}

	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	uint16_t read_value = (data[0] << 8) | data[1];

	return read_value;
}
