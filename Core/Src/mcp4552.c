#include "mcp4552.h"


HAL_StatusTypeDef MCP4552_init(MCP4552_HandleTypeDef* mcp, I2C_HandleTypeDef* hi2c, uint8_t addr){
	mcp->hi2c = hi2c;
	mcp->addr = addr;

    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(mcp->hi2c, mcp->addr, 5, 100);
	if(status != HAL_OK){
	   return status;
	}
	return HAL_OK;
}


void MCP4552_write_volatile(MCP4552_HandleTypeDef* mcp, uint16_t value){
	mcp->tx_data[0] = (MCP45xx_VOLATILE_WIPER_0 << 4) | ((value >> 8) & 0x01);
	mcp->tx_data[1] = value & 0xFF;

	while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	if (HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, mcp->tx_data, 2) != HAL_OK) {
		Error_Handler();
	}
}


void MCP4552_increment_volatile(MCP4552_HandleTypeDef* mcp){
	mcp->tx_data[0] = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_INCREMENT_OP;

	while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, &mcp->tx_data[0], 1);
}


void MCP4552_decrement_volatile(MCP4552_HandleTypeDef* mcp){
	mcp->tx_data[0] = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_DECREMENT_OP;

    while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, &mcp->tx_data[0], 1);
}


uint16_t MCP4552_read_volatile(MCP4552_HandleTypeDef* mcp) {
	mcp->tx_data[0] = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_READ_OP;

	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	if (HAL_I2C_Mem_Read_DMA(mcp->hi2c, mcp->addr, mcp->tx_data[0],
			I2C_MEMADD_SIZE_8BIT, mcp->rx_data, 2) != HAL_OK) {
		return 0xFFFF; // Read error
	}
	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);

	uint16_t value = ((mcp->rx_data[0] & 0x01) << 8) | mcp->rx_data[1];
	return value;
}

