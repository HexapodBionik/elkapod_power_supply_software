#include "mcp45xx.h"


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


HAL_StatusTypeDef MCP45xx_init(MCP45xx_HandleTypeDef* mcp, PCF8574_HandleTypeDef* pcf, uint8_t en_pin, uint8_t hvc_pin,
                  I2C_HandleTypeDef* hi2c, uint8_t addr){
	mcp->hi2c = hi2c;
	mcp->addr = addr;

	mcp->pcf = pcf;
	mcp->en_pin = en_pin;
	mcp->hvc_pin = hvc_pin;

    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(pcf->hi2c, pcf->addr, 5, 100);
	if(status != HAL_OK){
	   return status;
	}
	return HAL_OK;
}


void MCP45xx_write_volatile(MCP45xx_HandleTypeDef* mcp, uint16_t value){
	mcp->tx_data[0] = (MCP45xx_VOLATILE_WIPER_0 << 4) | ((value >> 8) & 0x01);
	mcp->tx_data[1] = value & 0xFF;

	while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	if (HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, mcp->tx_data, 2) != HAL_OK) {
		Error_Handler();
	}
}


void MCP45xx_increment_volatile(MCP45xx_HandleTypeDef* mcp){
	mcp->tx_data[0] = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_INCREMENT_OP;

	while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, &mcp->tx_data[0], 1);
}


void MCP45xx_decrement_volatile(MCP45xx_HandleTypeDef* mcp){
	mcp->tx_data[0] = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_DECREMENT_OP;

    while(HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
    HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, &mcp->tx_data[0], 1);
}


//uint16_t MCP45xx_read_volatile(MCP45xx_HandleTypeDef* mcp) {
//	uint8_t cmd = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_READ_OP;
//
//	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
//	if (HAL_I2C_Master_Transmit_DMA(mcp->hi2c, mcp->addr, &cmd, 1) != HAL_OK)
//		return 0xFFFF;
//	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
//	if (HAL_I2C_Master_Receive_DMA(mcp->hi2c, mcp->addr, mcp->rx_data, 2) != HAL_OK)
//		return 0xFFFF;
////	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
////	HAL_Delay(1);
//	DMA_HandleTypeDef *hdma = mcp->hi2c->hdmarx;
//	while (hdma->State != HAL_DMA_STATE_READY);
//
//	uint16_t value = ((mcp->rx_data[0] & 0x01) << 8) | mcp->rx_data[1];
//
//	return value;
//}

uint16_t MCP45xx_read_volatile(MCP45xx_HandleTypeDef* mcp) {
	uint8_t command = (MCP45xx_VOLATILE_WIPER_0 << 4) | MCP45xx_READ_OP;

	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);
	if (HAL_I2C_Mem_Read_DMA(mcp->hi2c, mcp->addr, command,
			I2C_MEMADD_SIZE_8BIT, mcp->rx_data, 2) != HAL_OK) {
		return 0xFFFF; // Read error
	}
	while (HAL_I2C_GetState(mcp->hi2c) != HAL_I2C_STATE_READY);

	uint16_t value = ((mcp->rx_data[0] & 0x01) << 8) | mcp->rx_data[1];
	return value;
}

