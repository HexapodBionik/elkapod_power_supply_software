/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, VOLTAGE_EN_Pin|SERVO17_EN_Pin|SERVO16_EN_Pin|SERVO15_EN_Pin
                          |SERVO14_EN_Pin|SERVO13_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SERVO18_EN_Pin|SERVO1_EN_Pin|LED_CONV1_Pin|LED_CONV2_Pin
                          |LED_CONV3_Pin|LED_CONV4_Pin|V_OUT_EN3_Pin|V_OUT_EN2_Pin
                          |V_OUT_EN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SERVO12_EN_Pin|SERVO11_EN_Pin|BAT_INDICATOR4_Pin|BAT_INDICATOR5_Pin
                          |LED_FATAL_ERROR_Pin|LED_POWER_ON_Pin|V_OUT_EN6_Pin|V_OUT_EN5_Pin
                          |V_OUT_EN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SERVO10_EN_Pin|SERVO9_EN_Pin|SERVO8_EN_Pin|SERVO7_EN_Pin
                          |SERVO6_EN_Pin|SERVO5_EN_Pin|SERVO4_EN_Pin|SERVO3_EN_Pin
                          |SERVO2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_CONV5_Pin|LED_STATUS4_Pin|LED_STATUS3_Pin|LED_STATUS2_Pin
                          |LED_STATUS1_Pin|BAT_INDICATOR1_Pin|BAT_INDICATOR2_Pin|BAT_INDICATOR3_Pin
                          |OPTIONAL_Pin|MUX_B_Pin|MUX_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CONV4_OC_Pin CONV5_OC_Pin */
  GPIO_InitStruct.Pin = CONV4_OC_Pin|CONV5_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VOLTAGE_EN_Pin SERVO17_EN_Pin SERVO16_EN_Pin SERVO15_EN_Pin
                           SERVO14_EN_Pin SERVO13_EN_Pin */
  GPIO_InitStruct.Pin = VOLTAGE_EN_Pin|SERVO17_EN_Pin|SERVO16_EN_Pin|SERVO15_EN_Pin
                          |SERVO14_EN_Pin|SERVO13_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO18_EN_Pin SERVO1_EN_Pin LED_CONV1_Pin LED_CONV2_Pin
                           LED_CONV3_Pin LED_CONV4_Pin V_OUT_EN3_Pin V_OUT_EN2_Pin
                           V_OUT_EN1_Pin */
  GPIO_InitStruct.Pin = SERVO18_EN_Pin|SERVO1_EN_Pin|LED_CONV1_Pin|LED_CONV2_Pin
                          |LED_CONV3_Pin|LED_CONV4_Pin|V_OUT_EN3_Pin|V_OUT_EN2_Pin
                          |V_OUT_EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO12_EN_Pin SERVO11_EN_Pin BAT_INDICATOR4_Pin BAT_INDICATOR5_Pin
                           LED_FATAL_ERROR_Pin LED_POWER_ON_Pin V_OUT_EN6_Pin V_OUT_EN5_Pin
                           V_OUT_EN4_Pin */
  GPIO_InitStruct.Pin = SERVO12_EN_Pin|SERVO11_EN_Pin|BAT_INDICATOR4_Pin|BAT_INDICATOR5_Pin
                          |LED_FATAL_ERROR_Pin|LED_POWER_ON_Pin|V_OUT_EN6_Pin|V_OUT_EN5_Pin
                          |V_OUT_EN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO10_EN_Pin SERVO9_EN_Pin SERVO8_EN_Pin SERVO7_EN_Pin
                           SERVO6_EN_Pin SERVO5_EN_Pin SERVO4_EN_Pin SERVO3_EN_Pin
                           SERVO2_EN_Pin */
  GPIO_InitStruct.Pin = SERVO10_EN_Pin|SERVO9_EN_Pin|SERVO8_EN_Pin|SERVO7_EN_Pin
                          |SERVO6_EN_Pin|SERVO5_EN_Pin|SERVO4_EN_Pin|SERVO3_EN_Pin
                          |SERVO2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_CONV5_Pin LED_STATUS4_Pin LED_STATUS3_Pin LED_STATUS2_Pin
                           LED_STATUS1_Pin BAT_INDICATOR1_Pin BAT_INDICATOR2_Pin BAT_INDICATOR3_Pin
                           OPTIONAL_Pin MUX_B_Pin MUX_A_Pin */
  GPIO_InitStruct.Pin = LED_CONV5_Pin|LED_STATUS4_Pin|LED_STATUS3_Pin|LED_STATUS2_Pin
                          |LED_STATUS1_Pin|BAT_INDICATOR1_Pin|BAT_INDICATOR2_Pin|BAT_INDICATOR3_Pin
                          |OPTIONAL_Pin|MUX_B_Pin|MUX_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_FUNC_Pin SW_CLR_ERR_Pin SW_BAT_INDICATOR_Pin */
  GPIO_InitStruct.Pin = SW_FUNC_Pin|SW_CLR_ERR_Pin|SW_BAT_INDICATOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_EMERGENCY_Pin */
  GPIO_InitStruct.Pin = SW_EMERGENCY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_EMERGENCY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CONV1_OC_Pin */
  GPIO_InitStruct.Pin = CONV1_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CONV1_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CONV2_OC_Pin CONV3_OC_Pin */
  GPIO_InitStruct.Pin = CONV2_OC_Pin|CONV3_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
