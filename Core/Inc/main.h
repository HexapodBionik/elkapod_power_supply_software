/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_3
#define BUZZER_GPIO_Port GPIOE
#define SW_POWER_ON_Pin GPIO_PIN_13
#define SW_POWER_ON_GPIO_Port GPIOC
#define CONV4_OC_Pin GPIO_PIN_14
#define CONV4_OC_GPIO_Port GPIOC
#define CONV5_OC_Pin GPIO_PIN_15
#define CONV5_OC_GPIO_Port GPIOC
#define VOLTAGE_EN_Pin GPIO_PIN_2
#define VOLTAGE_EN_GPIO_Port GPIOF
#define U_TEMP_Pin GPIO_PIN_3
#define U_TEMP_GPIO_Port GPIOF
#define I_MANIP_SENSE_Pin GPIO_PIN_4
#define I_MANIP_SENSE_GPIO_Port GPIOF
#define I_5V_POW_SENSE_Pin GPIO_PIN_5
#define I_5V_POW_SENSE_GPIO_Port GPIOF
#define I_3V3_POW_SENSE_Pin GPIO_PIN_6
#define I_3V3_POW_SENSE_GPIO_Port GPIOF
#define I_STANDBY_SENSE_Pin GPIO_PIN_8
#define I_STANDBY_SENSE_GPIO_Port GPIOF
#define I_SERVO1_Pin GPIO_PIN_9
#define I_SERVO1_GPIO_Port GPIOF
#define I_SERVO2_Pin GPIO_PIN_10
#define I_SERVO2_GPIO_Port GPIOF
#define I_SERVO3_Pin GPIO_PIN_0
#define I_SERVO3_GPIO_Port GPIOC
#define I_SERVO4_Pin GPIO_PIN_1
#define I_SERVO4_GPIO_Port GPIOC
#define I_SERVO5_Pin GPIO_PIN_2
#define I_SERVO5_GPIO_Port GPIOC
#define I_SERVO6_Pin GPIO_PIN_3
#define I_SERVO6_GPIO_Port GPIOC
#define I_SERVO7_Pin GPIO_PIN_0
#define I_SERVO7_GPIO_Port GPIOA
#define I_SERVO8_Pin GPIO_PIN_1
#define I_SERVO8_GPIO_Port GPIOA
#define I_SERVO9_Pin GPIO_PIN_2
#define I_SERVO9_GPIO_Port GPIOA
#define I_SERVO10_Pin GPIO_PIN_3
#define I_SERVO10_GPIO_Port GPIOA
#define I_SERVO11_Pin GPIO_PIN_4
#define I_SERVO11_GPIO_Port GPIOA
#define I_SERVO12_Pin GPIO_PIN_5
#define I_SERVO12_GPIO_Port GPIOA
#define I_SERVO13_Pin GPIO_PIN_6
#define I_SERVO13_GPIO_Port GPIOA
#define I_SERVO14_Pin GPIO_PIN_7
#define I_SERVO14_GPIO_Port GPIOA
#define I_SERVO15_Pin GPIO_PIN_4
#define I_SERVO15_GPIO_Port GPIOC
#define I_SERVO16_Pin GPIO_PIN_5
#define I_SERVO16_GPIO_Port GPIOC
#define I_SERVO17_Pin GPIO_PIN_0
#define I_SERVO17_GPIO_Port GPIOB
#define I_SERVO18_Pin GPIO_PIN_1
#define I_SERVO18_GPIO_Port GPIOB
#define SERVO18_EN_Pin GPIO_PIN_2
#define SERVO18_EN_GPIO_Port GPIOB
#define SERVO17_EN_Pin GPIO_PIN_11
#define SERVO17_EN_GPIO_Port GPIOF
#define SERVO16_EN_Pin GPIO_PIN_12
#define SERVO16_EN_GPIO_Port GPIOF
#define SERVO15_EN_Pin GPIO_PIN_13
#define SERVO15_EN_GPIO_Port GPIOF
#define SERVO14_EN_Pin GPIO_PIN_14
#define SERVO14_EN_GPIO_Port GPIOF
#define SERVO13_EN_Pin GPIO_PIN_15
#define SERVO13_EN_GPIO_Port GPIOF
#define SERVO12_EN_Pin GPIO_PIN_0
#define SERVO12_EN_GPIO_Port GPIOG
#define SERVO11_EN_Pin GPIO_PIN_1
#define SERVO11_EN_GPIO_Port GPIOG
#define SERVO10_EN_Pin GPIO_PIN_7
#define SERVO10_EN_GPIO_Port GPIOE
#define SERVO9_EN_Pin GPIO_PIN_8
#define SERVO9_EN_GPIO_Port GPIOE
#define SERVO8_EN_Pin GPIO_PIN_9
#define SERVO8_EN_GPIO_Port GPIOE
#define SERVO7_EN_Pin GPIO_PIN_10
#define SERVO7_EN_GPIO_Port GPIOE
#define SERVO6_EN_Pin GPIO_PIN_11
#define SERVO6_EN_GPIO_Port GPIOE
#define SERVO5_EN_Pin GPIO_PIN_12
#define SERVO5_EN_GPIO_Port GPIOE
#define SERVO4_EN_Pin GPIO_PIN_13
#define SERVO4_EN_GPIO_Port GPIOE
#define SERVO3_EN_Pin GPIO_PIN_14
#define SERVO3_EN_GPIO_Port GPIOE
#define SERVO2_EN_Pin GPIO_PIN_15
#define SERVO2_EN_GPIO_Port GPIOE
#define SERVO1_EN_Pin GPIO_PIN_10
#define SERVO1_EN_GPIO_Port GPIOB
#define LED_CONV1_Pin GPIO_PIN_11
#define LED_CONV1_GPIO_Port GPIOB
#define LED_CONV2_Pin GPIO_PIN_13
#define LED_CONV2_GPIO_Port GPIOB
#define LED_CONV3_Pin GPIO_PIN_14
#define LED_CONV3_GPIO_Port GPIOB
#define LED_CONV4_Pin GPIO_PIN_15
#define LED_CONV4_GPIO_Port GPIOB
#define LED_CONV5_Pin GPIO_PIN_8
#define LED_CONV5_GPIO_Port GPIOD
#define LED_STATUS4_Pin GPIO_PIN_9
#define LED_STATUS4_GPIO_Port GPIOD
#define LED_STATUS3_Pin GPIO_PIN_10
#define LED_STATUS3_GPIO_Port GPIOD
#define LED_STATUS2_Pin GPIO_PIN_11
#define LED_STATUS2_GPIO_Port GPIOD
#define LED_STATUS1_Pin GPIO_PIN_12
#define LED_STATUS1_GPIO_Port GPIOD
#define BAT_INDICATOR1_Pin GPIO_PIN_13
#define BAT_INDICATOR1_GPIO_Port GPIOD
#define BAT_INDICATOR2_Pin GPIO_PIN_14
#define BAT_INDICATOR2_GPIO_Port GPIOD
#define BAT_INDICATOR3_Pin GPIO_PIN_15
#define BAT_INDICATOR3_GPIO_Port GPIOD
#define BAT_INDICATOR4_Pin GPIO_PIN_2
#define BAT_INDICATOR4_GPIO_Port GPIOG
#define BAT_INDICATOR5_Pin GPIO_PIN_3
#define BAT_INDICATOR5_GPIO_Port GPIOG
#define LED_FATAL_ERROR_Pin GPIO_PIN_4
#define LED_FATAL_ERROR_GPIO_Port GPIOG
#define LED_POWER_ON_Pin GPIO_PIN_5
#define LED_POWER_ON_GPIO_Port GPIOG
#define SW_FUNC_Pin GPIO_PIN_6
#define SW_FUNC_GPIO_Port GPIOG
#define SW_CLR_ERR_Pin GPIO_PIN_7
#define SW_CLR_ERR_GPIO_Port GPIOG
#define SW_BAT_INDICATOR_Pin GPIO_PIN_8
#define SW_BAT_INDICATOR_GPIO_Port GPIOG
#define FAN_Pin GPIO_PIN_9
#define FAN_GPIO_Port GPIOC
#define OPTIONAL_Pin GPIO_PIN_0
#define OPTIONAL_GPIO_Port GPIOD
#define MUX_B_Pin GPIO_PIN_3
#define MUX_B_GPIO_Port GPIOD
#define MUX_A_Pin GPIO_PIN_4
#define MUX_A_GPIO_Port GPIOD
#define SW_EMERGENCY_Pin GPIO_PIN_7
#define SW_EMERGENCY_GPIO_Port GPIOD
#define V_OUT_EN6_Pin GPIO_PIN_10
#define V_OUT_EN6_GPIO_Port GPIOG
#define V_OUT_EN5_Pin GPIO_PIN_11
#define V_OUT_EN5_GPIO_Port GPIOG
#define V_OUT_EN4_Pin GPIO_PIN_12
#define V_OUT_EN4_GPIO_Port GPIOG
#define V_OUT_EN3_Pin GPIO_PIN_5
#define V_OUT_EN3_GPIO_Port GPIOB
#define V_OUT_EN2_Pin GPIO_PIN_6
#define V_OUT_EN2_GPIO_Port GPIOB
#define V_OUT_EN1_Pin GPIO_PIN_8
#define V_OUT_EN1_GPIO_Port GPIOB
#define CONV1_OC_Pin GPIO_PIN_9
#define CONV1_OC_GPIO_Port GPIOB
#define CONV2_OC_Pin GPIO_PIN_0
#define CONV2_OC_GPIO_Port GPIOE
#define CONV3_OC_Pin GPIO_PIN_1
#define CONV3_OC_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define ADC1_CHANNELS 16

#define I_SERVO3_ADC1_channel 0
#define I_SERVO4_ADC1_channel 1
#define I_SERVO5_ADC1_channel 2
#define I_SERVO6_ADC1_channel 3
#define I_SERVO7_ADC1_channel 4
#define I_SERVO8_ADC1_channel 5
#define I_SERVO9_ADC1_channel 6
#define I_SERVO10_ADC1_channel 7
#define I_SERVO11_ADC1_channel 8
#define I_SERVO12_ADC1_channel 9
#define I_SERVO13_ADC1_channel 10
#define I_SERVO14_ADC1_channel 11
#define I_SERVO15_ADC1_channel 12
#define I_SERVO16_ADC1_channel 13
#define I_SERVO17_ADC1_channel 14
#define I_SERVO18_ADC1_channel 15


#define I_SERVO1_ADC3_channel 11
#define I_SERVO2_ADC3_channel 12


#define EXPANDER1_CONV1_EN 0
#define EXPANDER1_CONV2_EN 1
#define EXPANDER1_CONV3_EN 2
#define EXPANDER1_CONV4_EN 3
#define EXPANDER1_CONV5_EN 4
#define EXPANDER1_POT_HVC 6
#define EXPANDER1_POT_EN 7




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
