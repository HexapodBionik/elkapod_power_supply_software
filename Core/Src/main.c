/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pcf8574.h"
#include "mcp45xx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define EXPANDER1_ADDRESS 0x70
#define EXPANDER2_ADDRESS 0x72

#define POT1_ADDRESS 0x5A
#define POT2_ADDRESS 0x5C
#define POT3_ADDRESS 0x58
#define POT4_ADDRESS 0x5E


#define CONV_EN_OC_DELAY 50


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PCF8574_HandleTypeDef expander1;
PCF8574_HandleTypeDef expander2;

MCP45xx_HandleTypeDef pot1;
MCP45xx_HandleTypeDef pot2;
MCP45xx_HandleTypeDef pot3;
MCP45xx_HandleTypeDef pot4;


uint16_t adc1_buf[ADC1_CHANNELS];

uint16_t I_servo1_adc_value = 1;
uint16_t I_servo2_adc_value = 1;
uint16_t I_servo3_adc_value = 1;
uint16_t I_servo4_adc_value = 1;
uint16_t I_servo5_adc_value = 1;
uint16_t I_servo6_adc_value = 1;
uint16_t I_servo7_adc_value = 1;
uint16_t I_servo8_adc_value = 1;
uint16_t I_servo9_adc_value = 1;
uint16_t I_servo10_adc_value = 1;
uint16_t I_servo11_adc_value = 1;
uint16_t I_servo12_adc_value = 1;
uint16_t I_servo13_adc_value = 1;
uint16_t I_servo14_adc_value = 1;
uint16_t I_servo15_adc_value = 1;
uint16_t I_servo16_adc_value = 1;
uint16_t I_servo17_adc_value = 1;
uint16_t I_servo18_adc_value = 1;

uint16_t adc_test_value1 = 0;
uint16_t adc_test_value3 = 0;

uint8_t conv1_oc_status = 0;
uint8_t conv2_oc_status = 0;
uint8_t conv3_oc_status = 0;
uint8_t conv4_oc_status = 0;
uint8_t conv5_oc_status = 0;

uint32_t last_tick_toggle_en_conv = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  uint8_t HVC_bool = 0;
  uint8_t pot_value = 1;
  uint8_t converters_en = 1;
  uint16_t read_value = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */


  HAL_GPIO_WritePin(SERVO1_EN_GPIO_Port, SERVO1_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO2_EN_GPIO_Port, SERVO2_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO3_EN_GPIO_Port, SERVO3_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO4_EN_GPIO_Port, SERVO4_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO5_EN_GPIO_Port, SERVO5_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO6_EN_GPIO_Port, SERVO6_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO7_EN_GPIO_Port, SERVO7_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO8_EN_GPIO_Port, SERVO8_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO9_EN_GPIO_Port, SERVO9_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO10_EN_GPIO_Port, SERVO10_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO11_EN_GPIO_Port, SERVO11_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO12_EN_GPIO_Port, SERVO12_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO13_EN_GPIO_Port, SERVO13_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO14_EN_GPIO_Port, SERVO14_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO15_EN_GPIO_Port, SERVO15_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO16_EN_GPIO_Port, SERVO16_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO17_EN_GPIO_Port, SERVO17_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SERVO18_EN_GPIO_Port, SERVO18_EN_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(VOLTAGE_EN_GPIO_Port, VOLTAGE_EN_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LED_CONV1_GPIO_Port, LED_CONV1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_CONV2_GPIO_Port, LED_CONV2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_CONV3_GPIO_Port, LED_CONV3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_CONV4_GPIO_Port, LED_CONV4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_CONV5_GPIO_Port, LED_CONV5_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(V_OUT_EN1_GPIO_Port, V_OUT_EN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(V_OUT_EN2_GPIO_Port, V_OUT_EN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(V_OUT_EN3_GPIO_Port, V_OUT_EN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(V_OUT_EN4_GPIO_Port, V_OUT_EN4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V_OUT_EN5_GPIO_Port, V_OUT_EN5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(V_OUT_EN6_GPIO_Port, V_OUT_EN6_Pin, GPIO_PIN_RESET);


  if(PCF7485_init(&expander1, &hi2c2, EXPANDER1_ADDRESS) != HAL_OK) {
	  Error_Handler();
  }

  if(PCF7485_init(&expander2, &hi2c2, EXPANDER2_ADDRESS) != HAL_OK) {
	  Error_Handler();
  }


  MCP45xx_init(&pot1, &expander1, EXPANDER1_POT_EN, EXPANDER1_POT_HVC, &hi2c2, POT1_ADDRESS);
  MCP45xx_init(&pot2, &expander1, EXPANDER1_POT_EN, EXPANDER1_POT_HVC, &hi2c2, POT2_ADDRESS);
  MCP45xx_init(&pot3, &expander1, EXPANDER1_POT_EN, EXPANDER1_POT_HVC, &hi2c2, POT3_ADDRESS);
  if(MCP45xx_init(&pot4, &expander1, EXPANDER1_POT_EN, EXPANDER1_POT_HVC, &hi2c2, POT4_ADDRESS) != HAL_OK) {
	  Error_Handler();
  }

//  HAL_TIM_Base_Start(&htim6);
//  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC1_CHANNELS) != HAL_OK) {
//	Error_Handler();
//  }

  PCF7485_write_pin(&expander1, EXPANDER1_CONV1_EN, GPIO_PIN_RESET);
  PCF7485_write_pin(&expander1, EXPANDER1_CONV2_EN, GPIO_PIN_RESET);
  PCF7485_write_pin(&expander1, EXPANDER1_CONV3_EN, GPIO_PIN_RESET);
  PCF7485_write_pin(&expander1, EXPANDER1_CONV4_EN, GPIO_PIN_RESET);
  PCF7485_write_pin(&expander1, EXPANDER1_CONV5_EN, GPIO_PIN_RESET);
  PCF7485_write_pin(&expander1, EXPANDER1_POT_HVC, GPIO_PIN_SET);
  PCF7485_write_pin(&expander1, EXPANDER1_POT_EN, GPIO_PIN_RESET);

  PCF7485_write_buffer(&expander2, 0xFF);

	// --- PWM FAN (500 Hz) ---
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 500); // 50% duty (500 / 999)

//	// --- BUZZER (1 kHz) ---
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500); // 50% duty cycle

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t main_iteration = 0;

  while (1)
  {
	HAL_GPIO_TogglePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin);
	HAL_GPIO_TogglePin(LED_STATUS2_GPIO_Port, LED_STATUS2_Pin);
	HAL_GPIO_TogglePin(LED_STATUS3_GPIO_Port, LED_STATUS3_Pin);
	HAL_GPIO_TogglePin(LED_STATUS4_GPIO_Port, LED_STATUS4_Pin);

	if(conv1_oc_status == 1) {
		HAL_GPIO_WritePin(BAT_INDICATOR1_GPIO_Port, BAT_INDICATOR1_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BAT_INDICATOR1_GPIO_Port, BAT_INDICATOR1_Pin, GPIO_PIN_RESET);
	}
	if(conv2_oc_status == 1) {
		HAL_GPIO_WritePin(BAT_INDICATOR2_GPIO_Port, BAT_INDICATOR2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BAT_INDICATOR2_GPIO_Port, BAT_INDICATOR2_Pin, GPIO_PIN_RESET);
	}
	if(conv3_oc_status == 1) {
		HAL_GPIO_WritePin(BAT_INDICATOR3_GPIO_Port, BAT_INDICATOR3_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BAT_INDICATOR3_GPIO_Port, BAT_INDICATOR3_Pin, GPIO_PIN_RESET);
	}
	if(conv4_oc_status == 1) {
		HAL_GPIO_WritePin(BAT_INDICATOR4_GPIO_Port, BAT_INDICATOR4_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BAT_INDICATOR4_GPIO_Port, BAT_INDICATOR4_Pin, GPIO_PIN_RESET);
	}
	if(conv5_oc_status == 1) {
		HAL_GPIO_WritePin(BAT_INDICATOR5_GPIO_Port, BAT_INDICATOR5_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BAT_INDICATOR5_GPIO_Port, BAT_INDICATOR5_Pin, GPIO_PIN_RESET);
	}


	if(main_iteration == 10) {
		MCP45xx_write_volatile(&pot4, 256);
		HAL_Delay(100);
		read_value = MCP45xx_read_volatile(&pot4);
	}
	if(main_iteration == 20) {
		MCP45xx_write_volatile(&pot4, 255);
		HAL_Delay(100);
		read_value = MCP45xx_read_volatile(&pot4);
	}
	if(main_iteration == 30) {
		MCP45xx_write_volatile(&pot4, 4);
		HAL_Delay(100);
		read_value = MCP45xx_read_volatile(&pot4);
	}
	if(main_iteration == 40) {
		MCP45xx_write_volatile(&pot4, 128);
		HAL_Delay(100);
		read_value = MCP45xx_read_volatile(&pot4);
	}
	if(main_iteration == 50) {
		MCP45xx_write_volatile(&pot4, 0);
		HAL_Delay(100);
		read_value = MCP45xx_read_volatile(&pot4);
		main_iteration = 0;
	}



	if(HAL_GPIO_ReadPin(SW_FUNC_GPIO_Port, SW_FUNC_Pin) == GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(converters_en == 0) {
			last_tick_toggle_en_conv = HAL_GetTick();
			PCF7485_write_pin(&expander1, EXPANDER1_CONV1_EN, GPIO_PIN_RESET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV2_EN, GPIO_PIN_RESET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV3_EN, GPIO_PIN_RESET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV4_EN, GPIO_PIN_RESET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV5_EN, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(LED_CONV1_GPIO_Port, LED_CONV1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_CONV2_GPIO_Port, LED_CONV2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_CONV3_GPIO_Port, LED_CONV3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_CONV4_GPIO_Port, LED_CONV4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_CONV5_GPIO_Port, LED_CONV5_Pin, GPIO_PIN_SET);
			converters_en = 1;
		} else {
			last_tick_toggle_en_conv = HAL_GetTick();
			PCF7485_write_pin(&expander1, EXPANDER1_CONV1_EN, GPIO_PIN_SET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV2_EN, GPIO_PIN_SET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV3_EN, GPIO_PIN_SET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV4_EN, GPIO_PIN_SET);
			PCF7485_write_pin(&expander1, EXPANDER1_CONV5_EN, GPIO_PIN_SET);

			HAL_GPIO_WritePin(LED_CONV1_GPIO_Port, LED_CONV1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_CONV2_GPIO_Port, LED_CONV2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_CONV3_GPIO_Port, LED_CONV3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_CONV4_GPIO_Port, LED_CONV4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_CONV5_GPIO_Port, LED_CONV5_Pin, GPIO_PIN_RESET);
			converters_en = 0;
		}
	}


//	if(HAL_GPIO_ReadPin(SW_BAT_INDICATOR_GPIO_Port, SW_BAT_INDICATOR_Pin) == GPIO_PIN_RESET) {
//		if(HVC_bool == 0) {
//			PCF7485_write_pin(&expander1, EXPANDER1_POT_HVC, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(LED_FATAL_ERROR_GPIO_Port, LED_FATAL_ERROR_Pin, GPIO_PIN_SET);
//			HVC_bool = 1;
//		} else {
//			PCF7485_write_pin(&expander1, EXPANDER1_POT_HVC, GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED_FATAL_ERROR_GPIO_Port, LED_FATAL_ERROR_Pin, GPIO_PIN_RESET);
//			HVC_bool = 0;
//		}
//		HAL_Delay(1000);
//	}


	if (HAL_GPIO_ReadPin(SW_BAT_INDICATOR_GPIO_Port, SW_BAT_INDICATOR_Pin) == GPIO_PIN_RESET) {
		if (pot_value == 1) {
			for (uint16_t i = 0; i <= 255; i++) {
				MCP45xx_increment_volatile(&pot1);
				MCP45xx_increment_volatile(&pot2);
				MCP45xx_increment_volatile(&pot3);
				MCP45xx_increment_volatile(&pot4);
				HAL_Delay(1);
			}
			HAL_Delay(1000);
			pot_value = 0;
		} else {
			for (uint16_t i = 255; i > 0; i--) {
				MCP45xx_decrement_volatile(&pot1);
				MCP45xx_decrement_volatile(&pot2);
				MCP45xx_decrement_volatile(&pot3);
				MCP45xx_decrement_volatile(&pot4);
				HAL_Delay(1);
			}
			pot_value = 1;
			HAL_Delay(1000);
		}
	}

	if(HAL_GPIO_ReadPin(SW_CLR_ERR_GPIO_Port, SW_CLR_ERR_Pin) == GPIO_PIN_RESET) {
		conv1_oc_status = 0;
		conv2_oc_status = 0;
		conv3_oc_status = 0;
		conv4_oc_status = 0;
		conv5_oc_status = 0;
	}



	HAL_ADC_Start(&hadc1);

	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	{
		adc_test_value1 = HAL_ADC_GetValue(&hadc1);

	}

	HAL_ADC_Stop(&hadc1);

	HAL_ADC_Start(&hadc3);

	if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
	{
		adc_test_value3 = HAL_ADC_GetValue(&hadc3);
	}

	HAL_ADC_Stop(&hadc3);

	HAL_Delay(100);

//	for (uint32_t freq = 1000; freq <= 4000; freq += 500) {
//		uint32_t period = (HAL_RCC_GetPCLK1Freq() / 80) / freq;
//		__HAL_TIM_SET_AUTORELOAD(&htim3, period);
//		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period / 2);
//		HAL_Delay(300);
//	}




	HAL_Delay(100);
	main_iteration++;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 2;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GetTick() - last_tick_toggle_en_conv > CONV_EN_OC_DELAY) {
		switch (GPIO_Pin)
		{
			case CONV1_OC_Pin:  // PB9
				conv1_oc_status = 1;   // Zmień stan (toggle)
				break;

			case CONV2_OC_Pin:  // PE0
				conv2_oc_status = 1;
				break;

			case CONV3_OC_Pin:  // PE1
				conv3_oc_status = 1;
				break;

			case CONV4_OC_Pin:  // PC14
				conv4_oc_status = 1;
				break;

			case CONV5_OC_Pin:  // PC15
				conv5_oc_status = 1;
				break;

			default:
				break;
		}
	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//    if (hadc->Instance == ADC1) {
//    	I_servo3_adc_value = adc1_buf[I_SERVO3_ADC1_channel];
//		I_servo4_adc_value = adc1_buf[I_SERVO4_ADC1_channel];
//		I_servo5_adc_value = adc1_buf[I_SERVO5_ADC1_channel];
//		I_servo6_adc_value = adc1_buf[I_SERVO6_ADC1_channel];
//		I_servo7_adc_value = adc1_buf[I_SERVO7_ADC1_channel];
//		I_servo8_adc_value = adc1_buf[I_SERVO8_ADC1_channel];
//		I_servo9_adc_value = adc1_buf[I_SERVO9_ADC1_channel];
//		I_servo10_adc_value = adc1_buf[I_SERVO10_ADC1_channel];
//		I_servo11_adc_value = adc1_buf[I_SERVO11_ADC1_channel];
//		I_servo12_adc_value = adc1_buf[I_SERVO12_ADC1_channel];
//		I_servo13_adc_value = adc1_buf[I_SERVO13_ADC1_channel];
//		I_servo14_adc_value = adc1_buf[I_SERVO14_ADC1_channel];
//		I_servo15_adc_value = adc1_buf[I_SERVO15_ADC1_channel];
//		I_servo16_adc_value = adc1_buf[I_SERVO16_ADC1_channel];
//		I_servo17_adc_value = adc1_buf[I_SERVO17_ADC1_channel];
//		I_servo18_adc_value = adc1_buf[I_SERVO18_ADC1_channel];
//    }
//
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
