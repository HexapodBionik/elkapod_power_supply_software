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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hardware_config.h"
#include "pcf8574.h"
#include "mcp4552.h"
#include "adc121s021.h"
#include "i2c_manager.h"
#include "can_app.h"
#include "can_logic.h"
#include "pots_controller.h"
#include "servos_controller.h"
#include "voltage_outputs_controller.h"
#include "sequences_functions.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// peripherals structures
I2C_Manager hi2c2_mgr;

PCF8574_HandleTypeDef expander1;
PCF8574_HandleTypeDef expander2;

ADC121S021_HandleTypeDef spi_adcs[SPI_ADC_COUNT];

MCP4552_HandleTypeDef pot1;
MCP4552_HandleTypeDef pot2;
MCP4552_HandleTypeDef pot3;
MCP4552_HandleTypeDef pot4;

PotChannel pots[4] = {
		{&pot1, 128, 128, 2},
		{&pot2, 128, 128, 2},
		{&pot3, 128, 128, 2},
		{&pot4, 128, 128, 2}
};

ServoControllerState servos = {
    .target_mask = 0,
    .current_mask = 0,
    .done = 2
};

VoltageOutputsController vouts = {
    .current_mask = 0,
    .target_mask = 0,
    .done = 2
};

// buffers and variables for SPI ADCs
uint16_t spi_adc_samples[SPI_ADC_COUNT][SPI_ADC_AVG_SAMPLES] = {0};
uint8_t spi_adc_sample_index[SPI_ADC_COUNT] = {0};
float spi_adc_avg_values[SPI_ADC_COUNT] = {0.0f};

volatile uint8_t spi_current_adc = 0;
volatile uint8_t spi_adc_conversion_in_progress = 0;

// buffers and variables for STM ADCs
uint16_t adc1_buf[ADC1_CHANNELS] = {0};
uint16_t adc3_buf[ADC3_CHANNELS] = {0};

uint16_t adc1_samples[ADC1_CHANNELS][ADC_AVG_SAMPLES] = {0};
uint16_t adc3_samples[ADC3_CHANNELS][ADC_AVG_SAMPLES] = {0};

uint8_t adc1_index = 0;
uint8_t adc3_index = 0;

uint8_t adc1_done_samples = 0;
uint8_t adc3_done_samples = 0;

uint8_t measured_temperature = 0;

// measured values
float U_converter1 = 0.0f;
float U_converter2 = 0.0f;
float U_converter3 = 0.0f;
float U_converter4 = 0.0f;
float U_converter5 = 0.0f;
float I_supply = 0.0f;
float U_supply = 0.0f;
float U_bat_ADC = 0.0f;

float servo_currents[18] = {0.0f};
float I_manip = 0.0f;
float I_5V_pow = 0.0f;
float I_3V3_pow = 0.0f;
float I_standby = 0.0f;

float temperatures[3] = {0.0f};

// flags etc
uint8_t conv1_oc_status = 0;
uint8_t conv2_oc_status = 0;
uint8_t conv3_oc_status = 0;
uint8_t conv4_oc_status = 0;
uint8_t conv5_oc_status = 0;

uint8_t manip_conv_en = 0;
uint8_t manip_conv_en_done = 2;

uint32_t last_tick_toggle_en_conv = 0;

volatile uint8_t system_in_sleep = 0;
volatile uint8_t wake_up_sequence_request = 0;
volatile uint8_t power_off_sequence_request = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void spi_adc_read_callback(void* user, HAL_StatusTypeDef status, uint16_t value);

static inline float adc_to_current(uint16_t adc_val, float scale) {
	return (float)adc_val * scale;
}


static inline float adc_to_voltage(uint16_t adc_val, float scale) {
	return (float)adc_val * scale;
}


static inline float adc_to_temperature(uint16_t adc_val, float a, float b) {
	return (((float)adc_val/4095.0f)*3.3f - b)/a;
}


void Wakeup_From_Stop1(void) {
	HAL_ResumeTick();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_CAN1_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_SPI3_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
	MX_TIM17_Init();
	MX_TIM7_Init();

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

	system_in_sleep = 0;
	wake_up_sequence_request = 1;
}


void EnterStop1(void) {
	system_in_sleep = 1;
	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}


uint16_t converter_voltage_to_pot_value(float voltage, float R_FB1,
		float R_FB2, float R_FB3,
		float R_pot, float R_pot_offset) {
	float Re = R_FB2/((voltage/1.215f) - 1);
	float Rp = (R_FB1*R_FB3 - Re*(R_FB1 + R_FB3))/(Re - R_FB1) - R_pot_offset;
	float pot_value = Rp*256.0/R_pot;

	if(pot_value <= 0.0f) {
		pot_value = 0.0f;
	} else if (pot_value >= 256.0f) {
		pot_value = 256.0f;
	}
	return (uint16_t)pot_value;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	//  uint8_t pot_value = 1;
	uint8_t converters_en = 1;
	uint16_t main_iteration = 0;



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
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM17_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

	I2C_Manager_Init(&hi2c2_mgr, &hi2c2);

	power_on_seqence();

//	if(CAN_App_Init(CAN_ID_MIN, CAN_ID_MAX) != HAL_OK) {
//		Error_Handler();
//	}

	// Go to standby mode
	EnterStop1();



	//	Buzzer
	//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500); // 50% duty cycle



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
		if(power_off_sequence_request == 1) {
			power_off_sequence();
			power_off_sequence_request = 0;
			EnterStop1();
		}

		if(wake_up_sequence_request == 1) {
			wake_up_sequence();
			wake_up_sequence_request = 0;
		}

		HAL_GPIO_TogglePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin);
		HAL_GPIO_TogglePin(LED_STATUS2_GPIO_Port, LED_STATUS2_Pin);
		HAL_GPIO_TogglePin(LED_STATUS3_GPIO_Port, LED_STATUS3_Pin);
		HAL_GPIO_TogglePin(LED_STATUS4_GPIO_Port, LED_STATUS4_Pin);

		if(conv1_oc_status == 1) {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR1_GPIO_Port, LED_BAT_INDICATOR1_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR1_GPIO_Port, LED_BAT_INDICATOR1_Pin, GPIO_PIN_RESET);
		}
		if(conv2_oc_status == 1) {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR2_GPIO_Port, LED_BAT_INDICATOR2_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR2_GPIO_Port, LED_BAT_INDICATOR2_Pin, GPIO_PIN_RESET);
		}
		if(conv3_oc_status == 1) {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR3_GPIO_Port, LED_BAT_INDICATOR3_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR3_GPIO_Port, LED_BAT_INDICATOR3_Pin, GPIO_PIN_RESET);
		}
		if(conv4_oc_status == 1) {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR4_GPIO_Port, LED_BAT_INDICATOR4_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR4_GPIO_Port, LED_BAT_INDICATOR4_Pin, GPIO_PIN_RESET);
		}
		if(conv5_oc_status == 1) {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR5_GPIO_Port, LED_BAT_INDICATOR5_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_BAT_INDICATOR5_GPIO_Port, LED_BAT_INDICATOR5_Pin, GPIO_PIN_RESET);
		}




		if(HAL_GPIO_ReadPin(SW_FUNC_GPIO_Port, SW_FUNC_Pin) == GPIO_PIN_RESET) {
			HAL_Delay(20);
			if(converters_en == 0) {
				last_tick_toggle_en_conv = HAL_GetTick();
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV1_EN, GPIO_PIN_RESET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV2_EN, GPIO_PIN_RESET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV3_EN, GPIO_PIN_RESET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV4_EN, GPIO_PIN_RESET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV5_EN, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(LED_CONV1_GPIO_Port, LED_CONV1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_CONV2_GPIO_Port, LED_CONV2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_CONV3_GPIO_Port, LED_CONV3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_CONV4_GPIO_Port, LED_CONV4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_CONV5_GPIO_Port, LED_CONV5_Pin, GPIO_PIN_SET);
				converters_en = 1;
			} else {
				last_tick_toggle_en_conv = HAL_GetTick();
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV1_EN, GPIO_PIN_SET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV2_EN, GPIO_PIN_SET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV3_EN, GPIO_PIN_SET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV4_EN, GPIO_PIN_SET);
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV5_EN, GPIO_PIN_SET);

				HAL_GPIO_WritePin(LED_CONV1_GPIO_Port, LED_CONV1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_CONV2_GPIO_Port, LED_CONV2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_CONV3_GPIO_Port, LED_CONV3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_CONV4_GPIO_Port, LED_CONV4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_CONV5_GPIO_Port, LED_CONV5_Pin, GPIO_PIN_RESET);
				converters_en = 0;

				HAL_Delay(1);
//				EnterStop1();
			}
		}

		if(HAL_GPIO_ReadPin(SW_CLR_ERR_GPIO_Port, SW_CLR_ERR_Pin) == GPIO_PIN_RESET) {
			conv1_oc_status = 0;
			conv2_oc_status = 0;
			conv3_oc_status = 0;
			conv4_oc_status = 0;
			conv5_oc_status = 0;
		}


		// set manipulator converter state
		if(manip_conv_en_done == 0) {
			if(manip_conv_en == 1) {
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV4_EN, GPIO_PIN_RESET);
			} else {
				PCF7485_write_pin_blocking(&expander1, EXPANDER1_CONV4_EN, GPIO_PIN_SET);
			}
			manip_conv_en_done = 1;
		}


		Pots_Tick(); // setting voltages in converters
		Servos_Tick();
		VoltageOutputs_Tick();
		CAN_Logic_Tick();

		HAL_Delay(1);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == SW_POWER_ON_Pin) {
		if (system_in_sleep == 1) {
			Wakeup_From_Stop1();
		}
	}

	if(GPIO_Pin == SW_EMERGENCY_Pin) {
		if (system_in_sleep == 0) {
			power_off_sequence_request = 1;
		}
	}

	if(HAL_GetTick() - last_tick_toggle_en_conv > CONV_EN_OC_DELAY) {
		switch (GPIO_Pin) {
		case CONV1_OC_Pin:  // PB9
			conv1_oc_status = 1;
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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// timer for SPI ADCs conversions
	if(htim->Instance == TIM17) {
		if(!spi_adc_conversion_in_progress) {
			if(ADC121S021_read_start(&spi_adcs[spi_current_adc], spi_adc_read_callback,
					(void*)(uintptr_t)spi_current_adc) == HAL_OK) {
				spi_adc_conversion_in_progress = 1;
				spi_current_adc++;
				if(spi_current_adc >= SPI_ADC_COUNT) {
					spi_current_adc = 0;
				}
				return;
			}
		}
	}

	// timer for STM ADCs conversions
	if(htim->Instance == TIM7) {
		switch(measured_temperature) {
		case 0:
			HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_SET);
			break;
		}
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC1_CHANNELS);
	}

}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if(hadc->Instance == ADC1) {
		for(uint8_t ch = 0; ch < ADC1_CHANNELS; ch++)
			adc1_samples[ch][adc1_index] = adc1_buf[ch];

		adc1_index = (adc1_index + 1) % ADC_AVG_SAMPLES;
		adc1_done_samples++;

		if(adc1_done_samples == ADC_AVG_SAMPLES) {
			float avg[ADC1_CHANNELS] = {0};
			for (uint8_t ch = 0; ch < ADC1_CHANNELS; ch++) {
				uint32_t sum = 0;
				for (uint8_t i = 0; i < ADC_AVG_SAMPLES; i++) {
					sum += adc1_samples[ch][i];
				}
				avg[ch] = (float)sum / ADC_AVG_SAMPLES;
			}

			servo_currents[2]  = adc_to_current(avg[I_SERVO3_ADC1_rank],  I_SERVO_COEFF);
			servo_currents[3]  = adc_to_current(avg[I_SERVO4_ADC1_rank],  I_SERVO_COEFF);
			servo_currents[4]  = adc_to_current(avg[I_SERVO5_ADC1_rank],  I_SERVO_COEFF);
			servo_currents[5]  = adc_to_current(avg[I_SERVO6_ADC1_rank],  I_SERVO_COEFF);
			servo_currents[6]  = adc_to_current(avg[I_SERVO7_ADC1_rank],  I_SERVO_COEFF);
			servo_currents[7]  = adc_to_current(avg[I_SERVO8_ADC1_rank],  I_SERVO_COEFF);
			servo_currents[8]  = adc_to_current(avg[I_SERVO9_ADC1_rank],  I_SERVO_COEFF);
			servo_currents[9]  = adc_to_current(avg[I_SERVO10_ADC1_rank], I_SERVO_COEFF);
			servo_currents[10] = adc_to_current(avg[I_SERVO11_ADC1_rank], I_SERVO_COEFF);
			servo_currents[11] = adc_to_current(avg[I_SERVO12_ADC1_rank], I_SERVO_COEFF);
			servo_currents[12] = adc_to_current(avg[I_SERVO13_ADC1_rank], I_SERVO_COEFF);
			servo_currents[13] = adc_to_current(avg[I_SERVO14_ADC1_rank], I_SERVO_COEFF);
			servo_currents[14] = adc_to_current(avg[I_SERVO15_ADC1_rank], I_SERVO_COEFF);
			servo_currents[15] = adc_to_current(avg[I_SERVO16_ADC1_rank], I_SERVO_COEFF);
			servo_currents[16] = adc_to_current(avg[I_SERVO17_ADC1_rank], I_SERVO_COEFF);
			servo_currents[17] = adc_to_current(avg[I_SERVO18_ADC1_rank], I_SERVO_COEFF);

			adc1_done_samples = 0;
		}

		HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, ADC3_CHANNELS);

	} else if (hadc->Instance == ADC3) {
		for (uint8_t ch = 0; ch < ADC3_CHANNELS; ch++) {
			adc3_samples[ch][adc3_index] = adc3_buf[ch];
		}

		adc3_index = (adc3_index + 1) % ADC_AVG_SAMPLES;
		adc3_done_samples++;

		if(adc3_done_samples == ADC_AVG_SAMPLES) {
			float avg[ADC3_CHANNELS] = {0};
			for (uint8_t ch = 0; ch < ADC3_CHANNELS; ch++) {
				uint32_t sum = 0;
				for (uint8_t i = 0; i < ADC_AVG_SAMPLES; i++) {
					sum += adc3_samples[ch][i];
				}
				avg[ch] = (float)sum / ADC_AVG_SAMPLES;
			}

			I_manip  	  	  = adc_to_current(avg[I_MANIP_SENSE_ADC3_rank], I_MANIP_COEFF);
			I_5V_pow    	  = adc_to_current(avg[I_5V_POW_SENSE_ADC3_rank], I_5V_POW_COEFF);
			I_3V3_pow    	  = adc_to_current(avg[I_3V3_POW_SENSE_ADC3_rank], I_3V3_POW_COEFF);
			I_standby		  = adc_to_current(avg[I_STANDBY_SENSE_ADC3_rank], I_STANDBY_COEFF);
			servo_currents[0] = adc_to_current(avg[I_SERVO1_ADC3_rank], I_SERVO_COEFF);
			servo_currents[1] = adc_to_current(avg[I_SERVO2_ADC3_rank], I_SERVO_COEFF);

			temperatures[measured_temperature] = adc_to_temperature(avg[U_TEMP_ADC3_rank],
					TEMP_SENSOR_COEFF_A,
					TEMP_SENSOR_COEFF_B);
			adc3_done_samples = 0;

			measured_temperature++;
			if(measured_temperature >= 3) {
				measured_temperature = 0;
			}
		}
	}
}


void spi_adc_read_callback(void* user, HAL_StatusTypeDef status, uint16_t value) {
	uint8_t adc_index = (uint8_t)(uintptr_t)user;

	if(status == HAL_OK) {
		spi_adc_samples[adc_index][spi_adc_sample_index[adc_index]] = value;

		spi_adc_sample_index[adc_index]++;
		if(spi_adc_sample_index[adc_index] >= SPI_ADC_AVG_SAMPLES) {
			spi_adc_sample_index[adc_index] = 0;

			uint32_t sum = 0;
			for(uint8_t i = 0; i < SPI_ADC_AVG_SAMPLES; i++)
				sum += spi_adc_samples[adc_index][i];
			spi_adc_avg_values[adc_index] = (float)sum / SPI_ADC_AVG_SAMPLES;

			if(adc_index >= SPI_ADC_COUNT - 1) {
				U_converter1 = adc_to_voltage(spi_adc_avg_values[ADC1_CS_PIN], U_CONVERTER1_COEFF);
				U_converter2 = adc_to_voltage(spi_adc_avg_values[ADC2_CS_PIN], U_CONVERTER2_COEFF);
				U_converter3 = adc_to_voltage(spi_adc_avg_values[ADC3_CS_PIN], U_CONVERTER3_COEFF);
				U_converter4 = adc_to_voltage(spi_adc_avg_values[ADC4_CS_PIN], U_CONVERTER4_COEFF);
				U_converter5 = adc_to_voltage(spi_adc_avg_values[ADC5_CS_PIN], U_CONVERTER5_COEFF);
				I_supply 	 = adc_to_voltage(spi_adc_avg_values[ADC_I_SUPPLY_CS_PIN], I_SUPPLY_COEFF);
				U_supply	 = adc_to_voltage(spi_adc_avg_values[ADC_U_SUPPLY_CS_PIN], U_SUPPLY_COEFF);
				U_bat_ADC	 = adc_to_voltage(spi_adc_avg_values[ADC_U_BAT_CS_PIN], U_BAT_ADC_COEFF);
			}
		}
	}
	spi_adc_conversion_in_progress = 0;
}



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
