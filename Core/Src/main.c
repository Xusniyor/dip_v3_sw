/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "hrtim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC1_BUF_LEN 2
#define ADC2_BUF_LEN 5

/* Controller parameters */
#define PID_KP  0.12f
#define PID_KI  0.8f
#define PID_KD  0.003f
#define PID_TAU 0.02f
#define PID_LIM_MAX (float)(MAX_PWM_DUTY_CYCLE)
#define PID_LIM_MIN 0.0f
#define PID_LIM_MAX_INT (float)(MAX_PWM_DUTY_CYCLE)
#define PID_LIM_MIN_INT 0.0f
#define SAMPLE_TIME_S   0.00789f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t HRTIM_DMA_Buffer[1];
uint16_t ADC1_DMA_Buffer[ADC1_BUF_LEN];
uint16_t ADC2_DMA_Buffer[ADC2_BUF_LEN];
uint32_t pwm_duty_cycle;
uint32_t pwm_min_value;
uint32_t pwm_soft_start;
uint8_t  pwm_active;

/* Initialise PID controller */
PIDController pid = {
	PID_KP, PID_KI, PID_KD,
	PID_TAU,
	PID_LIM_MIN,     PID_LIM_MAX,
	PID_LIM_MIN_INT, PID_LIM_MAX_INT,
	SAMPLE_TIME_S
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc1) {
		if (pwm_active) {
			/* Get measurement from system */
			float measurement = (float)ADC1_DMA_Buffer[0];
			/* Simulate response using test system */
			float setpoint = fmap((float)ADC2_DMA_Buffer[0], 4095.0, 0.0, 1241.0, 3239.0);
			/* Compute new control signal */
			PIDController_Update(&pid, setpoint, measurement);

			pwm_min_value = pwm_soft_start;
			if (pwm_soft_start < MAX_PWM_DUTY_CYCLE)
				pwm_soft_start+=5;
			if (pwm_min_value > (uint32_t)(pid.out))
				pwm_min_value = (uint32_t)(pid.out);

			HRTIM_DMA_Buffer[0] = pwm_min_value;
		} else {
			pwm_soft_start = 0;
			HRTIM_DMA_Buffer[0] = 0;
		}
	} else if (hadc == &hadc2) {
		HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // Start HRTIM
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 + HRTIM_OUTPUT_TA2);
  HAL_HRTIM_WaveformCountStart_DMA(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  // Start ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_DMA_Buffer, ADC1_BUF_LEN);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2_DMA_Buffer, ADC2_BUF_LEN);

  HAL_Delay(1000);
  pwm_active = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/* USER CODE BEGIN 4 */

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
