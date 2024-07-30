/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with the software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Include standard integer definitions
#include <stdint.h>
// Include header for HD44780 LCD driver
#include "../../Drivers/Device_Drivers/HD44780_LCD/HD44780_LCD.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the maximum PWM value
#define MAX_PWM 2000
// Define the minimum PWM value
#define MIN_PWM 998
// Define the maximum angle value
#define MAX_ANGLE 90
// Define the minimum angle value
#define MIN_ANGLE -90
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Handle for Timer 1
TIM_HandleTypeDef htim1;
// Handle for Timer 2
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Initialize the LCD struct with GPIO pins
Alcd_t lcd =
		{ .RS_GPIO = GPIOA, .RS_GPIO_Pin = GPIO_PIN_4, .EN_GPIO = GPIOA,
				.EN_GPIO_Pin = GPIO_PIN_5, .Data_GPIO = GPIOA,
				.Data_GPIO_Start_Pin = 0 };
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	// Start the base timer for Timer 1
	HAL_TIM_Base_Start(&htim1);

	// Start the PWM generation for Timer 1 Channel 1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	// Initialize the LCD with 2 lines and 16 columns
	Alcd_Init(&lcd, 2, 16);

	// Start the encoder interface on Timer 2
	HAL_TIM_Encoder_Start(&htim2, TIM_ENCODERMODE_TI1);

	// Set initial PWM duty cycle to center value (neutral position = 0 angle)
	TIM1->CCR1 = 1499;

	char str[16];
	int16_t message;
	int16_t angle = 0;
	int16_t pwm_cycle;

	// Clear the LCD
	Alcd_Clear(&lcd);
	// Display initial messages on the LCD
	Alcd_PutAt(&lcd, 0, 0, "The angle range");
	Alcd_PutAt(&lcd, 1, 0, "is 90 and -90");
	HAL_Delay(2000);
	Alcd_Clear(&lcd);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// Read the current position of the encoder
		int32_t encoder_position = __HAL_TIM_GET_COUNTER(&htim2);

		// Update the angle with the encoder position
		angle = encoder_position;
		Alcd_Clear(&lcd);

		// Limit the angle to the maximum allowed value
		if (angle > MAX_ANGLE) {
			angle = MAX_ANGLE;
		}

		// Limit the angle to the minimum allowed value
		if (angle < MIN_ANGLE) {
			angle = MIN_ANGLE;
		}

		// Display the current angle on the LCD
		message = sprintf(str, "angle = %d", angle);
		Alcd_PutAt_n(&lcd, 0, 0, str, message);

		// Calculate the PWM duty cycle based on the angle
		pwm_cycle = (5.5555 * angle) + 1499;

		// Ensure the PWM duty cycle is within the allowed range
		if ((MIN_PWM < pwm_cycle) & (pwm_cycle < MAX_PWM)) {
			// Update the PWM duty cycle
			TIM1->CCR1 = pwm_cycle;

			// Display the current PWM duty cycle on the LCD
			message = sprintf(str, "duty = %d", pwm_cycle);
			Alcd_PutAt_n(&lcd, 1, 0, str, message);
		}

		// Delay for a short period
		HAL_Delay(50);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 71; // Set prescaler to 71
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP; // Set counter mode to up
	htim1.Init.Period = 19999; // Set period to 19999
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Set clock division to 1
	htim1.Init.RepetitionCounter = 0; // Set repetition counter to 0
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Disable auto-reload preload
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler(); // Handle error if initialization fails
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // Set clock source to internal
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler(); // Handle error if configuration fails
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler(); // Handle error if PWM initialization fails
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // Set master output trigger to reset
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // Disable master-slave mode
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(); // Handle error if synchronization configuration fails
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1; // Set output compare mode to PWM1
	sConfigOC.Pulse = 1000; // Set pulse value to 1000
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // Set output compare polarity to high
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // Set output compare polarity to high
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; // Disable output compare fast mode
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; // Set output compare idle state to reset
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // Set output compare idle state to reset
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler(); // Handle error if channel configuration fails
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE; // Disable off-state run mode
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE; // Disable off-state idle mode
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF; // Set lock level to off
	sBreakDeadTimeConfig.DeadTime = 0; // Set dead time to 0
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE; // Disable break state
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH; // Set break polarity to high
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; // Disable automatic output
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler(); // Handle error if break/dead-time configuration fails
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0; // Set prescaler to 0
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP; // Set counter mode to up
	htim2.Init.Period = 65535; // Set period to 65535
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Set clock division to 1
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Disable auto-reload preload
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1; // Set encoder mode to TI1
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING; // Set input capture polarity to rising edge
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI; // Set input capture selection to direct input
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1; // Set input capture prescaler to 1
	sConfig.IC1Filter = 0; // Set input capture filter to 0
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING; // Set input capture polarity to rising edge
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI; // Set input capture selection to direct input
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1; // Set input capture prescaler to 1
	sConfig.IC2Filter = 0; // Set input capture filter to 0
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler(); // Handle error if encoder initialization fails
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // Set master output trigger to reset
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // Disable master-slave mode
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(); // Handle error if synchronization configuration fails
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq(); // Disable interrupts
	while (1) {
		// Infinite loop to indicate error
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
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
