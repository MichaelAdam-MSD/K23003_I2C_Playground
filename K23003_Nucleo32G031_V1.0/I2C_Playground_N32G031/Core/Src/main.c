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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug_print.h"
#include "lm75.h"
#include "tca9534.h"
#include "mcp3021.h"
#include "mcp4716.h"
#include "eeprom.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define EEPROM_INIT_BYTE			0xA0

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t clk_tick_10ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 10ms Tick
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim17.Instance)
	{
		clk_tick_10ms = 0;
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
	uint8_t address;
	uint8_t error;
	uint8_t led_p0 = 0;
	uint8_t led_p1 = 0;
	uint8_t led_p2 = 0;
	uint8_t led_p3 = 0;

	uint8_t lm75_ready = 0;
	uint8_t mcp3021_ready = 0;
	uint8_t mcp4716_ready = 0;
	uint8_t tca9534_ready = 0;

	uint16_t eeprom_cycles = 0 ;

	float temperature;

	uint16_t adc_value;
	int16_t temperature_dec;

	uint64_t tick_counter = 0;
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
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */

	//Delay
	HAL_Delay(100);

	//vt52 Cursor home
	debug_print("\e[H");

	//vt52 Clear Screen
	debug_print("\e[J");

	//vt52 Turn OFF cursor
	debug_print("\x1b[?25l");

	debug_print("K23003 I2C Playground\n\r");

	for (address = 0; address < 127; address++)
	{
		if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t) (address << 1), 3, 100) == HAL_OK)
		{
			debug_print("\n\rI2C Address found: 0x%X", address);
		}
	}

	debug_print("\e[11;0H");
	debug_print("LM75 Temperature °C: ");
	debug_print("\e[12;0H");
	debug_print("ADC Value (Counts) : ");
	debug_print("\e[13;0H");
	debug_print("EEPROM Boot Cycles : ");
	debug_print("\e[15;0H");
	debug_print("Content from the EEPROM in the socket:");

	if (LM75_Init(LM75_ADDR_0) == SUCCESS)
	{
		lm75_ready = 1;
	}

	if (TCA9534_Init(TCA9534A_ADDR_0, 0b11110000) == SUCCESS)
	{
		tca9534_ready = 1;
	}

	if (MCP4716_Init(MCP4716_ADDR_0) == SUCCESS)
	{
		mcp4716_ready = 1;
	}

	if (MCP3021_Init(MCP3021_ADDR) == SUCCESS)
	{
		mcp3021_ready = 1;
	}

	if (M24C0X_Init(M24C02_ADDR_0) == SUCCESS)
	{
		if (M24C0X_Read_Byte(M24C02_ADDR_0, 0x00) != EEPROM_INIT_BYTE)
		{
			for (uint8_t i = 1; i < 0xFF; i++)
			{
				M24C0X_Write_Byte(M24C02_ADDR_0, i, 0x00);
				HAL_Delay(10);
			}
			M24C0X_Write_Byte(M24C02_ADDR_0, 0, EEPROM_INIT_BYTE);
		}
		eeprom_cycles = M24C0X_Read_Word(M24C02_ADDR_0, 0x01);
		eeprom_cycles++;
		HAL_Delay(10);
		M24C0X_Write_Word(M24C02_ADDR_0, 0x01, eeprom_cycles);
		debug_print("\e[13;22H");
		debug_print("%d   ", eeprom_cycles);
	}

	if (M24C0X_Init(M24C02_ADDR_1) == SUCCESS)
	{
		uint8_t byte_tmp;
		uint8_t offset = 0;
		uint8_t address = 0;

		debug_print("\e[16;0H");

		for(uint8_t x = 0; x < 32; x++)
		{
			for(uint8_t y = 0; y < 8; y++)
			{
				address = offset + y;
				byte_tmp = M24C0X_Read_Byte(M24C02_ADDR_1, address);
				debug_print("0x%X ", byte_tmp);
			}
			debug_print("\n\r");
			offset++;
		}
	}
	else
	{
		debug_print("\e[16;0H");
		debug_print("EEPROM not found");
	}

	HAL_TIM_Base_Start_IT(&htim17);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Stay here until the clk_tick_10ms flag has been set to 0
		while (clk_tick_10ms);
		clk_tick_10ms = 1;

		// Increment tick counter
		tick_counter++;

		// Poll reset pushbutton
		if (HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin) == 0)
		{
			HAL_Delay(100);
			NVIC_SystemReset();
		}

		// Readout and print temperature
		if (lm75_ready)
		{
			temperature = LM75_Poll_Temperature(LM75_ADDR_0);
			temperature_dec = temperature;
			debug_print("\e[11;22H");
			debug_print("%d  ", temperature_dec);
		}

		// Readout ADC
		if (mcp3021_ready)
		{
			adc_value = MCP3021_Read_ADC_Counts(MCP3021_ADDR);
			debug_print("\e[12;22H");
			debug_print("%d   ", adc_value);
		}

		// IO Expander
		if (tca9534_ready)
		{
			if (Read_INT() == 0)
			{
				if (Read_PB_P4() == 0)
				{
					led_p0 = !led_p0;

					Write_LED(led_p0, 0);
				}
				if (Read_PB_P5() == 0)
				{
					led_p1 = !led_p1;
					Write_LED(led_p1, 1);
				}
				if (Read_PB_P6() == 0)
				{
					led_p2 = !led_p2;
					Write_LED(led_p2, 2);
				}
				if (Read_PB_P7() == 0)
				{
					led_p3 = !led_p3;
					Write_LED(led_p3, 3);
				}
			}
		}

		// Set DAC to the ADC value
		if (mcp4716_ready)
		{
			MCP4716_Write_DAC(MCP4716_ADDR_0, adc_value);
		}

		// Toggle PA2 LED every 100ms
		if ((tick_counter % 10) == 0)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00C12166;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void)
{

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 100 - 1;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 6400 - 1;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : WP_Pin */
	GPIO_InitStruct.Pin = WP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RST_Pin */
	GPIO_InitStruct.Pin = RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : INT_Pin */
	GPIO_InitStruct.Pin = INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OS_Pin */
	GPIO_InitStruct.Pin = OS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OS_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
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
