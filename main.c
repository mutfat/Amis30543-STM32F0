/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void USART2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM2_IRQHandler(void);
void UartSettingsApply(void);
uint8_t readReg(uint8_t address, uint8_t choice);
uint8_t writeReg(uint8_t adress, uint8_t value, uint8_t choice);
void selectChip(uint8_t choice);
void deselectChip(uint8_t choice);
void Step(uint8_t choice);
void StepStop(uint8_t choice);
void resetSettings(uint8_t choice);
void applySetting(uint8_t choice);
void enableDriver(uint8_t choice);
void disableDriver(uint8_t choice);
void setCurrentMilliamps(uint16_t current, uint8_t choice);
uint16_t readPosition(uint8_t choice);
void setDirection(uint8_t value, uint8_t choice);
uint8_t getDirection(uint8_t choice);
void setStepMode(uint8_t mode, uint8_t choice);
void sleep(uint8_t choice);
void sleepStop(uint8_t choice);
void stepOnRisingEdge(uint8_t choice);
void stepOnFallingEdge(uint8_t choice);
void setPwmFrequencyDouble(uint8_t choice);
void setPwmFrequencyDefault(uint8_t choice);
void setPwmJitterOn(uint8_t choice);
void setPwmJitterOff(uint8_t choice);
void setPwmSlope(uint8_t emc, uint8_t choice);
void setSlaGainDefault(uint8_t choice);
void setSlaGainHalf(uint8_t choice);
void setSlaTransparencyOff(uint8_t choice);
void setSlaTransparencyOn(uint8_t choice);
uint16_t readNonLatchedStatusFlags(uint8_t choice);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum stepMode
{
	MicroStep128 = 128,
	MicroStep64 = 64,
	MicroStep32 = 32,
	MicroStep16 = 16,
	MicroStep8 = 8,
	MicroStep4 = 4,
	MicroStep2 = 2,
	MicroStep1 = 1,
	CompensatedHalf = MicroStep2,
	CompensatedFullTwoPhaseOn = MicroStep1,
	CompensatedFullOnePhaseOn = 200,
	UncompensatedHalf = 201,
	UncompensatedFull = 202,
};
enum nonLatchedStatusFlag
{
	OPENY = (1 << 2),
	OPENX = (1 << 3),
	WD = (1 << 4),
	CPFAIL = (1 << 5),
	TW = (1 << 6),
};

/*! Bitmasks for the return value of readLatchedStatusFlagsAndClear(). */
enum latchedStatusFlag
{
	OVCXNB = (1 << 3),
	OVCXNT = (1 << 4),
	OVCXPB = (1 << 5),
	OVCXPT = (1 << 6),
	TSD = (1 << 10),
	OVCYNB = (1 << 11),
	OVCYNT = (1 << 12),
	OVCYPB = (1 << 13),
	OVCYPT = (1 << 14),
};

/*! Addresses of control and status registers. */
enum regAddr
{
	WR = 0x0,
	CR0 = 0x1,
	CR1 = 0x2,
	CR2 = 0x3,
	CR3 = 0x9,
	SR0 = 0x4,
	SR1 = 0x5,
	SR2 = 0x6,
	SR3 = 0x7,
	SR4 = 0xA,
};
uint8_t wr, cr0, cr1, cr2, cr3;
uint8_t wr_2, cr0_2, cr1_2, cr2_2, cr3_2;
uint8_t readed_data[4];
uint8_t readed_data2[4];
uint8_t readed_status[4];
uint8_t readed_status2[4];
uint16_t motor1_position;
uint16_t motor2_position;
uint16_t max_value;
uint8_t motor1_working;
uint8_t motor2_working;
uint16_t value_SLA2;
uint16_t value_SLA1;
uint16_t step_mode;
uint16_t step_mode2;
uint16_t ADC_ReadedData_DMA[2];
uint32_t step_number;
uint32_t step_number2;
uint32_t for_i;
uint32_t for_i_2;
uint16_t receive_uart[20];
uint8_t transmit_uart[10];
uint16_t PWM_period;
uint16_t PWM_period2;
uint16_t motor_choice;
uint16_t motor_choice2;
uint16_t sleep_mode_on;
uint16_t sleep_mode_on2;
uint16_t SLA_stop_value;
uint16_t SLA_stop_value2;
uint16_t SLA_trans_on;
uint16_t SLA_trans_on2;
uint16_t run_motor;
uint16_t run_motor2;
uint16_t motor_current;
uint16_t motor_current2;
uint16_t motor_direction;
uint16_t motor_direction2;
uint16_t motor_deneme = 1;
uint8_t uart_callback_bit;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	wr = cr0 = cr1 = cr2 = cr3 = 0;
	wr_2 = cr0_2 = cr1_2 = cr2_2 = cr3_2 = 0;
	max_value = 0;

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
	MX_SPI2_Init();
	MX_SPI1_Init();
	MX_ADC_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Init(&hspi2);
	resetSettings(0);
	resetSettings(1);
	HAL_ADC_Start_DMA(&hadc, (uint32_t *) ADC_ReadedData_DMA, 2);
	HAL_UART_Receive_DMA(&huart1, (uint8_t *) receive_uart, 40);
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start_IT(&htim3);
	// Disable driver in order to disconnect  current
	disableDriver(0);
	disableDriver(1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if (uart_callback_bit == 1)
		{
			HAL_TIM_Base_Stop_IT(&htim14);
			HAL_TIM_Base_Stop_IT(&htim3);
			UartSettingsApply();
			uart_callback_bit = 0;
			HAL_TIM_Base_Start_IT(&htim14);
			HAL_TIM_Base_Start_IT(&htim3);
		}
		/*if (for_i == step_number)
		 {
		 HAL_TIM_Base_Stop_IT(&htim3);
		 for_i = 0;
		 if (motor_deneme == 1)
		 motor_deneme = 0;
		 else
		 motor_deneme = 1;
		 setDirection(motor_deneme, motor_choice);
		 HAL_Delay(1000);
		 HAL_GPIO_TogglePin(GPIOC, LD3_Pin);
		 HAL_TIM_Base_Start_IT(&htim3);
		 }*/
		//motor2_position = readPosition(1);
		if (readPosition(1) % 128 == 0) //128-256-384
		{
			//HAL_ADC_Start_DMA(&hadc, (uint32_t *) ADC_ReadedData_DMA, 2);
			value_SLA2 = ADC_ReadedData_DMA[0]; //sla adc channel 10- sla2 adc channel 8 so dma reads first sla2
			//HAL_ADC_Stop_DMA(&hadc);
		}
		//motor1_position = readPosition(0);
		if (readPosition(0) % 128 == 0) //128-256-384
		{
			//HAL_ADC_Start_DMA(&hadc, (uint32_t *) ADC_ReadedData_DMA, 2);
			value_SLA1 = ADC_ReadedData_DMA[1]; //sla adc channel 10- sla2 adc channel 8 so dma reads first sla2
			//HAL_ADC_Stop_DMA(&hadc);
		}
		if (value_SLA1 < SLA_stop_value && value_SLA1 > 2) // motor1 3370
		{
			for_i = step_number;
			motor1_working = 0;
			disableDriver(0);
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, RESET);
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, SET);
			//HAL_ADC_Start_DMA(&hadc, (uint32_t *) ADC_ReadedData_DMA, 2);
			value_SLA1 = 0;
		}
		if (value_SLA2 < SLA_stop_value2 && value_SLA2 > 2) // motor 2
		{
			for_i_2 = step_number;
			motor2_working = 0;
			disableDriver(1);
			HAL_GPIO_WritePin(GPIOC, LD4_Pin, RESET);
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, SET);
			//HAL_ADC_Start_DMA(&hadc, (uint32_t *) ADC_ReadedData_DMA, 2);
			value_SLA2 = 0;
		}
		if (receive_uart[6] == 1 || motor1_working == 0 || motor2_working == 0)
		{
			if (run_motor == 1 && run_motor2 == 0)
				motor2_working = 1;
			else if (run_motor == 1 && run_motor2 == 0)
				motor1_working = 1;
			else if (run_motor == 0 && run_motor2 == 0)
			{
				motor1_working = 1;
				motor2_working = 1;
			}
			readed_data2[2] = readReg(CR2, 1);
			readed_data2[0] = readReg(CR0, 1);
			readed_data2[1] = readReg(CR1, 1);
			readed_data2[3] = readReg(CR3, 1);
			// READ STATUS REGISTER DATA
			readed_status2[2] = readReg(SR2, 1);
			readed_status2[0] = readReg(SR0, 1);
			readed_status2[1] = readReg(SR1, 1);
			readed_status2[3] = readReg(SR3, 1);
			readed_data[2] = readReg(CR2, 0);
			readed_data[0] = readReg(CR0, 0);
			readed_data[1] = readReg(CR1, 0);
			readed_data[3] = readReg(CR3, 0);
			// READ STATUS REGISTER DATA
			readed_status[2] = readReg(SR2, 0);
			readed_status[0] = readReg(SR0, 0);
			readed_status[1] = readReg(SR1, 0);
			readed_status[3] = readReg(SR3, 0);
			//Transmit amis-30543 control and status registers.
			transmit_uart[0] = motor1_working;
			transmit_uart[1] = motor2_working;
			transmit_uart[2] = readed_data[0];
			transmit_uart[3] = readed_data[1];
			transmit_uart[4] = readed_data[2];
			transmit_uart[5] = readed_data[3];
			transmit_uart[6] = readed_data2[0];
			transmit_uart[7] = readed_data2[1];
			transmit_uart[8] = readed_data2[2];
			transmit_uart[9] = readed_data2[3];
			HAL_UART_Transmit_IT(&huart1, &transmit_uart, 10);
			receive_uart[6] = 0;
			motor1_working = 1;
			motor2_working = 1;
		}
		if (for_i_2 == step_number2)
			disableDriver(1);
		if (for_i == step_number)
			disableDriver(0);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 47;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 5;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void)
{

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 47;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 20;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 38400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	//HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, CLR2_Pin | LD4_Pin | LD3_Pin | NXT_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(_CS2_GPIO_Port, _CS2_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DIR2_Pin | NXT2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(_CS_GPIO_Port, _CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CLR_Pin | _POR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SLA_Pin */
	GPIO_InitStruct.Pin = SLA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SLA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SLA2_Pin */
	GPIO_InitStruct.Pin = SLA2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SLA2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CLR2_Pin _CS2_Pin LD4_Pin LD3_Pin
	 NXT_Pin */
	GPIO_InitStruct.Pin = CLR2_Pin | _CS2_Pin | LD4_Pin | LD3_Pin | NXT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : _ERR2_Pin */
	GPIO_InitStruct.Pin = _ERR2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(_ERR2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR2_Pin NXT2_Pin */
	GPIO_InitStruct.Pin = DIR2_Pin | NXT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : DIR_Pin */
	GPIO_InitStruct.Pin = DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : _CS_Pin CLR_Pin _POR_Pin */
	GPIO_InitStruct.Pin = _CS_Pin | CLR_Pin | _POR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : _ERR_Pin */
	GPIO_InitStruct.Pin = _ERR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(_ERR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//-----------------------------------------------------*****************************************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	uart_callback_bit = 1;
}
void TIM14_IRQHandler(void)
{
	/* USER CODE BEGIN TIM2_IRQn 0 */

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim14);
	/* USER CODE BEGIN TIM2_IRQn 1 */
	if (for_i_2 < step_number)
	{
		HAL_GPIO_TogglePin(NXT2_GPIO_Port, NXT2_Pin); // motor1 NXT pin toggle(PWM);
	}
	if (for_i_2 != step_number2)
	{
		for_i_2 = for_i_2 + 1;
	}
	/* USER CODE END TIM2_IRQn 1 */
}
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */
	if (for_i < step_number)
	{
		HAL_GPIO_TogglePin(NXT_GPIO_Port, NXT_Pin); // motor1 NXT pin toggle(PWM)
	}
	if (for_i != step_number)
	{
		for_i = for_i + 1;
	}
	/* USER CODE END TIM3_IRQn 1 */
}

// Reads the register at the given address and returns its raw value from the chosen amis.
void UartSettingsApply()
{
	HAL_GPIO_WritePin(GPIOC, LD4_Pin, SET);
	HAL_GPIO_WritePin(GPIOC, LD3_Pin, RESET);
	// Variables
	//Motor1
	step_mode = receive_uart[0];
	motor_current = receive_uart[1];
	PWM_period = receive_uart[2];
	SLA_stop_value = receive_uart[3];
	SLA_trans_on = receive_uart[4];
	motor_direction = receive_uart[5];

	sleep_mode_on = receive_uart[7];
	motor_choice = receive_uart[8];	//not necessary
	run_motor = receive_uart[9];
	// Motor2
	step_mode2 = receive_uart[10];
	motor_current2 = receive_uart[11];
	PWM_period2 = receive_uart[12];
	SLA_stop_value2 = receive_uart[13];
	SLA_trans_on2 = receive_uart[14];
	motor_direction2 = receive_uart[15];

	sleep_mode_on2 = receive_uart[17];
	motor_choice2 = receive_uart[18];	//not necessary
	run_motor2 = receive_uart[19];
	// After choice value got
	/////////////////////////////// Motor1 Settings
	resetSettings(0);
	setStepMode(step_mode, 0);
	setCurrentMilliamps(motor_current, 0);
	setDirection(motor_direction, 0);
	//sleep-mode
	if (sleep_mode_on == 1)
		sleep(0);
	else
		sleepStop(0);
	//sla-trans
	if (SLA_trans_on == 1)
		setSlaTransparencyOn(0);
	else
		setSlaTransparencyOff(0);
	//run-motor
	if (run_motor == 1)
		enableDriver(0);
	else
		disableDriver(0);
	/////////////////////////////// Motor2 Settings
	resetSettings(1);
	setStepMode(step_mode2, 1);
	setCurrentMilliamps(motor_current2, 1);
	setDirection(motor_direction2, 1);
	//sleep-mode
	if (sleep_mode_on2 == 1)
		sleep(1);
	else
		sleepStop(1);
	//sla-trans
	if (SLA_trans_on2 == 1)
		setSlaTransparencyOn(1);
	else
		setSlaTransparencyOff(1);
	//run-motor
	if (run_motor2 == 1)
		enableDriver(1);
	else
		disableDriver(1);
	//Timer Settings
	motor1_working = 1;
	motor2_working = 1;
	for_i = 0;
	for_i_2 = 0;
	step_number = PWM_period * ((step_mode * 2) * 320); // microstep_number * 2 * 320
	step_number2 = PWM_period2 * ((step_mode * 2) * 320); // microstep_number * 2 * 320
}
uint8_t readReg(uint8_t address, uint8_t choice)
{

	uint8_t RxBuf[2];
	uint8_t TxBuf[2];

	if (choice % 2 == 0)
	{
		selectChip(0);
		TxBuf[0] = address;

		HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
		HAL_SPI_Receive(&hspi1, RxBuf, 1, 50);
		deselectChip(0);

	}
	else
	{
		selectChip(1);
		TxBuf[0] = address;

		HAL_SPI_Transmit(&hspi2, TxBuf, 1, 50);
		HAL_SPI_Receive(&hspi2, RxBuf, 1, 50);
		deselectChip(1);

	}

	return RxBuf[0];
}

/*! Writes the specified value to a register to the chosen amis. */
uint8_t writeReg(uint8_t address, uint8_t value, uint8_t choice)
{
	uint8_t TxBuf[2];
	uint8_t RxBuf[2];
	if (choice % 2 == 0)
	{
		selectChip(0);
		TxBuf[0] = 0x80 | address;
		TxBuf[1] = value;
		HAL_SPI_Transmit(&hspi1, TxBuf, 2, 50);
		deselectChip(0);
		selectChip(0);
		HAL_SPI_Receive(&hspi1, RxBuf, 2, 50);
		deselectChip(0);

	}

	else
	{
		TxBuf[0] = 0x80 | address;
		TxBuf[1] = value;
		selectChip(1);
		HAL_SPI_Transmit(&hspi2, TxBuf, 2, 50);
		deselectChip(1);
		selectChip(1);
		HAL_SPI_Receive(&hspi2, RxBuf, 2, 50);
		deselectChip(1);
	}

	// The CS line must go high after writing for the value to actually take
	// effect.
	return RxBuf[1];
}

void selectChip(uint8_t choice) 	// Pull Down the selected amis CS Pin.
{
	if (choice == 0)
		HAL_GPIO_WritePin(_CS_GPIO_Port, _CS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(_CS2_GPIO_Port, _CS2_Pin, GPIO_PIN_RESET);

}

void deselectChip(uint8_t choice)	// Pull Up the selected amis CS Pin.
{
	if (choice == 0)
		HAL_GPIO_WritePin(_CS_GPIO_Port, _CS_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(_CS2_GPIO_Port, _CS2_Pin, GPIO_PIN_SET);

	// The CS high time is specified as 2.5 us in the AMIS-30543 datasheet.
	for (int i = 100; i > 0; i--)
	{

	}
}
void Step(uint8_t choice)
{
	if (choice % 2 == 0)
	{
		HAL_GPIO_TogglePin(NXT_GPIO_Port, NXT_Pin);
	}
	else
	{
		HAL_GPIO_TogglePin(NXT2_GPIO_Port, NXT2_Pin);
	}
}
void StepStop(uint8_t choice)
{
	if (choice % 2 == 0)
	{
		HAL_TIM_Base_Stop_IT(&htim3);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim14);
	}
}
void resetSettings(uint8_t choice)
{
	if (choice % 2 == 0)
		wr = cr0 = cr1 = cr2 = cr3 = 0;
	else
		wr_2 = cr0_2 = cr1_2 = cr2_2 = cr3_2 = 0;

	applySetting(choice);
}
void applySetting(uint8_t choice)
{
	if (choice % 2 == 0)
	{
		writeReg(CR2, cr2, choice);
		for (int i = 100; i > 0; i--)
		{
			;
		}
		writeReg(WR, wr, choice);
		writeReg(CR0, cr0, choice);
		writeReg(CR1, cr1, choice);
		writeReg(CR3, cr3, choice);
	}
	else
	{
		writeReg(CR2, cr2_2, choice);
		for (int i = 100; i > 0; i--)
		{
			;
		}
		writeReg(WR, wr_2, choice);
		writeReg(CR0, cr0_2, choice);
		writeReg(CR1, cr1_2, choice);
		writeReg(CR3, cr3_2, choice);
	}
}
void enableDriver(uint8_t choice) // Set MOTEN bit.
{
	if (choice % 2 == 0)
		cr2 |= 0b10000000;
	else
		cr2_2 |= 0b10000000;
	applySetting(choice);
}
void disableDriver(uint8_t choice) // Reset MOTEN bit.
{
	if (choice % 2 == 0)
		cr2 &= ~0b10000000;
	else
		cr2_2 &= ~0b10000000;
	applySetting(choice);
}
void setCurrentMilliamps(uint16_t current, uint8_t choice)
{
	// This comes from Table 13 of the AMIS-30543 datasheet.
	uint8_t code = 0;
	if (current >= 3000)
	{
		code = 0b11001;
	}
	else if (current >= 2845)
	{
		code = 0b11000;
	}
	else if (current >= 2700)
	{
		code = 0b10111;
	}
	else if (current >= 2440)
	{
		code = 0b10110;
	}
	else if (current >= 2240)
	{
		code = 0b10101;
	}
	else if (current >= 2070)
	{
		code = 0b10100;
	}
	else if (current >= 1850)
	{
		code = 0b10011;
	}
	else if (current >= 1695)
	{
		code = 0b10010;
	}
	else if (current >= 1520)
	{
		code = 0b10001;
	}
	else if (current >= 1405)
	{
		code = 0b10000;
	}
	else if (current >= 1260)
	{
		code = 0b01111;
	}
	else if (current >= 1150)
	{
		code = 0b01110;
	}
	else if (current >= 1060)
	{
		code = 0b01101;
	}
	else if (current >= 955)
	{
		code = 0b01100;
	}
	else if (current >= 870)
	{
		code = 0b01011;
	}
	else if (current >= 780)
	{
		code = 0b01010;
	}
	else if (current >= 715)
	{
		code = 0b01001;
	}
	else if (current >= 640)
	{
		code = 0b01000;
	}
	else if (current >= 585)
	{
		code = 0b00111;
	}
	else if (current >= 540)
	{
		code = 0b00110;
	}
	else if (current >= 485)
	{
		code = 0b00101;
	}
	else if (current >= 445)
	{
		code = 0b00100;
	}
	else if (current >= 395)
	{
		code = 0b00011;
	}
	else if (current >= 355)
	{
		code = 0b00010;
	}
	else if (current >= 245)
	{
		code = 0b00001;
	}
	if (choice % 2 == 0)
	{
		cr0 = (cr0 & 0b11100000) | code;
		writeReg(CR0, cr0, 0);
	}
	else
	{
		cr0_2 = (cr0_2 & 0b11100000) | code;
		writeReg(CR0, cr0_2, 0);
	}
}
uint16_t readPosition(uint8_t choice) // Motor position reading.
{
	uint8_t sr3;
	uint8_t sr4;
	if (choice % 2 == 0)
	{
		sr3 = readReg(SR3, 0) & 0x7F;
		sr4 = readReg(SR4, 0) & 0x7F;
	}
	else
	{
		sr3 = readReg(SR3, 1) & 0x7F;
		sr4 = readReg(SR4, 1) & 0x7F;
	}
	return ((uint16_t) sr3 << 2) | (sr4 & 3);
}
void setDirection(uint8_t value, uint8_t choice) // when( DIR = 1 & DIRCTRL = 1, DIR = 0 DIRCTRL = 0)CW
{
	if (choice % 2 == 0)
	{
		if (value % 2 == 0)
		{
			cr1 |= 0x80;
		}
		else
		{
			cr1 &= ~0x80;
		}
		writeReg(CR1, cr1, choice);
	}
	else
	{
		if (value % 2 == 0)
		{
			cr1_2 |= 0x80;
		}
		else
		{
			cr1_2 &= ~0x80;
		}
		writeReg(CR1, cr1_2, choice);
	}
}
uint8_t getDirection(uint8_t choice) // Read DIRCTRL bit from chosen amis..
{
	if (choice % 2 == 0)
		return cr1 >> 7 & 1;
	else
		return cr1_2 >> 7 & 1;
}
void setStepMode(uint8_t mode, uint8_t choice)
{
	// Pick 1/32 micro-step by default.
	uint8_t esm = 0b000;
	uint8_t sm = 0b000;

	// The order of these cases matches the order in Table 12 of the
	// AMIS-30543 datasheet.
	switch (mode)
	{
	case MicroStep32:
		sm = 0b000;
		break;
	case MicroStep16:
		sm = 0b001;
		break;
	case MicroStep8:
		sm = 0b010;
		break;
	case MicroStep4:
		sm = 0b011;
		break;
	case CompensatedHalf:
		sm = 0b100;
		break; 
	case UncompensatedHalf:
		sm = 0b101;
		break;
	case UncompensatedFull:
		sm = 0b110;
		break;
	case MicroStep128:
		esm = 0b001;
		break;
	case MicroStep64:
		esm = 0b010;
		break;
	case CompensatedFullTwoPhaseOn:
		esm = 0b011;
		break; 
	case CompensatedFullOnePhaseOn:
		esm = 0b100;
		break;
	}

	if (choice % 2 == 0)
	{
		cr0 = (cr0 & ~0b11100000) | (sm << 5);
		cr3 = (cr3 & ~0b111) | esm;
		writeReg(CR0, cr0, choice);
		writeReg(CR3, cr3, choice);
	}
	else
	{
		cr0_2 = (cr0 & ~0b11100000) | (sm << 5);
		cr3_2 = (cr3 & ~0b111) | esm;
		writeReg(CR0, cr0_2, choice);
		writeReg(CR3, cr3_2, choice);
	}
}
void sleep(uint8_t choice)
{
	if (choice % 2 == 0)
		cr2 |= (1 << 6);
	else
		cr2_2 |= (1 << 6);

	applySetting(choice);
}
void sleepStop(uint8_t choice)
{
	if (choice % 2 == 0)
		cr2 &= ~(1 << 6);
	else
		cr2_2 &= ~(1 << 6);
	applySetting(choice);
}
void stepOnRisingEdge(uint8_t choice)
{
	cr1 &= ~0b01000000;
	writeReg(CR1, cr1, choice);
}
void stepOnFallingEdge(uint8_t choice)
{
	cr1 |= 0b01000000;
	writeReg(CR1, cr1, choice);
}
void setPwmFrequencyDouble(uint8_t choice)
{
	cr1 |= (1 << 3);
	writeReg(CR1, cr1, choice);
}
void setPwmFrequencyDefault(uint8_t choice)
{
	cr1 &= ~(1 << 3);
	writeReg(CR1, cr1, choice);
}
void setPwmJitterOn(uint8_t choice)
{
	cr1 |= (1 << 2);
	writeReg(CR1, cr1, choice);
}
void setPwmJitterOff(uint8_t choice)
{
	cr1 &= ~(1 << 2);
	writeReg(CR1, cr1, choice);
}
void setPwmSlope(uint8_t emc, uint8_t choice)
{
	cr1 = (cr1 & ~0b11) | (emc & 0b11);
	writeReg(CR1, cr1, choice);
}
void setSlaGainDefault(uint8_t choice)
{
	if (choice % 2 == 0)
		cr2 &= ~(1 << 5);
	else
		cr2_2 &= ~(1 << 5);

	applySetting(choice);
}
void setSlaGainHalf(uint8_t choice)
{
	if (choice % 2 == 0)
		cr2 |= (1 << 5);
	else
		cr2_2 |= (1 << 5);
	applySetting(choice);
}
void setSlaTransparencyOff(uint8_t choice)
{
	if (choice % 2 == 0)
		cr2 &= ~(1 << 4);
	else
		cr2_2 &= ~(1 << 4);

	applySetting(choice);
}
void setSlaTransparencyOn(uint8_t choice)
{
	if (choice % 2 == 0)
		cr2 |= (1 << 4);
	else
		cr2_2 |= (1 << 4);

	applySetting(choice);
}
uint16_t readNonLatchedStatusFlags(uint8_t choice)
{
	return readReg(SR0, choice);
}
uint16_t readLatchedStatusFlagsAndClear(uint8_t choice)
{
	uint8_t sr1 = readReg(SR1, choice);
	uint8_t sr2 = readReg(SR2, choice);
	return (sr2 << 8) | sr1;
}
void setDirectionCW(uint8_t dir, uint8_t choice)
{
	// The NXT/STEP pin must not change for at least 0.5
	// microseconds before and after changing the DIR pin.
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dir % 2);
}

//-----------------------------------------------------*****************************************************

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
