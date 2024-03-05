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
//#include "SX1278.h"
#include "LoRa.h"
#include "string.h"
#include "stdio.h"
#include "ds18b20driver.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t DataIn[20];  
uint8_t DataOut[10];  
LoRa myLoRa;
#define true 1
#define false 0

#define LEDThresh 0x00
#define ACK 0x01
#define OTAA 0x02
#define RxTimeout 90

DS18B20_Sensor sensor1;

float temperature;
uint16_t Thresh1 = 50;
uint16_t Thresh2 = 100;

#define NodeAddress 0x02
#define GatewayAddress 0xFF
#define Broadcast 0x00

uint8_t processdata = 0;
uint8_t ack = 0;
uint32_t MediumClear = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1) <= time);
}

void Send_Temp(){
	char string[6];

	for (uint8_t i = 2; i < 10; i++)
		DataOut[i] = 0;
	
	sprintf(string, "%.1f", temperature);
	DataOut[0] = NodeAddress;
	DataOut[1] = GatewayAddress;
	strcat(DataOut, string);
	LoRa_transmit(&myLoRa, DataOut, strlen(DataOut), 10);
}

void OTAA_Req(){
	DataOut[0] = NodeAddress;
	DataOut[1] = GatewayAddress;
	DataOut[2] = OTAA;
	LoRa_transmit(&myLoRa, DataOut, 20, 10);	
}

void Command_Execute(){
	switch(DataIn[2] - '0'){
		case LEDThresh:
			Thresh1 = DataIn[3] * 10 + DataIn[4] - '0' * 11;
			Thresh2 = DataIn[5] * 10 + DataIn[6] - '0' * 11;
			break;
		case ACK:
			ack = true;
		default: 
			break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	if(GPIO_Pin == DIO0_Pin){
	//	processdata = 1;

		uint8_t received_length = LoRa_receive(&myLoRa, DataIn, 20);
	//	return;
		if(received_length != 0){
			if(DataIn[0] != GatewayAddress)
				MediumClear += RxTimeout;
			else if(DataIn[1] != NodeAddress && DataIn[1] != Broadcast)
				MediumClear += 30;
			else Command_Execute();
		}
	}
}

void LBT(){
	do{
		while(HAL_GetTick()< MediumClear){}
		MediumClear = 0;
		LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);

	}while(MediumClear != 0);
}

uint16_t j = 0;
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1); 

	sensor1.GPIOx = DS18B20_GPIO_Port;
	sensor1.GPIO_Pin = DS18B20_Pin;
	
	DS18B20_ReadROM(&sensor1);
	DS18B20_SetResolution(sensor1, Resolution9Bit);

	myLoRa = newLoRa();
	
	myLoRa.hSPIx                 = &hspi1;
	myLoRa.CS_port               = NSS_GPIO_Port;
	myLoRa.CS_pin                = NSS_Pin;
	myLoRa.reset_port            = RST_GPIO_Port;
	myLoRa.reset_pin             = RST_Pin;
	myLoRa.DIO0_port						 = DIO0_GPIO_Port;
	myLoRa.DIO0_pin							 = DIO0_Pin;
	
	myLoRa.frequency             = 433;							  // default = 433 MHz
	myLoRa.spredingFactor        = SF_7;							// default = SF_7
	myLoRa.bandWidth			       = BW_125KHz;				  // default = BW_125KHz
	myLoRa.crcRate				       = CR_4_5;						// default = CR_4_5
	myLoRa.power					       = POWER_20db;				// default = 20db
	myLoRa.overCurrentProtection = 120; 							// default = 100 mA
	myLoRa.preamble				       = 8;		  					// default = 8;
	
	j = LoRa_init(&myLoRa);

	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_Y_Pin, 1);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_R_Pin, 1);	
	HAL_Delay(1000);

	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_Y_Pin, 0);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_R_Pin, 0);	
	
/*	LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);
	processdata = 1;
	while(1){
		if(processdata == 1){
			LoRa_transmit(&myLoRa, "H", 1, 10);
			processdata = 0;
		}
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	}
	*/
/*	do{
		LBT();
		OTAA_Req();
		delay_us(RxDelay_us);
		LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);
		HAL_Delay(RxTimeout);
		delay_us(RxDelay_us);
	}	while(ack != true);
	ack = false;
*/	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*			LoRa_transmit(&myLoRa, "Hello", 6, 1000);
			LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
			HAL_Delay(300);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
			HAL_Delay(300);	
			DS18B20_GetTemp(sensor1, &temperature);
			Send_Temp();
*/	
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_Y_Pin, 0);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_R_Pin, 0);	
		LoRa_gotoMode(&myLoRa, SLEEP_MODE);
		
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
		
		LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);
		DS18B20_GetTemp(sensor1, &temperature);
		
		if(temperature<Thresh1){
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_Y_Pin, 0);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_R_Pin, 0);	
		}
		else if(temperature<Thresh2){
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_Y_Pin, 1);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_R_Pin, 0);	
		}
		else{
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_Y_Pin, 0);
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_R_Pin, 1);
		}
	
			LBT();
			Send_Temp();
			LoRa_gotoMode(&myLoRa, RXCONTIN_MODE);
			HAL_Delay(RxTimeout);
		

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_R_Pin|LED_Y_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_Y_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_Y_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin RST_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
