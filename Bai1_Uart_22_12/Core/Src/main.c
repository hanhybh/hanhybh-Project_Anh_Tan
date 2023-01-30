/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"

#include <delay_timer.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId Task1_TranVolHandle;
osThreadId Task2_UARTHandle;
osThreadId Task3_PluseHandle;
/* USER CODE BEGIN PV */




/*************************KHAI BAO.Task1: Dieu khien dong co bang Relay****************************************/
uint8_t Tx0_buff[] = "Task1. Dieu khien dong co bang relay.\n";
uint8_t Rx_buff[30];
uint8_t Rx_data = 0;
/*************************KHAI BAO.Task2: Doc gia tri ADC****************************************/
uint8_t Tx1_buff[] = "Task2. Dung ADC doc gia tri dien ap.\n";
uint8_t V1_buff[MAX], V2_buff[MAX], Vref_buff[MAX];
float V[3];
float V1, V2, VDDA;

uint8_t PA0[MAX], PA1[MAX];
/*************************KHAI BAO.Test RTOS****************************************/
uint32_t cnt1 = 0, cnt2 = 0, cnt3 = 0;
/*****************************************************************************/


/*************************Khai bao ham****************************************/

void Pluse();    
void uart_json();

/*****************************************************************************************************/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
void Voltage(void const * argument);
void Tranfer_Receive(void const * argument);
void Pluse100(void const * argument);

/* USER CODE BEGIN PFP */
/************************Printf UART******************************************/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/************************UART. NGAT NHAN******************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  /*****************RELAY*****************/
	if(Rx_data == '1')
	{
		HAL_UART_Transmit(&huart1, (uint8_t *) "Relay 1 dong.\n", 15, 300);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	}
	if(Rx_data == '2')
	{
		HAL_UART_Transmit(&huart1, (uint8_t *) "Relay 2 dong.\n", 15, 300);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	}
	if(Rx_data == '3')
	{
		HAL_UART_Transmit(&huart1,(uint8_t *) "Relay 3 dong.\n", 15, 300);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	}
	if(Rx_data == 'a')
	{
		HAL_UART_Transmit(&huart1, (uint8_t *) "Relay 1 cat.\n", 14, 300);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	}
	if(Rx_data == 'b')
	{
		HAL_UART_Transmit(&huart1, (uint8_t *) "Relay 2 cat.\n", 14, 300);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	if(Rx_data == 'c')
	{
		HAL_UART_Transmit(&huart1,(uint8_t *) "Relay 3 cat.\n", 14, 300);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	if(Rx_data == '0')
	{
		HAL_UART_Transmit(&huart1, (uint8_t *) "Ca 3 Relay cung cat.\n", 21, 300);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	  /*****************PWM*****************/
	if(Rx_data == 'p')                                                             
	{
		HAL_UART_Transmit(&huart1, (uint8_t *) "Start\n", 6, 300);
		Pluse();
		HAL_UART_Transmit(&huart1, (uint8_t *) "End\n", 4, 300);
	}
	  /*****************ADC*****************/
	if(Rx_data == 'd')                                                             
	{
		uart_json();
	}
	
	HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
}
/*************************ADC. CH0 AND CH1****************************************/
void ADC_Select_CH0 (void)
{
 ADC_ChannelConfTypeDef sConfig = {0};
 sConfig.Channel = ADC_CHANNEL_0;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
 if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
}

void ADC_Select_CH1 (void)
{
 ADC_ChannelConfTypeDef sConfig = {0};
 sConfig.Channel = ADC_CHANNEL_1;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
 if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
}
void ReadVol_Vref()                                       
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 300);
	V[0] = HAL_ADC_GetValue(&hadc1);
  VDDA = 4095.0 * 1.212 / (float)V[0];
	HAL_ADC_Stop(&hadc1);
	
//	snprintf((char *) Vref_buff, 99, "Gia tri dien ap tai chan Vref: %.2f\n", VDDA);
}

void ReadVol_PA0()                                       
{
	ADC_Select_CH0();
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 300);
	V[1] = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
  V1 = V[1]*VDDA/4095;
	V1 = V1 *5/3.3;
	
//	snprintf((char *) V1_buff, 99, "Gia tri dien ap tai chan PA0(2.5 - 5V) la: %.2f\n", V1);
	snprintf((char *) V1_buff, 99, "%.2f", V1);

}
	
void ReadVol_PA1()                                       
{
	ADC_Select_CH1();
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 300);
	V[2] = HAL_ADC_GetValue(&hadc2);
  V2 = V[2]*VDDA/4095;
	HAL_ADC_Stop(&hadc2);

	snprintf((char *) V2_buff, 99, "%.2f", V2);
}

void TranVol_Vref()  
{
	ReadVol_Vref();
	HAL_UART_Transmit(&huart1, Vref_buff, sizeof(Vref_buff), 300);
}

void TranVol_PA0()                                       
{
	ReadVol_PA0();
	if(V1 < 3.0)
	{
		strcpy((char *)PA0, "Chan PA0 dang tha noi.");
	}
	else
	{
//	  HAL_UART_Transmit(&huart1, V1_buff, sizeof(V1_buff), 300);
		strcpy((char *)PA0,(char *)V1_buff);
	}
}


void TranVol_PA1()                                       
{
	ReadVol_PA1();
	if(V2 < 2.5)
	{
		strcpy((char *)PA1, "Chan PA1 dang tha noi.\n");
	}
	else
	{
//	  HAL_UART_Transmit(&huart1, V2_buff, sizeof(V2_buff), 300);
		strcpy((char *)PA1,(char *)V2_buff);
	}
}

void uart_json()
{
	TranVol_Vref();
	TranVol_PA0();
	TranVol_PA1();
	char *out;
	cJSON *root;
	root  = cJSON_CreateObject();

  cJSON_AddItemToObject(root, "PA0: ", cJSON_CreateString((char *) PA0));
  cJSON_AddItemToObject(root, "PA1: ", cJSON_CreateString((char *) PA1));
  
  out = cJSON_Print(root);
  printf("%s\n",out);
}
/*************************PWM. 100 PLUSE****************************************/

void Pluse()                                           
{
	 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 5000);
	Delayms(1000);
	
	 __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, Tx0_buff, sizeof(Tx0_buff), 300);
	HAL_UART_Transmit(&huart1, Tx1_buff, sizeof(Tx1_buff), 300);
  HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task1_TranVol */
  osThreadDef(Task1_TranVol, Voltage, osPriorityNormal, 0, 128);
  Task1_TranVolHandle = osThreadCreate(osThread(Task1_TranVol), NULL);

  /* definition and creation of Task2_UART */
  osThreadDef(Task2_UART, Tranfer_Receive, osPriorityIdle, 0, 128);
  Task2_UARTHandle = osThreadCreate(osThread(Task2_UART), NULL);

  /* definition and creation of Task3_Pluse */
  osThreadDef(Task3_Pluse, Pluse100, osPriorityIdle, 0, 128);
  Task3_PluseHandle = osThreadCreate(osThread(Task3_Pluse), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Voltage */
/**
  * @brief  Function implementing the Task1_TranVol thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Voltage */
void Voltage(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//		TranVol_Vref();
//		TranVol_PA0();     
//		TranVol_PA1();   
//    Delayms(5000);
		osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Tranfer_Receive */
/**
* @brief Function implementing the Task2_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tranfer_Receive */
void Tranfer_Receive(void const * argument)
{
  /* USER CODE BEGIN Tranfer_Receive */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }
  /* USER CODE END Tranfer_Receive */
}

/* USER CODE BEGIN Header_Pluse100 */
/**
* @brief Function implementing the Task3_Pluse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Pluse100 */
void Pluse100(void const * argument)
{
  /* USER CODE BEGIN Pluse100 */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
    
  }
  /* USER CODE END Pluse100 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
