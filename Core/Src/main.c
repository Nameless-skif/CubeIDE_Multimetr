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
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

typedef enum {
	vBus,
	vShunt,
	Current
} DataTask_t;

typedef struct {
	char Buf[30];
	char mess[10];
} QUEUE_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADC_Task */
osThreadId_t ADC_TaskHandle;
uint32_t ADC_TaskBuffer[ 500 ];
osStaticThreadDef_t ADC_TaskControlBlock;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .cb_mem = &ADC_TaskControlBlock,
  .cb_size = sizeof(ADC_TaskControlBlock),
  .stack_mem = &ADC_TaskBuffer[0],
  .stack_size = sizeof(ADC_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
uint32_t UART_TaskBuffer[ 128 ];
osStaticThreadDef_t UART_TaskControlBlock;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .cb_mem = &UART_TaskControlBlock,
  .cb_size = sizeof(UART_TaskControlBlock),
  .stack_mem = &UART_TaskBuffer[0],
  .stack_size = sizeof(UART_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TFT_Task */
osThreadId_t TFT_TaskHandle;
uint32_t TFT_TaskBuffer[ 128 ];
osStaticThreadDef_t TFT_TaskControlBlock;
const osThreadAttr_t TFT_Task_attributes = {
  .name = "TFT_Task",
  .cb_mem = &TFT_TaskControlBlock,
  .cb_size = sizeof(TFT_TaskControlBlock),
  .stack_mem = &TFT_TaskBuffer[0],
  .stack_size = sizeof(TFT_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for INA219_Current */
osThreadId_t INA219_CurrentHandle;
uint32_t INA219_TaskBuffer[ 400 ];
osStaticThreadDef_t INA219_TaskControlBlock;
const osThreadAttr_t INA219_Current_attributes = {
  .name = "INA219_Current",
  .cb_mem = &INA219_TaskControlBlock,
  .cb_size = sizeof(INA219_TaskControlBlock),
  .stack_mem = &INA219_TaskBuffer[0],
  .stack_size = sizeof(INA219_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for InitMyDevice */
osThreadId_t InitMyDeviceHandle;
uint32_t InitMyDeviceBuffer[ 128 ];
osStaticThreadDef_t InitMyDeviceControlBlock;
const osThreadAttr_t InitMyDevice_attributes = {
  .name = "InitMyDevice",
  .cb_mem = &InitMyDeviceControlBlock,
  .cb_size = sizeof(InitMyDeviceControlBlock),
  .stack_mem = &InitMyDeviceBuffer[0],
  .stack_size = sizeof(InitMyDeviceBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for INA219_vBus */
osThreadId_t INA219_vBusHandle;
const osThreadAttr_t INA219_vBus_attributes = {
  .name = "INA219_vBus",
  .stack_size = 350 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for INA219_vShunt */
osThreadId_t INA219_vShuntHandle;
const osThreadAttr_t INA219_vShunt_attributes = {
  .name = "INA219_vShunt",
  .stack_size = 350 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ManagementMatri */
osThreadId_t ManagementMatriHandle;
uint32_t ManagementMatriBuffer[ 250 ];
osStaticThreadDef_t ManagementMatriControlBlock;
const osThreadAttr_t ManagementMatri_attributes = {
  .name = "ManagementMatri",
  .cb_mem = &ManagementMatriControlBlock,
  .cb_size = sizeof(ManagementMatriControlBlock),
  .stack_mem = &ManagementMatriBuffer[0],
  .stack_size = sizeof(ManagementMatriBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
uint8_t myQueue01Buffer[ 10 * sizeof( QUEUE_t ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01",
  .cb_mem = &myQueue01ControlBlock,
  .cb_size = sizeof(myQueue01ControlBlock),
  .mq_mem = &myQueue01Buffer,
  .mq_size = sizeof(myQueue01Buffer)
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void StartADC_Task(void *argument);
void StartUART_Task(void *argument);
void StartTFT_Task(void *argument);
void StartINA219_Current_Task(void *argument);
void StartInitMyDevice(void *argument);
void INA219_vBus_Task(void *argument);
void INA219_vShunt_Task(void *argument);
void StartManagementMatrix(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
INA219_t ina219;
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

//  /* Код для ina219   */
//  unsigned char uart_tx_buff[100];
//  uint16_t vbus, vshunt, current, config;
//  float current_correctly;
//  while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS))
//       {
//
//        }
//   sprintf(uart_tx_buff, "**********		Hello battery app	 **********\r\n");
//   HAL_UART_Transmit(&huart1, uart_tx_buff, strlen(uart_tx_buff), 100);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (10, sizeof(QUEUE_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ADC_Task */
  ADC_TaskHandle = osThreadNew(StartADC_Task, NULL, &ADC_Task_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(StartUART_Task, NULL, &UART_Task_attributes);

  /* creation of TFT_Task */
  TFT_TaskHandle = osThreadNew(StartTFT_Task, NULL, &TFT_Task_attributes);

  /* creation of INA219_Current */
  INA219_CurrentHandle = osThreadNew(StartINA219_Current_Task, NULL, &INA219_Current_attributes);

  /* creation of InitMyDevice */
  InitMyDeviceHandle = osThreadNew(StartInitMyDevice, NULL, &InitMyDevice_attributes);

  /* creation of INA219_vBus */
  INA219_vBusHandle = osThreadNew(INA219_vBus_Task, NULL, &INA219_vBus_attributes);

  /* creation of INA219_vShunt */
  INA219_vShuntHandle = osThreadNew(INA219_vShunt_Task, NULL, &INA219_vShunt_attributes);

  /* creation of ManagementMatri */
  ManagementMatriHandle = osThreadNew(StartManagementMatrix, NULL, &ManagementMatri_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_Blink_GPIO_Port, LED1_Blink_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TOUCH_CS_Pin|TFT_CS_Pin|TFT_RST_Pin|TFT_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Blink_Pin */
  GPIO_InitStruct.Pin = LED1_Blink_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_Blink_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_CS_Pin TFT_CS_Pin TFT_RST_Pin TFT_DC_Pin */
  GPIO_InitStruct.Pin = TOUCH_CS_Pin|TFT_CS_Pin|TFT_RST_Pin|TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartADC_Task */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC_Task */
void StartADC_Task(void *argument)
{
  /* USER CODE BEGIN StartADC_Task */
   QUEUE_t msg;
   float u_res =0;
   char ADC_char_res[6];
  /* Infinite loop */
  for(;;)
  {

	HAL_ADC_Start(&hadc1);
	u_res = HAL_ADC_GetValue(&hadc1)* 3.3f / 4095.0f;
    sprintf(ADC_char_res, "%1.3f", u_res);
	strcpy(msg.Buf,ADC_char_res);
	strcpy(msg.mess,"ADC");
	osMessageQueuePut(myQueue01Handle, &msg, 0, osWaitForever); //Поместили в очередь данные
	osDelay(300);
  }
  /* USER CODE END StartADC_Task */
}

/* USER CODE BEGIN Header_StartUART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_Task */
void StartUART_Task(void *argument)
{
  /* USER CODE BEGIN StartUART_Task */
   QUEUE_t msg;
   char message_ADC[] = "Value ADC = ";
   char message_vBus[] = "Value vBus[V] = ";
   char message_vShunt[] = "Value vShunt[mV] = ";
   char message_Current[] = "Value Current[mA] = ";
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(myQueue01Handle, &msg,0 ,osWaitForever);
	  if(!strcmp(msg.mess, "ADC")) {
		    HAL_UART_Transmit(&huart1, (uint8_t*)message_ADC, strlen(message_ADC), osWaitForever);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)msg.Buf, strlen(msg.Buf), osWaitForever);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)" \n", 2, osWaitForever);
	  		 }
	  else if(!strcmp(msg.mess, "vBus")){
		    HAL_UART_Transmit(&huart1, (uint8_t*)message_vBus, strlen(message_vBus), osWaitForever);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)msg.Buf, strlen(msg.Buf), osWaitForever);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)" \n", 2, osWaitForever);
	  	  	  }
	  else if(!strcmp(msg.mess, "vShunt")){
		    HAL_UART_Transmit(&huart1, (uint8_t*)message_vShunt, strlen(message_vShunt), osWaitForever);
		 	HAL_UART_Transmit(&huart1, (uint8_t*)msg.Buf, strlen(msg.Buf), osWaitForever);
		 	HAL_UART_Transmit(&huart1, (uint8_t*)" \n", 2, osWaitForever);
	  	  	  }
	  else if(!strcmp(msg.mess, "Current")){
		    HAL_UART_Transmit(&huart1, (uint8_t*)message_Current, strlen(message_Current), osWaitForever);
		 	HAL_UART_Transmit(&huart1, (uint8_t*)msg.Buf, strlen(msg.Buf), osWaitForever);
		 	HAL_UART_Transmit(&huart1, (uint8_t*)" \n", 2, osWaitForever);
	  	  	  }
	osDelay(500);
  }
  /* USER CODE END StartUART_Task */
}

/* USER CODE BEGIN Header_StartTFT_Task */
/**
* @brief Function implementing the TFT_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTFT_Task */
void StartTFT_Task(void *argument)
{
  /* USER CODE BEGIN StartTFT_Task */
  QUEUE_t msg;
  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(myQueue01Handle, &msg,0 ,10);
	if(!strcmp(msg.mess, "ADC")) {
			ILI9341_WriteString(0, 36, msg.Buf, Font_11x18, WHITE, MYFON);
		 }
	else if(!strcmp(msg.mess, "vBus")){
			ILI9341_WriteString(150, 54, msg.Buf, Font_11x18, WHITE, MYFON);
		}
	else if(!strcmp(msg.mess, "vShunt")){
			ILI9341_WriteString(150, 72, msg.Buf, Font_11x18, WHITE, MYFON);
			}
	else if(!strcmp(msg.mess, "Current")){
			ILI9341_WriteString(150, 90, msg.Buf, Font_11x18, WHITE, MYFON);
			}
    osDelay(100);
  }
  /* USER CODE END StartTFT_Task */
}

/* USER CODE BEGIN Header_StartINA219_Current_Task */
/**
* @brief Function implementing the INA219_Current thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINA219_Current_Task */
void StartINA219_Current_Task(void *argument)
{
  /* USER CODE BEGIN StartINA219_Current_Task */
  QUEUE_t msg;
  char uart_tx_buff[100];
  float current;
  /* Infinite loop */
  for(;;)
  {
	current = INA219_ReadCurrent(&ina219);
	sprintf(uart_tx_buff,"%4.2f",current);
	strcpy(msg.mess,"Current");
	strcpy(msg.Buf,uart_tx_buff);
	osMessageQueuePut(myQueue01Handle, &msg, 0, osWaitForever); //Поместили в очередь данные
    osDelay(300);
  }
  /* USER CODE END StartINA219_Current_Task */
}

/* USER CODE BEGIN Header_StartInitMyDevice */
/**
* @brief Function implementing the InitMyDevice thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInitMyDevice */
void StartInitMyDevice(void *argument)
{
  /* USER CODE BEGIN StartInitMyDevice */
	 char uart_tx_buff[100];
  /* Infinite loop */
  for(;;)
  {
	  __HAL_SPI_ENABLE(DISP_SPI_PTR);
	   ILI9341_Init();
	   ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);

	   ILI9341_Fill_Screen(MYFON);
	   ILI9341_WriteString(0, 0, "<---Pulsar--->", Font_11x18, WHITE, MYFON);
	   ILI9341_WriteString(0, 18, "The value of the ADC", Font_11x18, WHITE, MYFON);
	   ILI9341_WriteString(60, 36, "V", Font_11x18, WHITE, MYFON);
	   ILI9341_WriteString(0, 54, "vBus[V] ", Font_11x18, WHITE, MYFON);
	   ILI9341_WriteString(0, 72, "vShunt[mV]", Font_11x18, WHITE, MYFON);
	   ILI9341_WriteString(0, 90, "Current[mA] ", Font_11x18, WHITE, MYFON);
	   HAL_Delay(100);
	   /* Код для ina219   */
	   while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS))
	        {

	         }
	    sprintf(uart_tx_buff, "**********		Hello INA219 app	 **********\r\n");
	    HAL_UART_Transmit(&huart1, uart_tx_buff, strlen(uart_tx_buff), 100);
	    INA219_setCalibration_16V_400mA(&ina219);

    osDelay(1);
    vTaskSuspend(InitMyDeviceHandle); //Блокировка задачи
  }
  /* USER CODE END StartInitMyDevice */
}

/* USER CODE BEGIN Header_INA219_vBus_Task */
/**
* @brief Function implementing the INA219_vBus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INA219_vBus_Task */
void INA219_vBus_Task(void *argument)
{
  /* USER CODE BEGIN INA219_vBus_Task */
  QUEUE_t msg;
  char uart_tx_buff[100];
  float vbus;
  /* Infinite loop */
  for(;;)
  {
	vbus = INA219_ReadBusVoltage(&ina219)/ 1000.0; // переводим в В из мВ
	sprintf(uart_tx_buff, "%4.2f",vbus);
	strcpy(msg.mess,"vBus");
	strcpy(msg.Buf,uart_tx_buff);
	osMessageQueuePut(myQueue01Handle, &msg, 0, osWaitForever); //Поместили в очередь данные
    osDelay(70);
  }
  /* USER CODE END INA219_vBus_Task */
}

/* USER CODE BEGIN Header_INA219_vShunt_Task */
/**
* @brief Function implementing the INA219_vShunt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INA219_vShunt_Task */
void INA219_vShunt_Task(void *argument)
{
  /* USER CODE BEGIN INA219_vShunt_Task */
	 QUEUE_t msg;
	 char uart_tx_buff[100];
	 uint16_t vShunt;
  /* Infinite loop */
  for(;;)
  {
	vShunt = INA219_ReadShuntVolage(&ina219);
//	sprintf(uart_tx_buff, "vShunt: %hu mV\r\n",vShunt);
	sprintf(uart_tx_buff, "%hu",vShunt);
	strcpy(msg.mess,"vShunt");
	strcpy(msg.Buf,uart_tx_buff);
	osMessageQueuePut(myQueue01Handle, &msg, 0, osWaitForever); //Поместили в очередь данные
//	HAL_UART_Transmit(&huart1, uart_tx_buff, strlen(uart_tx_buff), 100);
	osDelay(300);
  }
  /* USER CODE END INA219_vShunt_Task */
}

/* USER CODE BEGIN Header_StartManagementMatrix */
/**
* @brief Function implementing the ManagementMatri thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartManagementMatrix */
void StartManagementMatrix(void *argument)
{
  /* USER CODE BEGIN StartManagementMatrix */
	uint8_t SPI2_Data[1];
	SPI2_Data[0] = 232;
	/* Infinite loop */
  for(;;)
  {
	HAL_SPI_Transmit(&hspi2, (uint8_t*)SPI2_Data, 1, osWaitForever);
//	HAL_SPI_Transmit(&hspi2, tranmit_Data, 1, 100);
//	GPIOB ->BSRR|= GPIO_BSRR_BS12;
//	GPIOB ->BSRR|= GPIO_BSRR_BR12;
    osDelay(1);
  }
  /* USER CODE END StartManagementMatrix */
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
  void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
  {
    if (hspi->Instance == SPI1)
    {
      // Передача завершена
    	ILI9341_WriteString(20, 120, "Trancmitt complete", Font_16x26, WHITE, MYFON);
  }
  else {
	  ILI9341_WriteString(20, 120, "Not complete", Font_16x26, WHITE, MYFON);
}}
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
