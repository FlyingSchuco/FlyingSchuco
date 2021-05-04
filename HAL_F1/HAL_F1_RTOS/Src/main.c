/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
#include "pid.h"
#include "delay.h"
#include "stepper.h"
#include "comm.h"
#include "opticalFlow.h"
#include "lcd.h"
#include "oled.h"
#include "bsp.h"
#include "it.h"
#include "motor.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pi 3.1415
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId_t defaultTaskHandle;
osThreadId_t LFSpeedHandle;
osThreadId_t LBSpeedHandle;
osThreadId_t RFSpeedHandle;
osThreadId_t RBSpeedHandle;
osThreadId_t MoveControlHandle;
osThreadId_t DecisionHandle;
osThreadId_t IMUDataHandle;
/* USER CODE BEGIN PV */
extern Motor *motorLF;
extern Motor *motorLB;
extern Motor *motorRF;
extern Motor *motorRB;
extern MotionState *MyRob;

uint8_t IRQBuffer;
ProtocolData *RxData;
PID_Typedef *PID_Front;
Stepper *StepperFront;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartLFSpeed(void *argument);
void StartLBSpeed(void *argument);
void StartRFSpeed(void *argument);
void StartRBSpeed(void *argument);
void StartMoveControl(void *argument);
void StartDecision(void *argument);
void StartIMUData(void *argument);

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
  MX_I2C1_Init();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  
  
  /* USER CODE BEGIN 2 */

BSP_Init();
  /* USER CODE END 2 */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of LFSpeed */
  const osThreadAttr_t LFSpeed_attributes = {
    .name = "LFSpeed",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };
  LFSpeedHandle = osThreadNew(StartLFSpeed, NULL, &LFSpeed_attributes);

  /* definition and creation of LBSpeed */
  const osThreadAttr_t LBSpeed_attributes = {
    .name = "LBSpeed",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };
  LBSpeedHandle = osThreadNew(StartLBSpeed, NULL, &LBSpeed_attributes);

  /* definition and creation of RFSpeed */
  const osThreadAttr_t RFSpeed_attributes = {
    .name = "RFSpeed",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };
  RFSpeedHandle = osThreadNew(StartRFSpeed, NULL, &RFSpeed_attributes);

  /* definition and creation of RBSpeed */
  const osThreadAttr_t RBSpeed_attributes = {
    .name = "RBSpeed",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };
  RBSpeedHandle = osThreadNew(StartRBSpeed, NULL, &RBSpeed_attributes);

  /* definition and creation of MoveControl */
  const osThreadAttr_t MoveControl_attributes = {
    .name = "MoveControl",
    .priority = (osPriority_t) osPriorityAboveNormal,
    .stack_size = 512
  };
  MoveControlHandle = osThreadNew(StartMoveControl, NULL, &MoveControl_attributes);

  /* definition and creation of Decision */
  const osThreadAttr_t Decision_attributes = {
    .name = "Decision",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 512
  };
  DecisionHandle = osThreadNew(StartDecision, NULL, &Decision_attributes);

  /* definition and creation of IMUData */
  const osThreadAttr_t IMUData_attributes = {
    .name = "IMUData",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 512
  };
  IMUDataHandle = osThreadNew(StartIMUData, NULL, &IMUData_attributes);

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
	;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_L_Pin|STP_Pin|OLED_DC_Pin|OLED_RST_Pin 
                          |OLED_SDA_Pin|OLED_SCL_Pin|IN3_L_Pin|IN4_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN4_R_Pin|IN3_R_Pin|IN2_R_Pin|IN1_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OPTF_RST_GPIO_Port, OPTF_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN2_L_GPIO_Port, IN2_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_L_Pin IN3_L_Pin IN4_L_Pin */
  GPIO_InitStruct.Pin = IN1_L_Pin|IN3_L_Pin|IN4_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STP_Pin OLED_DC_Pin OLED_RST_Pin OLED_SDA_Pin 
                           OLED_SCL_Pin */
  GPIO_InitStruct.Pin = STP_Pin|OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin 
                          |OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY0_Pin */
  GPIO_InitStruct.Pin = KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_R_Pin IN3_R_Pin IN2_R_Pin IN1_R_Pin */
  GPIO_InitStruct.Pin = IN4_R_Pin|IN3_R_Pin|IN2_R_Pin|IN1_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_NCS_Pin */
  GPIO_InitStruct.Pin = SPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OPTF_RST_Pin */
  GPIO_InitStruct.Pin = OPTF_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OPTF_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN2_L_Pin */
  GPIO_InitStruct.Pin = IN2_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN2_L_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN Header_StartLFSpeed */
/**
* @brief Function implementing the LFSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLFSpeed */
void StartLFSpeed(void *argument)
{
  /* USER CODE BEGIN StartLFSpeed */
	uint32_t countLF = 0;
	int rpmLF = 0;
	//char stringLF[10];
  /* Infinite loop */
  for(;;)
  {
	  taskENTER_CRITICAL();
	  countLF = ulTaskNotifyTake(pdTRUE,0); 
	  rpmLF = 100*countLF/13/2;
	  motorLF->Speed = rpmLF;
	  taskEXIT_CRITICAL();
    osDelay(10);
  }
  /* USER CODE END StartLFSpeed */
}

/* USER CODE BEGIN Header_StartLBSpeed */
/**
* @brief Function implementing the LBSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLBSpeed */
void StartLBSpeed(void *argument)
{
  /* USER CODE BEGIN StartLBSpeed */
	uint32_t countLB = 0;
	uint32_t rpmLB = 0;
	//char stringLB[10];
  /* Infinite loop */
  for(;;)
  {
	  taskENTER_CRITICAL();
	  countLB = ulTaskNotifyTake(pdTRUE,0);
	  rpmLB = 100*countLB/13/2;
	  motorLB->Speed = rpmLB;
	  taskEXIT_CRITICAL();
    osDelay(10);
  }
  /* USER CODE END StartLBSpeed */
}

/* USER CODE BEGIN Header_StartRFSpeed */
/**
* @brief Function implementing the RFSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRFSpeed */
void StartRFSpeed(void *argument)
{
  /* USER CODE BEGIN StartRFSpeed */
	uint32_t countRF = 0;
	uint32_t rpmRF = 0;
	//char stringRF[10];
  /* Infinite loop */
  for(;;)
  {
	  
	  taskENTER_CRITICAL();
	  countRF = ulTaskNotifyTake(pdTRUE,0);
	  rpmRF = 100*countRF/13/2;
	  motorRF->Speed = rpmRF;
	  taskEXIT_CRITICAL();
    osDelay(10);
  }
  /* USER CODE END StartRFSpeed */
}

/* USER CODE BEGIN Header_StartRBSpeed */
/**
* @brief Function implementing the RBSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRBSpeed */
void StartRBSpeed(void *argument)
{
  /* USER CODE BEGIN StartRBSpeed */
	uint32_t countRB = 0;
	uint32_t rpmRB = 0;
	//char stringRB[10];
  /* Infinite loop */
  for(;;)
  {
	  
	  taskENTER_CRITICAL();
	  countRB = ulTaskNotifyTake(pdTRUE,0);
	  rpmRB = 100*countRB/13/2;
	  motorRB->Speed = rpmRB;
	  taskEXIT_CRITICAL();
    osDelay(10);
  }
  /* USER CODE END StartRBSpeed */
}

/* USER CODE BEGIN Header_StartMoveControl */
/**
* @brief Function implementing the MoveControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMoveControl */
void StartMoveControl(void *argument)
{
  /* USER CODE BEGIN StartMoveControl */
	PID_Typedef *LF_PID = PID_Init(POSITION,0.6,0.08,0.04,1,100,45);
	PID_Typedef *LB_PID = PID_Init(POSITION,0.6,0.08,0.04,1,100,45);
	PID_Typedef *RF_PID = PID_Init(POSITION,0.6,0.08,0.04,1,100,45);
	PID_Typedef *RB_PID = PID_Init(POSITION,0.6,0.08,0.04,1,100,45);
	int pwmLF = 0, pwmLB = 0, pwmRF = 0, pwmRB = 0;
	int flag = 0,count = 0,trans = 0;
  /* Infinite loop */
  for(;;)
  {
	  taskENTER_CRITICAL();
		  {
			  
			  count++;

			  
		  pwmLF = (int)PID_Calc(LF_PID, ABS(motorLF->TargetSpeed), motorLF->Speed);
		  pwmLB = (int)PID_Calc(LB_PID, ABS(motorLB->TargetSpeed), motorLB->Speed);
		  pwmRF = (int)PID_Calc(RF_PID, ABS(motorRF->TargetSpeed), motorRF->Speed);
		  pwmRB = (int)PID_Calc(RB_PID, ABS(motorRB->TargetSpeed), motorRB->Speed);
			  
		  /*
		  printf("%d,%d,%d,%d,%d\n",
			  count,
			  motorLF->Speed,motorLB->Speed,
			  motorRF->Speed,motorRB->Speed);
		  */
		  
		  MotorRunToTarget(motorLF, pwmLF); 
		  MotorRunToTarget(motorLB, pwmLB); 
		  MotorRunToTarget(motorRF, pwmRF); 
		  MotorRunToTarget(motorRB, pwmRB); 
		  
			  
		  }
	  taskEXIT_CRITICAL();
    osDelay(10);
  }
  /* USER CODE END StartMoveControl */
}

/* USER CODE BEGIN Header_StartDecision */
/**
* @brief Function implementing the Decision thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDecision */
void StartDecision(void *argument)
{
  /* USER CODE BEGIN StartDecision */
	PID_Typedef *OMG_PID = PID_Init(POSITION,0.08,0,0.5,2,24,-24);
	float vx = 0,vy = 0,omega = 0;
	float L = 17, R = 8;
	int count = 0;
  /* Infinite loop */
  for(;;)
  {
	  //正式
	  count++;
	  vx = 0, vy = 0;
	  omega = PID_Calc(OMG_PID,0,MyRob->yaw);
	  printf("%d,%f\n",count,MyRob->yaw);

	  motorLF->TargetSpeed = (int)((-vx	-vy	-omega*L)/R * (30/Pi));
	  motorLB->TargetSpeed = (int)((-vx	+vy	-omega*L)/R * (30/Pi));
	  motorRF->TargetSpeed = (int)((+vx	-vy	-omega*L)/R * (30/Pi));
	  motorRB->TargetSpeed = (int)((+vx	+vy	-omega*L)/R * (30/Pi));
    osDelay(10);
	  
	 
	  //测试
	  /*
	  
	  
	  */
	  
  }
  /* USER CODE END StartDecision */
}

/* USER CODE BEGIN Header_StartIMUData */
/**
* @brief Function implementing the IMUData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUData */
void StartIMUData(void *argument)
{
  /* USER CODE BEGIN StartIMUData */
	float pitch = 0,roll = 0,yaw = 0; 		
	short aacx = 0,aacy = 0,aacz = 0;		
	short gyrox = 0,gyroy = 0,gyroz= 0;	
	short temp = 0;					
  /* Infinite loop */
  for(;;)
  {
	  taskENTER_CRITICAL();
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
			temp = MPU_Get_Temperature();	
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	
			MyRob->aacx = aacx;
			MyRob->aacy = -aacy;
			MyRob->aacz = -aacz;
			MyRob->gyrox = gyrox;
			MyRob->gyroy = -gyroy;
			MyRob->gyroz = -gyroz;
			MyRob->roll = roll;
			MyRob->pitch = -pitch;
			MyRob->yaw = -yaw;
			MyRob->temp = temp; 
		}
		taskEXIT_CRITICAL();
    osDelay(5);
  }
  /* USER CODE END StartIMUData */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
