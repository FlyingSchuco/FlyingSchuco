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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "myMath.h"
#include "pid.h"
#include "oled.h"
#include "bsp.h"
#include "it.h"
#include "motor.h"
#include "motion.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ultraSonar.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pi 3.1416f
#define K2 1.414f
#define ANGLE_TH 45.0f
#define DECISION

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId_t defaultTaskHandle;
osThreadId_t IMUDataHandle;
osThreadId_t wheelControlHandle;
osThreadId_t wheelSpeedHandle;
osThreadId_t moveDecisionHandle;
osThreadId_t stepperControlHandle;
osThreadId_t ultraSonarHandle;
osThreadId_t openmvComHandle;
/* USER CODE BEGIN PV */
extern MotionState *myRob;
extern Motor *motorLF;
extern Motor *motorLB;
extern Motor *motorRF;
extern Motor *motorRB;
extern Stepper *stepperFront;
extern Sonar *sonarF;
extern Sonar *sonarB;
extern Sonar *sonarL;
extern Sonar *sonarR;
extern ProtocolData *RxData;

float vx = 0.0f,vy = 0.0f,	//cm/s
	omega = 0.0f, // rad/s
	yaw = 0.0f;	// deg
float L = 16.0f, R = 3.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartIMUData(void *argument);
void StartWheelControl(void *argument);
void StartWheelSpeed(void *argument);
void StartDecision(void *argument);
void StartStepperControl(void *argument);
void StartUltraSonar(void *argument);
void StartOpenmvCom(void *argument);

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
  MX_GPIO_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
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
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 128
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of IMUData */
  const osThreadAttr_t IMUData_attributes = {
    .name = "IMUData",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 512
  };
  IMUDataHandle = osThreadNew(StartIMUData, NULL, &IMUData_attributes);

  /* definition and creation of wheelControl */
  const osThreadAttr_t wheelControl_attributes = {
    .name = "wheelControl",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  wheelControlHandle = osThreadNew(StartWheelControl, NULL, &wheelControl_attributes);

  /* definition and creation of wheelSpeed */
  const osThreadAttr_t wheelSpeed_attributes = {
    .name = "wheelSpeed",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 512
  };
  wheelSpeedHandle = osThreadNew(StartWheelSpeed, NULL, &wheelSpeed_attributes);

  /* definition and creation of moveDecision */
  const osThreadAttr_t moveDecision_attributes = {
    .name = "moveDecision",
    .priority = (osPriority_t) osPriorityAboveNormal,
    .stack_size = 512
  };
  moveDecisionHandle = osThreadNew(StartDecision, NULL, &moveDecision_attributes);

  /* definition and creation of stepperControl */
  const osThreadAttr_t stepperControl_attributes = {
    .name = "stepperControl",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 512
  };
  stepperControlHandle = osThreadNew(StartStepperControl, NULL, &stepperControl_attributes);

  /* definition and creation of ultraSonar */
  const osThreadAttr_t ultraSonar_attributes = {
    .name = "ultraSonar",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 1024
  };
  ultraSonarHandle = osThreadNew(StartUltraSonar, NULL, &ultraSonar_attributes);

  /* definition and creation of openmvCom */
  const osThreadAttr_t openmvCom_attributes = {
    .name = "openmvCom",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 512
  };
  openmvComHandle = osThreadNew(StartOpenmvCom, NULL, &openmvCom_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 8-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 4-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  htim14.Init.Prescaler = 84-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
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
  huart2.Init.Mode = UART_MODE_RX;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SONARR_TRIG_GPIO_Port, SONARR_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SONARF_TRIG_Pin|SONARL_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SONARB_TRIG_GPIO_Port, SONARB_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR_Pin|STP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IMU_SCL_Pin|IMU_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SONARR_TRIG_Pin */
  GPIO_InitStruct.Pin = SONARR_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SONARR_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SONARR_ECHO_Pin */
  GPIO_InitStruct.Pin = SONARR_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONARR_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SONARF_TRIG_Pin SONARL_TRIG_Pin */
  GPIO_InitStruct.Pin = SONARF_TRIG_Pin|SONARL_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SONARF_ECHO_Pin */
  GPIO_InitStruct.Pin = SONARF_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONARF_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SONARB_TRIG_Pin */
  GPIO_InitStruct.Pin = SONARB_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SONARB_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SONARB_ECHO_Pin */
  GPIO_InitStruct.Pin = SONARB_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONARB_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin STP_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SONARL_ECHO_Pin */
  GPIO_InitStruct.Pin = SONARL_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SONARL_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IMU_SCL_Pin IMU_SDA_Pin */
  GPIO_InitStruct.Pin = IMU_SCL_Pin|IMU_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
			myRob->aacx = aacx;
			myRob->aacy = -aacy;
			myRob->aacz = -aacz;
			myRob->gyrox = gyrox;
			myRob->gyroy = -gyroy;
			myRob->gyroz = -gyroz;
			myRob->roll = roll;
			myRob->pitch = -pitch;
			myRob->yaw = -yaw;
			myRob->temp = temp; 
		//printf("pitch = %f, yaw = %f, roll = %f\n", myRob->pitch, myRob->yaw, myRob->roll);
	}
	  
	taskEXIT_CRITICAL();
    osDelay(5);
  }
  /* USER CODE END StartIMUData */
}

/* USER CODE BEGIN Header_StartWheelControl */
/**
* @brief Function implementing the wheelControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWheelControl */
void StartWheelControl(void *argument)
{
  /* USER CODE BEGIN StartWheelControl */
	PID_Typedef *LF_PID = PID_Init(POSITION,6,1.2,0.2,1,1000,0);
	PID_Typedef *LB_PID = PID_Init(POSITION,6,1.2,0.2,1,1000,0);
	PID_Typedef *RF_PID = PID_Init(POSITION,6,1.2,0.2,1,1000,0);
	PID_Typedef *RB_PID = PID_Init(POSITION,6,1.2,0.2,1,1000,0);
	PID_Typedef *OMG_PID = PID_Init(POSITION,0.08,0,0.5,2,24,-24);
	int pwmLF = 0, pwmLB = 0, pwmRF = 0, pwmRB = 0;
  /* Infinite loop */
  for(;;)
  {
	  taskENTER_CRITICAL();
	  if(fabs(yaw)>150.0f)
	  {
		omega = PID_Calc(OMG_PID,DegRange180(yaw+180.0f),DegRange180(myRob->yaw+180.0f));
	  }
	  else 
	  {
		omega = PID_Calc(OMG_PID,yaw,myRob->yaw);
	  }
	  taskEXIT_CRITICAL();
	  //printf("%f\n",myRob->yaw);
	  
	  motorLF->TargetSpeed = (int)((-vx/K2	-vy/K2	-omega*L)/R * (30/Pi));
	  motorLB->TargetSpeed = (int)((-vx/K2	+vy/K2	-omega*L)/R * (30/Pi));
	  motorRF->TargetSpeed = (int)((+vx/K2	-vy/K2	-omega*L)/R * (30/Pi));
	  motorRB->TargetSpeed = (int)((+vx/K2	+vy/K2	-omega*L)/R * (30/Pi));
	  
	  
	  
	  taskENTER_CRITICAL();
	  pwmLF = (int)PID_Calc(LF_PID, ABS(motorLF->TargetSpeed), motorLF->Speed);
	  pwmLB = (int)PID_Calc(LB_PID, ABS(motorLB->TargetSpeed), motorLB->Speed);
	  pwmRF = (int)PID_Calc(RF_PID, ABS(motorRF->TargetSpeed), motorRF->Speed);
	  pwmRB = (int)PID_Calc(RB_PID, ABS(motorRB->TargetSpeed), motorRB->Speed);
	  taskEXIT_CRITICAL();
	  
	  MotorRunToTarget(motorLF,pwmLF);
	  MotorRunToTarget(motorLB,pwmLB);
	  MotorRunToTarget(motorRF,pwmRF);
	  MotorRunToTarget(motorRB,pwmRB);
	  
	osDelay(10);
  }
  /* USER CODE END StartWheelControl */
}

/* USER CODE BEGIN Header_StartWheelSpeed */
/**
* @brief Function implementing the wheelSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWheelSpeed */
void StartWheelSpeed(void *argument)
{
  /* USER CODE BEGIN StartWheelSpeed */
  /* Infinite loop */
  for(;;)
  {
	  //measure the speed of 4 motors
	  MotorSpeedMeasure(motorLF);
	  MotorSpeedMeasure(motorLB);
	  MotorSpeedMeasure(motorRF);
	  MotorSpeedMeasure(motorRB);
	  //printf("%d,%d,%d,%d\n",motorLF->Speed,motorLB->Speed,motorRF->Speed,motorRB->Speed);
    osDelay(10);
  }
  /* USER CODE END StartWheelSpeed */
}

/* USER CODE BEGIN Header_StartDecision */
/**
* @brief Function implementing the moveDecision thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDecision */
void StartDecision(void *argument)
{
  /* USER CODE BEGIN StartDecision */
	float angle = 0.0f;
  /* Infinite loop */
  for(;;)
  {
	  #ifdef DECISION
	  if(myRob->target == 0)			//No targets are found 
	  {
		  //Cruise();					//Cruising on a fixed route
		  
		  if(fabs(myRob->yaw) < 90.0f)
		  {
			  yaw = 0.0f;
		  }
		  else
		  {
			  yaw = -180.0f;
		  }
		  
		  if(fabs(myRob->yaw) < 10.0f)
		  {
		      myRob->x = (300.0f - myRob->dF + myRob->dB)/2.0f;
			  myRob->y = (200.0f - myRob->dR + myRob->dL)/2.0f;
		  }
		  else if(fabs(myRob->yaw) > (180.0f - 10.0f))
		  {
		      myRob->x = (300.0f + myRob->dF - myRob->dB)/2.0f;
			  myRob->y = (200.0f + myRob->dR - myRob->dL)/2.0f; 
		  }
		  else 
		  {
			  
		  }
		  
		  if(fabs(myRob->x-150.0f)>5.0f)
		  {
			  vx = sign(150.0f - myRob->x) * 10.0f ;// * cos(myRob->yaw);
		  }
		  else 
		  {
			  vx = 0.0f;
		  }
		  
		  if(fabs(myRob->y-100.0f)>5.0f)
		  {
			  vy = sign(150.0f - myRob->y) * 10.0f ;// * cos(myRob->yaw);
		  }
		  else 
		  {
			  vy = 0.0f;
		  }
		  
	  } 
	  else if(myRob->target == 1)	//target is found
	  {
		  // -360~360 to -180~180
		  angle = DegRange180(myRob->yaw + stepperFront->pos);
		  
		  if(fabs(angle) < ANGLE_TH || fabs(angle) > (180.0f - ANGLE_TH) ) 			//yaw is small
		  {
			  vx = 20.0f;
			  vy = 0.0f;
			  yaw = 0.0f;
		  }
		  else 						//yaw is large enough
		  {
			  yaw = stepperFront->pos;
			  //ready to bump
			  if(myRob->dF>30.0f)
			  {
				  vx = 30.0f * cos(stepperFront->pos);
				  vy = 30.0f * sin(stepperFront->pos);
			  }
			  else if(myRob->dF>15.0f)
			  {
				  vx = myRob->dF * cos(stepperFront->pos);
				  vy = myRob->dF * sin(stepperFront->pos);
			  }
			  else
			  {
				  vx = 10.0f * cos(stepperFront->pos);
				  vy = 10.0f * sin(stepperFront->pos);
			  }
		  }
	  }
	  else 							//Not used
	  {
		  
	  }
	  
	#endif
    osDelay(20);
  }
  /* USER CODE END StartDecision */
}

/* USER CODE BEGIN Header_StartStepperControl */
/**
* @brief Function implementing the stepperControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStepperControl */
void StartStepperControl(void *argument)
{
  /* USER CODE BEGIN StartStepperControl */
	PID_Typedef *STP_PID = PID_Init(POSITION,0.2,0.008,0.02,2,45,-45);
  /* Infinite loop */
  for(;;)
  {
	// StepperRun =  (int)PID_Calc(STP_PID,0,myRob->pixel_dx);
    osDelay(10);
  }
  /* USER CODE END StartStepperControl */
}

/* USER CODE BEGIN Header_StartUltraSonar */
/**
* @brief Function implementing the ultraSonar thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltraSonar */
void StartUltraSonar(void *argument)
{
  /* USER CODE BEGIN StartUltraSonar */
	float distF[5] = {300.0f, 300.0f, 300.0f, 300.0f, 300.0f};
	float distB[5] = {300.0f, 300.0f, 300.0f, 300.0f, 300.0f};
	float distL[5] = {300.0f, 300.0f, 300.0f, 300.0f, 300.0f};
	float distR[5] = {300.0f, 300.0f, 300.0f, 300.0f, 300.0f};
	int count = 0;
  /* Infinite loop */
  for(;;)
  {
	SonarTrig(sonarF);
	SonarTrig(sonarB);
	SonarTrig(sonarL);
	SonarTrig(sonarR);
    osDelay(50);
	distF[count] = SonarMeasure(sonarF);
	distB[count] = SonarMeasure(sonarB);
	distL[count] = SonarMeasure(sonarL);
	distR[count] = SonarMeasure(sonarR);
	count++;
	if(count == 5)
	{
		qsort(distF,5,sizeof(float),CmpFloat);
		qsort(distB,5,sizeof(float),CmpFloat);
		qsort(distL,5,sizeof(float),CmpFloat);
		qsort(distR,5,sizeof(float),CmpFloat);
		myRob->dF = (distF[1]+distF[2]+distF[3])/3.0f;
		myRob->dB = (distB[1]+distB[2]+distB[3])/3.0f;
		myRob->dL = (distL[1]+distL[2]+distL[3])/3.0f;
		myRob->dR = (distR[1]+distR[2]+distR[3])/3.0f;
		count = 0;
	}
  }
  /* USER CODE END StartUltraSonar */
}

/* USER CODE BEGIN Header_StartOpenmvCom */
/**
* @brief Function implementing the openmvCom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOpenmvCom */
void StartOpenmvCom(void *argument)
{
  /* USER CODE BEGIN StartOpenmvCom */
	
  /* Infinite loop */
  for(;;)
  {
	taskENTER_CRITICAL();
	if(RxData->color == LOST)
	{
		myRob->target = 1;
	}
	else 
	{
		myRob->target = 0;
	}
	myRob->pixel_dx = RxData->dx;
	myRob->pixel_dy = RxData->dy;
	taskEXIT_CRITICAL();
    osDelay(20);
  }
  /* USER CODE END StartOpenmvCom */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM13 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM13) {
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
