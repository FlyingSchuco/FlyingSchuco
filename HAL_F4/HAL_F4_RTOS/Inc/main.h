/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RF_A_Pin GPIO_PIN_5
#define RF_A_GPIO_Port GPIOE
#define RF_B_Pin GPIO_PIN_6
#define RF_B_GPIO_Port GPIOE
#define SONARR_TRIG_Pin GPIO_PIN_8
#define SONARR_TRIG_GPIO_Port GPIOF
#define SONARR_ECHO_Pin GPIO_PIN_9
#define SONARR_ECHO_GPIO_Port GPIOF
#define SONARR_ECHO_EXTI_IRQn EXTI9_5_IRQn
#define RB_A_Pin GPIO_PIN_14
#define RB_A_GPIO_Port GPIOB
#define RB_B_Pin GPIO_PIN_15
#define RB_B_GPIO_Port GPIOB
#define SONARF_ECHO_Pin GPIO_PIN_3
#define SONARF_ECHO_GPIO_Port GPIOG
#define SONARF_ECHO_EXTI_IRQn EXTI3_IRQn
#define LF_A_Pin GPIO_PIN_6
#define LF_A_GPIO_Port GPIOC
#define LF_B_Pin GPIO_PIN_7
#define LF_B_GPIO_Port GPIOC
#define LB_A_Pin GPIO_PIN_8
#define LB_A_GPIO_Port GPIOC
#define LB_B_Pin GPIO_PIN_9
#define LB_B_GPIO_Port GPIOC
#define SONARB_TRIG_Pin GPIO_PIN_11
#define SONARB_TRIG_GPIO_Port GPIOC
#define SONARB_ECHO_Pin GPIO_PIN_12
#define SONARB_ECHO_GPIO_Port GPIOC
#define SONARB_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOD
#define STP_Pin GPIO_PIN_5
#define STP_GPIO_Port GPIOD
#define SONARL_TRIG_Pin GPIO_PIN_15
#define SONARL_TRIG_GPIO_Port GPIOG
#define SONARL_ECHO_Pin GPIO_PIN_4
#define SONARL_ECHO_GPIO_Port GPIOB
#define SONARL_ECHO_EXTI_IRQn EXTI4_IRQn
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
