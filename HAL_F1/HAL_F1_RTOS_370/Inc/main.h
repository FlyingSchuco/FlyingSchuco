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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define IN1_L_Pin GPIO_PIN_1
#define IN1_L_GPIO_Port GPIOC
#define STP_Pin GPIO_PIN_2
#define STP_GPIO_Port GPIOC
#define P4_Pin GPIO_PIN_2
#define P4_GPIO_Port GPIOA
#define P5_Pin GPIO_PIN_3
#define P5_GPIO_Port GPIOA
#define KEY0_Pin GPIO_PIN_5
#define KEY0_GPIO_Port GPIOC
#define RF_Pin GPIO_PIN_0
#define RF_GPIO_Port GPIOB
#define RB_Pin GPIO_PIN_1
#define RB_GPIO_Port GPIOB
#define ENA_L_Pin GPIO_PIN_10
#define ENA_L_GPIO_Port GPIOB
#define ENB_L_Pin GPIO_PIN_11
#define ENB_L_GPIO_Port GPIOB
#define IN4_R_Pin GPIO_PIN_12
#define IN4_R_GPIO_Port GPIOB
#define IN3_R_Pin GPIO_PIN_13
#define IN3_R_GPIO_Port GPIOB
#define IN2_R_Pin GPIO_PIN_14
#define IN2_R_GPIO_Port GPIOB
#define IN1_R_Pin GPIO_PIN_15
#define IN1_R_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_6
#define OLED_DC_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_8
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_9
#define OLED_SCL_GPIO_Port GPIOC
#define SPI_NCS_Pin GPIO_PIN_8
#define SPI_NCS_GPIO_Port GPIOA
#define OPTF_RST_Pin GPIO_PIN_11
#define OPTF_RST_GPIO_Port GPIOA
#define IN3_L_Pin GPIO_PIN_11
#define IN3_L_GPIO_Port GPIOC
#define IN4_L_Pin GPIO_PIN_12
#define IN4_L_GPIO_Port GPIOC
#define IN2_L_Pin GPIO_PIN_2
#define IN2_L_GPIO_Port GPIOD
#define LF_Pin GPIO_PIN_4
#define LF_GPIO_Port GPIOB
#define LB_Pin GPIO_PIN_5
#define LB_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB
#define ENB_R_Pin GPIO_PIN_8
#define ENB_R_GPIO_Port GPIOB
#define ENA_R_Pin GPIO_PIN_9
#define ENA_R_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
