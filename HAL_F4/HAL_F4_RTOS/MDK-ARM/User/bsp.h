#ifndef __BSP_H
#define __BSP_H
#include "motion.h"
#include "motor.h"
#include "oled.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ultraSonar.h"
#include "stepper.h"
#include "comm.h"
#include "stdio.h"
#include "main.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


extern u8 nonside[];
void BSP_Init(void);
#endif

