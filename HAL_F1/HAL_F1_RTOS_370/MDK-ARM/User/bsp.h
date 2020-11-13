#ifndef __BSP_H
#define __BSP_H
#include "motion.h"
#include "motor.h"
#include "oled.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "stdio.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern MotionState *MyRob;
extern u8 nonside[];
void BSP_Init(void);
#endif

