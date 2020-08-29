#ifndef __BSP_H
#define __BSP_H
#include "motor.h"
#include "oled.h"
#include "stdio.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern u8 nonside[];
void BSP_Init();
#endif

