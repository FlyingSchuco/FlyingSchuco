#ifndef __IT_H
#define __IT_H
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "oled.h"
#include "string.h"
#include "ultraSonar.h"
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
extern TIM_HandleTypeDef htim6;
extern Sonar *sonarF;
#endif 
