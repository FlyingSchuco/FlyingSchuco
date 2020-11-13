#ifndef __IT_H
#define __IT_H
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "oled.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
extern osThreadId_t LFSpeedHandle;
extern osThreadId_t LBSpeedHandle;
extern osThreadId_t RFSpeedHandle;
extern osThreadId_t RBSpeedHandle;
#endif 
