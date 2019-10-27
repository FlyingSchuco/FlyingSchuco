#ifndef __STEPPER_H
#define __STEPPER_H
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "delay.h"
typedef struct
{
	GPIO_TypeDef *STP_GPIOx,*DIR_GPIOx;
	uint16_t STP_GPIO_Pin,DIR_GPIO_Pin;
	TIM_HandleTypeDef *htimx;
	float pos;
}Stepper;
Stepper *StepperInit(GPIO_TypeDef* STP_GPIOx,uint16_t STP_GPIO_Pin,GPIO_TypeDef* DIR_GPIOx,uint16_t DIR_GPIO_Pin,TIM_HandleTypeDef *htimx);
void StepperRun(Stepper *StepperNum,int speed,int dir);
#define LEFT 1
#define RIGHT -1
#endif
