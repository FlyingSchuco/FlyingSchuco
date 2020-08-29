#ifndef __STEPPER_H
#define __STEPPER_H
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "delay.h"
#include "pid.h"
#define PULSE_MS 1
#define PULSE_US 1000
typedef struct
{
	GPIO_TypeDef *STP_GPIOx,*DIR_GPIOx;
	uint16_t STP_GPIO_Pin,DIR_GPIO_Pin;
	float pos;
}Stepper;
Stepper *StepperInit(GPIO_TypeDef* STP_GPIOx,uint16_t STP_GPIO_Pin,GPIO_TypeDef* DIR_GPIOx,uint16_t DIR_GPIO_Pin);
void StepperRun(Stepper *StepperPTR, int speed, int dir);
void StepperRotate(Stepper *StepperPTR, float angle, int dir);
void StepperAutoRotate(Stepper *StepperPTR, int angle);
#define LEFT 1
#define RIGHT -1
#endif
