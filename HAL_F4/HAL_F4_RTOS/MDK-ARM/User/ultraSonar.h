#ifndef __ULTRASONAR_H 
#define __ULTRASONAR_H
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "FreeRTOS.h"
typedef struct
{
	GPIO_TypeDef *GPIO_Trig, *GPIO_Echo; 
	uint16_t PIN_Trig, PIN_Echo;
	HAL_StatusTypeDef state;	//状态
	uint16_t startTime;
	uint16_t endTime;
}Sonar;
Sonar* SonarInit(GPIO_TypeDef *GPIO_Trig, uint16_t PIN_Trig, GPIO_TypeDef *GPIO_Echo, uint16_t PIN_Echo);
void SonarTrig(Sonar *sonar);
#endif


