#include "ultraSonar.h"

Sonar* SonarInit(GPIO_TypeDef *GPIO_Trig, uint16_t PIN_Trig, GPIO_TypeDef *GPIO_Echo, uint16_t PIN_Echo)
{
	Sonar *sonar;
	sonar = (Sonar *)pvPortMalloc(sizeof(Sonar));
	sonar->GPIO_Echo = GPIO_Echo;
	sonar->PIN_Echo = PIN_Echo;
	sonar->GPIO_Trig = GPIO_Trig;
	sonar->PIN_Trig = PIN_Trig;
	sonar->state = HAL_OK;
	sonar->startTime = 0x0000;
	sonar->endTime = 0x0000;
	HAL_GPIO_WritePin(sonar->GPIO_Trig, sonar->PIN_Trig, GPIO_PIN_RESET);
	return sonar;
}

void SonarTrig(Sonar *sonar)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
	gen_delay_us(10);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,  GPIO_PIN_SET);
	gen_delay_us(50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,  GPIO_PIN_RESET);
	sonar->state = HAL_BUSY;
}


