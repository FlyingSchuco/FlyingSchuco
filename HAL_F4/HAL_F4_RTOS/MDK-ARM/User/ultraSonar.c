#include "ultraSonar.h"

Sonar* SonarInit(GPIO_TypeDef *GPIO_Trig, uint16_t PIN_Trig, GPIO_TypeDef *GPIO_Echo, uint16_t PIN_Echo, int num)
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
	sonar->num = num;
	HAL_GPIO_WritePin(sonar->GPIO_Trig, sonar->PIN_Trig, GPIO_PIN_RESET);
	return sonar;
}

void SonarTrig(Sonar *sonar)
{
	HAL_GPIO_WritePin(sonar->GPIO_Trig, sonar->PIN_Trig, GPIO_PIN_RESET);
	gen_delay_us(10);
	HAL_GPIO_WritePin(sonar->GPIO_Trig, sonar->PIN_Trig,  GPIO_PIN_SET);
	gen_delay_us(50);
	HAL_GPIO_WritePin(sonar->GPIO_Trig, sonar->PIN_Trig,  GPIO_PIN_RESET);
	sonar->state = HAL_BUSY;
}

float SonarMeasure(Sonar *sonar)
{
	float dist = 400.0f;
	if(sonar->state == HAL_BUSY)
	{
		sonar->state = HAL_OK;
		#ifdef SONAR_DEBUG 
			printf("soanr%d overtime\n",sonar->num); 
		#endif
		return 401.0f;
	}
	else if(sonar->state == HAL_OK)
	{
		if(sonar->endTime > sonar->startTime)
		{
			dist = 340.0f*(float)(+sonar->endTime-sonar->startTime)/2.0f/10000.0f;
		}
		else if(sonar->endTime < sonar->startTime)
		{
			dist = 340.0f*(float)(0xFFFF-sonar->startTime+sonar->endTime)/2.0f/10000.0f;
		}
		else 
		{
			#ifdef SONAR_DEBUG 
			printf("soanr%d endTime is equal to startTime\n",sonar->num); 
			#endif
			return 404.0f;
		}
	}
	else 
	{
		#ifdef SONAR_DEBUG 
			printf("soanr%d unknown state\n",sonar->num); 
		#endif
		return 403.0f;
	}
	return dist;
}
