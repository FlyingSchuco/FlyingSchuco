#include "stepper.h"

Stepper *StepperInit(GPIO_TypeDef* STP_GPIOx,uint16_t STP_GPIO_Pin,GPIO_TypeDef* DIR_GPIOx,uint16_t DIR_GPIO_Pin)
{
	Stepper *StepperPTR = (Stepper *)malloc(sizeof(Stepper));
	StepperPTR->STP_GPIOx = STP_GPIOx;
	StepperPTR->STP_GPIO_Pin = STP_GPIO_Pin;
	StepperPTR->DIR_GPIOx = DIR_GPIOx;
	StepperPTR->DIR_GPIO_Pin = DIR_GPIO_Pin;
	StepperPTR->pos = 0;
	return StepperPTR;
}

void StepperRun(Stepper *StepperPTR,int speed,int dir)
{
	int HalfPulseWidth = 0;
	if(speed) HalfPulseWidth = (int)112500/speed; // 900*1000/8/speed us
	else return ;
	if(dir == LEFT) 
	{
		StepperPTR->pos += 1.8/8;
		HAL_GPIO_WritePin(StepperPTR->DIR_GPIOx,StepperPTR->DIR_GPIO_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_SET);
		gen_delay_us(HalfPulseWidth);
		HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_RESET);
		gen_delay_us(HalfPulseWidth);
	}
	else if(dir == RIGHT)
	{
		StepperPTR->pos -= 1.8/8;
		HAL_GPIO_WritePin(StepperPTR->DIR_GPIOx,StepperPTR->DIR_GPIO_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_SET);
		gen_delay_us(HalfPulseWidth);
		HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_RESET);
		gen_delay_us(HalfPulseWidth);
	}
}

void StepperRotate(Stepper *StepperPTR, float angle, int dir)
{
	int steps = (int)ABS((angle/1.8*8.0));
	if(dir == LEFT) 
	{
		while(steps--)
		{
			StepperPTR->pos += 1.8/8.0;
			HAL_GPIO_WritePin(StepperPTR->DIR_GPIOx,StepperPTR->DIR_GPIO_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_SET);
			gen_delay_us(PULSE_US);
			HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_RESET);
			gen_delay_us(PULSE_US);
		}
	}
	else if(dir == RIGHT)
	{
		while(steps--)
		{
			StepperPTR->pos -= 1.8/8.0;
			HAL_GPIO_WritePin(StepperPTR->DIR_GPIOx,StepperPTR->DIR_GPIO_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_SET);
			gen_delay_us(PULSE_US);
			HAL_GPIO_WritePin(StepperPTR->STP_GPIOx,StepperPTR->STP_GPIO_Pin,GPIO_PIN_RESET);
			gen_delay_us(PULSE_US);
		}
	}
}

void StepperAutoRotate(Stepper *StepperPTR, int angle)
{
	if(angle>=0)
	{
		StepperRotate(StepperPTR,angle,LEFT);
	}
	else
	{
		StepperRotate(StepperPTR,-angle,RIGHT);
	}
}

