#include "stepper.h"

Stepper *StepperInit(GPIO_TypeDef* STP_GPIOx,uint16_t STP_GPIO_Pin,GPIO_TypeDef* DIR_GPIOx,uint16_t DIR_GPIO_Pin,TIM_HandleTypeDef *htimx)
{
	Stepper *StepperNum = (Stepper *)malloc(sizeof(Stepper));
	StepperNum->STP_GPIOx = STP_GPIOx;
	StepperNum->STP_GPIO_Pin = STP_GPIO_Pin;
	StepperNum->DIR_GPIOx = DIR_GPIOx;
	StepperNum->DIR_GPIO_Pin = DIR_GPIO_Pin;
	StepperNum->htimx = htimx;
	StepperNum->pos = 0;
	return StepperNum;
}
void StepperRun(Stepper *StepperNum,int speed,int dir)
{
	int HalfPulseWidth = 0;
	if(speed) HalfPulseWidth = (int)112500/speed; // 900*1000/8/speed us
	else return ;
	if(dir == LEFT) 
	{
		StepperNum->pos += 1.8/8;
		HAL_GPIO_WritePin(StepperNum->DIR_GPIOx,StepperNum->DIR_GPIO_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(StepperNum->STP_GPIOx,StepperNum->STP_GPIO_Pin,GPIO_PIN_SET);
		gen_delay_us(HalfPulseWidth,StepperNum->htimx);
		HAL_GPIO_WritePin(StepperNum->STP_GPIOx,StepperNum->STP_GPIO_Pin,GPIO_PIN_RESET);
		gen_delay_us(HalfPulseWidth,StepperNum->htimx);
	}
	else if(dir == RIGHT)
	{
		StepperNum->pos -= 1.8/8;
		HAL_GPIO_WritePin(StepperNum->DIR_GPIOx,StepperNum->DIR_GPIO_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(StepperNum->STP_GPIOx,StepperNum->STP_GPIO_Pin,GPIO_PIN_SET);
		gen_delay_us(HalfPulseWidth,StepperNum->htimx);
		HAL_GPIO_WritePin(StepperNum->STP_GPIOx,StepperNum->STP_GPIO_Pin,GPIO_PIN_RESET);
		gen_delay_us(HalfPulseWidth,StepperNum->htimx);
	}
}
