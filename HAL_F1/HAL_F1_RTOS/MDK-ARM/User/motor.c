#include "motor.h"

Motor *MotorInit(	GPIO_TypeDef *IN1,uint16_t IN1_PIN, 
					GPIO_TypeDef *IN2,uint16_t IN2_PIN, 
					GPIO_TypeDef *EN,uint16_t EN_PIN, 
					GPIO_TypeDef *V_Meter, uint16_t V_Meter_PIN,
					TIM_HandleTypeDef *TIM, uint16_t channel)
{
	Motor *MT;
	MT = (Motor *)pvPortMalloc(sizeof(Motor));
	MT->IN1.GPIO = IN1;
	MT->IN1.PIN = IN1_PIN;
	MT->IN2.GPIO = IN2;
	MT->IN2.PIN = IN2_PIN;
	MT->EN.GPIO = EN;
	MT->EN.PIN = EN_PIN;
	MT->V_Meter.GPIO = V_Meter;
	MT->V_Meter.PIN = V_Meter_PIN;
	MT->TIM = TIM;
	MT->channel = channel;
	MT->State = STOP;
	MT->Speed = 0;
	MT->PWM = 0;
	MT->TargetSpeed = 0;
	return MT;
}

void MotorRun(Motor *motor, MotorState state, uint32_t PWM)
{
	motor->PWM = PWM;
	motor->State = state;
	__HAL_TIM_SET_COMPARE(motor->TIM, motor->channel, PWM);
	switch(motor->State)
	{
		case STOP:
			HAL_GPIO_WritePin(motor->IN1.GPIO, motor->IN1.PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->IN2.GPIO, motor->IN2.PIN, GPIO_PIN_RESET);
			break;
		case FW:
			HAL_GPIO_WritePin(motor->IN1.GPIO, motor->IN1.PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->IN2.GPIO, motor->IN2.PIN, GPIO_PIN_SET);
			break;
		case RV:
			HAL_GPIO_WritePin(motor->IN1.GPIO, motor->IN1.PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->IN2.GPIO, motor->IN2.PIN, GPIO_PIN_RESET);
			break;
	}
	return ;
}

void MotorRunToTarget(Motor *motor, uint32_t PWM)
{
	motor->PWM = PWM;
	if(motor->TargetSpeed>0)
	{
		motor->State = FW;
	}
	else if(motor->TargetSpeed<0)
	{
		motor->State = RV;
	}
	else 
	{
		motor->State = STOP;
	}
	__HAL_TIM_SET_COMPARE(motor->TIM, motor->channel, PWM);
	switch(motor->State)
	{
		case STOP:
			HAL_GPIO_WritePin(motor->IN1.GPIO, motor->IN1.PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->IN2.GPIO, motor->IN2.PIN, GPIO_PIN_RESET);
			break;
		case FW:
			HAL_GPIO_WritePin(motor->IN1.GPIO, motor->IN1.PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->IN2.GPIO, motor->IN2.PIN, GPIO_PIN_SET);
			break;
		case RV:
			HAL_GPIO_WritePin(motor->IN1.GPIO, motor->IN1.PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->IN2.GPIO, motor->IN2.PIN, GPIO_PIN_RESET);
			break;
	}
	return ;
}

