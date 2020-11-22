#include "motor.h"


#ifdef MX1616
//电机初始化函数
Motor *MotorInit(	GPIO_TypeDef *IN1, 	uint16_t IN1_PIN, 
					GPIO_TypeDef *IN2, 	uint16_t IN2_PIN, 
					TIM_HandleTypeDef *Timer_Encoder,
					TIM_HandleTypeDef *TIMA, uint16_t channel1,
					TIM_HandleTypeDef *TIMB, uint16_t channel2	)
{
	Motor *MT;
	MT = (Motor *)pvPortMalloc(sizeof(Motor));
	MT->IN1.GPIO = IN1;
	MT->IN1.PIN = IN1_PIN;
	MT->IN2.GPIO = IN2;
	MT->IN2.PIN = IN2_PIN;
	MT->TIMA = TIMA;
	MT->channel1 = channel1;
	MT->TIMB = TIMB;
	MT->channel2 = channel2;
	MT->State = STOP;
	MT->Speed = 0;
	MT->PWM = 0;
	MT->TargetSpeed = 0;
	MT->Timer_Encoder = Timer_Encoder;
	//输出PMW开启
	HAL_TIM_PWM_Start(MT->TIMA,MT->channel1);
	HAL_TIM_PWM_Start(MT->TIMB,MT->channel2);
	//编码器模式开启
	HAL_TIM_Encoder_Start((MT->Timer_Encoder), TIM_CHANNEL_ALL);
	return MT;
}
#endif

//电机运行函数，需手动指定旋转方向，设置PWM值
void MotorRun(Motor *motor, MotorState state, uint32_t PWM)
{
	motor->PWM = PWM;
	motor->State = state;
	#ifdef MX1616
	switch(motor->State)
	{
		case STOP:
			__HAL_TIM_SET_COMPARE(motor->TIMA, motor->channel1, 0);
			__HAL_TIM_SET_COMPARE(motor->TIMB, motor->channel2, 0);
			break;
		case FW:
			__HAL_TIM_SET_COMPARE(motor->TIMA, motor->channel1, 0);
			__HAL_TIM_SET_COMPARE(motor->TIMB, motor->channel2, PWM);
			break;
		case RV:
			__HAL_TIM_SET_COMPARE(motor->TIMA, motor->channel1, PWM);
			__HAL_TIM_SET_COMPARE(motor->TIMB, motor->channel2, 0);
			break;
	}
	#endif
	return ;
}

//电机运行函数，根据设定速度，决定旋转方向，设置PWM值
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
	#ifdef MX1616
	switch(motor->State)
	{
		case STOP:
			__HAL_TIM_SET_COMPARE(motor->TIMA, motor->channel1, 0);
			__HAL_TIM_SET_COMPARE(motor->TIMB, motor->channel2, 0);
			break;
		case FW:
			__HAL_TIM_SET_COMPARE(motor->TIMA, motor->channel1, 0);
			__HAL_TIM_SET_COMPARE(motor->TIMB, motor->channel2, PWM);
			break;
		case RV:
			__HAL_TIM_SET_COMPARE(motor->TIMA, motor->channel1, PWM);
			__HAL_TIM_SET_COMPARE(motor->TIMB, motor->channel2, 0);
			break;
	}
	#endif
	return ;
}

void MotorSpeedMeasure(Motor *motor)
{
	motor->enc = (__HAL_TIM_GET_COUNTER((motor->Timer_Encoder)));
	motor->Speed = motor->enc-motor->enc_old;
	motor->enc_old=motor->enc;
	if (motor->Speed > 2000)
		motor->Speed -= 5000;
	else if (motor->Speed <= -2000)
		motor->Speed += 5000;
	motor->Speed = (int)(fabs((float)motor->Speed)*100.0f/385.0f/4.0f*60.0f);	//rpm
}

