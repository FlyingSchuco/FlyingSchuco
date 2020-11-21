#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "math.h"
#define MX1616


typedef enum 
{
	STOP=0, FW, RV
}MotorState;


typedef struct 
{
	GPIO_TypeDef *GPIO; 
	uint16_t PIN;
}GPIO_PIN;


#ifdef MX1616
typedef struct
{
	GPIO_PIN IN1, IN2;
	TIM_HandleTypeDef *TIMA;			//初始化时的定时器
	uint16_t channel1;				//初始化时的定时器
	TIM_HandleTypeDef *TIMB;			//初始化时的定时器
	uint16_t channel2;				//初始化时的定时器
	int Speed;					//速度数据，有方向
	MotorState State;				//电机状态  停止，正转，反转
	uint32_t PWM;					//设定PWM
	int TargetSpeed;				//目标速度，有方向，正转为正
	TIM_HandleTypeDef *Timer_Encoder;	//编码器模式
	int enc;							//编码器计数值
	int enc_old;						//编码器旧计数值
}Motor;
#endif

#ifdef MX1616
Motor *MotorInit(	GPIO_TypeDef *IN1, 	uint16_t IN1_PIN, 
					GPIO_TypeDef *IN2, 	uint16_t IN2_PIN, 
					TIM_HandleTypeDef *Timer_Encoder,
					TIM_HandleTypeDef *TIMA, uint16_t channel1,
					TIM_HandleTypeDef *TIMB, uint16_t channel2	);
#endif

void MotorRun(Motor *motor, MotorState state, uint32_t PWM);
void MotorRunToTarget(Motor *motor, uint32_t PWM);
void MotorSpeedMeasure(Motor *motor);

#endif
