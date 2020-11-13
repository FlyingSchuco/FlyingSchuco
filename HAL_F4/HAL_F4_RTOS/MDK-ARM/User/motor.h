#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
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

#ifdef XY160
typedef struct
{
	GPIO_PIN IN1, IN2;
	GPIO_PIN EN;
	GPIO_PIN V_Meter;	//初始化时的引脚
	TIM_HandleTypeDef *TIM;			//初始化时的定时器
	uint16_t channel;				//初始化时的定时器
	uint32_t Speed;					//速度数据，无方向
	MotorState State;				//电机状态  停止，正转，反转
	uint32_t PWM;					//设定PWM
	int TargetSpeed;				//目标速度，有方向，正转为正
}Motor;
#endif

#ifdef MX1616
typedef struct
{
	GPIO_PIN IN1, IN2;
	GPIO_PIN V_Meter;	//初始化时的引脚
	TIM_HandleTypeDef *TIMA;			//初始化时的定时器
	uint16_t channel1;				//初始化时的定时器
	TIM_HandleTypeDef *TIMB;			//初始化时的定时器
	uint16_t channel2;				//初始化时的定时器
	uint32_t Speed;					//速度数据，无方向
	MotorState State;				//电机状态  停止，正转，反转
	uint32_t PWM;					//设定PWM
	int TargetSpeed;				//目标速度，有方向，正转为正
}Motor;
#endif

#ifdef XY160
Motor *MotorInit(	GPIO_TypeDef *IN1, 	uint16_t IN1_PIN, 
					GPIO_TypeDef *IN2, 	uint16_t IN2_PIN, 
					GPIO_TypeDef *EN, 	uint16_t EN_PIN,
					GPIO_TypeDef *V_Meter, 	uint16_t V_Meter_PIN,
					TIM_HandleTypeDef *TIM, uint16_t channel	);
#endif

#ifdef MX1616
Motor *MotorInit(	GPIO_TypeDef *IN1, 	uint16_t IN1_PIN, 
					GPIO_TypeDef *IN2, 	uint16_t IN2_PIN, 
					GPIO_TypeDef *V_Meter, 	uint16_t V_Meter_PIN,
					TIM_HandleTypeDef *TIMA, uint16_t channel1,
					TIM_HandleTypeDef *TIMB, uint16_t channel2	);
#endif

void MotorRun(Motor *motor, MotorState state, uint32_t PWM);
void MotorRunToTarget(Motor *motor, uint32_t PWM);
#endif
