#include "motor.h"
#include "delay.h"

void Motor0Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_InitStructure.GPIO_Pin);
}

void Motor1Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_InitStructure.GPIO_Pin);
}

void Motor2Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_InitStructure.GPIO_Pin);
}

void Motor3Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_InitStructure.GPIO_Pin);
}

void MotorInit(void)
{
	Motor0Init();
	Motor1Init();
	Motor2Init();
	Motor3Init();
}

void Motor0Run(void)
{
	MOTOR0in1 = 1;
	MOTOR0in2 = 0;
	EN0 = 1;
}

void Motor0RunRV(void)
{
	MOTOR0in1 = 0;
	MOTOR0in2 = 1;
	EN0 = 1;
}

void Motor1Run(void)
{
	MOTOR1in1 = 1;
	MOTOR1in2 = 0;
	EN1 = 1;
}

void Motor1RunRV(void)
{
	MOTOR1in1 = 0;
	MOTOR1in2 = 1;
	EN1 = 1;
}

void Motor2Run(void)
{
	MOTOR2in1 = 1;
	MOTOR2in2 = 0;
	EN2 = 1;
}

void Motor2RunRV(void)
{
	MOTOR2in1 = 0;
	MOTOR2in2 = 1;
	EN2 = 1;
}

void Motor3Run(void)
{
	MOTOR3in1 = 1;
	MOTOR3in2 = 0;
	EN3 = 1;
}

void Motor3RunRV(void)
{
	MOTOR3in1 = 0;
	MOTOR3in2 = 1;
	EN3 = 1;
}
	
void MotorSelfCheck(void)
{
	Motor0Run();
	delay_ms(500);
	AllBrake();
	delay_ms(500);
	
	Motor1Run();
	delay_ms(500);
	AllBrake();
	delay_ms(500);
	
	Motor2Run();
	delay_ms(500);
	AllBrake();
	delay_ms(500);
	
	Motor3Run();
	delay_ms(500);
	AllBrake();
	delay_ms(500);
}

void AllBrake()
{
	MOTOR0in1 = 0;
	MOTOR0in2 = 0;
	MOTOR1in1 = 0;
	MOTOR1in2 = 0;
	MOTOR2in1 = 0;
	MOTOR2in2 = 0;
	MOTOR3in1 = 0;
	MOTOR3in2 = 0;
}
