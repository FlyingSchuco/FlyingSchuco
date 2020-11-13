#include "bsp.h"
MotionState *MyRob;
Motor *motorLF, *motorLB, *motorRF, *motorRB;

void BSP_Init(void)
{
	//OLED设置
	/*
	OLED_Init();
	OLED_Print(0, 0, "LF  LB  RF  RB",TYPE16X16,TYPE8X16);
	*/
	//ATK-MPU6050设置
	
	printf("MPU6050 TEST\r\n");
    while(mpu_dmp_init())//MPU DMP初始化
	{
	    printf("MPU6050 Error!!!\r\n");
		HAL_Delay(500);
	}
    printf("MPU6050 OK\r\n");
	
	//MPU6050设置
	/*
	printf("MPU Init");
	MPU_Init(FAST);
	HAL_UART_Receive_IT(&huart4,&tmpBuf,1);	// 开启串口中断
	printf("MPU OK!");
	*/
	//初始化机器人姿态
	MyRob = MotionStateInit();
	
	//输出PMW开启
	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);	//LF_A
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);	//LF_B
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	//LB_A
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	//LB_B
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);	//RF_A
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);	//RF_B
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);	//RB_A
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);	//RB_B
	
	//输入捕获中断开启
	/*
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);
	*/
	
	//初始化4个电机
	motorLF = MotorInit(GPIOC,GPIO_PIN_6,
						GPIOC,GPIO_PIN_7,
						GPIOF,GPIO_PIN_0,
						&htim8, TIM_CHANNEL_1,
						&htim8, TIM_CHANNEL_2);
						if(motorLF==NULL) printf("Error: motorLF NULL\n");
	motorLB = MotorInit(GPIOC,GPIO_PIN_8,
						GPIOC,GPIO_PIN_9,
						GPIOF,GPIO_PIN_0,
						&htim8, TIM_CHANNEL_3,
						&htim8, TIM_CHANNEL_4);
						if(motorLB==NULL) printf("Error: motorLB NULL\n");
	motorRF = MotorInit(GPIOE,GPIO_PIN_5,
						GPIOE,GPIO_PIN_6,
						GPIOF,GPIO_PIN_0,
						&htim9, TIM_CHANNEL_1,
						&htim9, TIM_CHANNEL_2);
						if(motorRF==NULL) printf("Error: motorRF NULL\n");
	motorRB = MotorInit(GPIOB,GPIO_PIN_14,
						GPIOB,GPIO_PIN_15,
						GPIOF,GPIO_PIN_0,
						&htim12, TIM_CHANNEL_1,
						&htim12, TIM_CHANNEL_2);
						if(motorRB==NULL) printf("Error: motorRB NULL\n");
	//设置初始速度
	MotorRun(motorLF, STOP, 80);
	MotorRun(motorLB, STOP, 80);
	MotorRun(motorRF, STOP, 80);
	MotorRun(motorRB, STOP, 80);
	
}

