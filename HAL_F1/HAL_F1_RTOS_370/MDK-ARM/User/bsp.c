#include "bsp.h"
MotionState *MyRob;
Motor *motorLF, *motorLB, *motorRF, *motorRB;
void BSP_Init(void)
{
	//
	/*
	OLED_Init();
	OLED_Print(0, 0, "LF  LB  RF  RB",TYPE16X16,TYPE8X16);
	*/
	//
	/*
	printf("MPU6050 TEST\r\n");
    while(mpu_dmp_init())//MPU DMP初始化
	{
	    printf("MPU6050 Error!!!\r\n");
		HAL_Delay(500);
	}
    printf("MPU6050 OK\r\n");
	*/
	//
	MyRob = MotionStateInit();
	
	//
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	//
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);
	
	
	
	motorLF = MotorInit(GPIOC,GPIO_PIN_11,
						GPIOC,GPIO_PIN_12,
						GPIOB,GPIO_PIN_11,
						GPIOC,GPIO_PIN_13,
						&htim2, TIM_CHANNEL_4);
						if(motorLF==NULL) printf("Error: motorLF NULL\n");
	motorLB = MotorInit(GPIOC,GPIO_PIN_1,
						GPIOD,GPIO_PIN_2,
						GPIOB,GPIO_PIN_10,
						GPIOC,GPIO_PIN_0,
						&htim2, TIM_CHANNEL_3);
						if(motorLB==NULL) printf("Error: motorLB NULL\n");
	motorRF = MotorInit(GPIOB,GPIO_PIN_15,
						GPIOB,GPIO_PIN_14,
						GPIOB,GPIO_PIN_9,
						GPIOC,GPIO_PIN_10,
						&htim4, TIM_CHANNEL_4);
						if(motorRF==NULL) printf("Error: motorRF NULL\n");
	motorRB = MotorInit(GPIOB,GPIO_PIN_13,
						GPIOB,GPIO_PIN_12,
						GPIOB,GPIO_PIN_8,
						GPIOC,GPIO_PIN_3,
						&htim4, TIM_CHANNEL_3);
						if(motorRB==NULL) printf("Error: motorRB NULL\n");
	
	MotorRun(motorLF, STOP, 80);
	MotorRun(motorLB, STOP, 80);
	MotorRun(motorRF, STOP, 80);
	MotorRun(motorRB, STOP, 80);
	
}

