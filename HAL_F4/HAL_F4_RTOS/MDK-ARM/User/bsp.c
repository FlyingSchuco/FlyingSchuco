#include "bsp.h"
MotionState *myRob;
Motor *motorLF, *motorLB, *motorRF, *motorRB;
Stepper *stepperFront;
Sonar *sonarF, *sonarB, *sonarL, *sonarR;

uint8_t FrontBuffer,BackBuffer;
ProtocolData *RxData;

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
	
	
	//定时时钟开启
	HAL_TIM_Base_Start(&htim6);
    __HAL_TIM_SET_COUNTER(&htim6,0x0000);
	
	//超声波初始化
	sonarF = SonarInit(SONARF_TRIG_GPIO_Port, SONARF_TRIG_Pin,
						SONARF_ECHO_GPIO_Port, SONARF_ECHO_Pin);
	if(sonarF == NULL) printf("Error: sonarF NULL\n");
	//保存openmv接受的数据
	RxData = (ProtocolData *)pvPortMalloc(sizeof(ProtocolData));

	//开启接受中断
	//HAL_UART_Receive_IT(&huart1, (uint8_t *)&FrontBuffer, 1);
	//HAL_UART_Receive_IT(&huart2, (uint8_t *)&BackBuffer, 1);
	
	//初始化机器人姿态
	myRob = MotionStateInit();
	
	
	//初始化4个电机
	motorLF = MotorInit(GPIOC,GPIO_PIN_6,
						GPIOC,GPIO_PIN_7,
						&htim1,
						&htim8, TIM_CHANNEL_1,
						&htim8, TIM_CHANNEL_2);
						if(motorLF==NULL) printf("Error: motorLF NULL\n");
	motorLB = MotorInit(GPIOC,GPIO_PIN_8,
						GPIOC,GPIO_PIN_9,
						&htim3,
						&htim8, TIM_CHANNEL_3,
						&htim8, TIM_CHANNEL_4);
						if(motorLB==NULL) printf("Error: motorLB NULL\n");
	motorRF = MotorInit(GPIOE,GPIO_PIN_5,
						GPIOE,GPIO_PIN_6,
						&htim2,
						&htim9, TIM_CHANNEL_1,
						&htim9, TIM_CHANNEL_2);
						if(motorRF==NULL) printf("Error: motorRF NULL\n");
	motorRB = MotorInit(GPIOB,GPIO_PIN_14,
						GPIOB,GPIO_PIN_15,
						&htim4,
						&htim12, TIM_CHANNEL_1,
						&htim12, TIM_CHANNEL_2);
						if(motorRB==NULL) printf("Error: motorRB NULL\n");
	//设置初始速度
	MotorRun(motorLF, STOP, 0);
	MotorRun(motorLB, STOP, 0);
	MotorRun(motorRF, STOP, 0);
	MotorRun(motorRB, STOP, 0);
	
}

