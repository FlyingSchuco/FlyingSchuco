#include "bsp.h"
MotionState *myRob;
Motor *motorLF, *motorLB, *motorRF, *motorRB;
Stepper *stepperFront;
Sonar *sonarF, *sonarB, *sonarL, *sonarR;
PID_Typedef *LF_PID;
PID_Typedef *LB_PID;
PID_Typedef *RF_PID;
PID_Typedef *RB_PID;

uint8_t FrontBuffer,BackBuffer;
ProtocolData *RxData;

void BSP_Init(void)
{
		
	LF_PID = PID_Init(POSITION,2.5,2,0,1,1000,0);
	LB_PID = PID_Init(POSITION,2.5,2,0,1,1000,0);
	RF_PID = PID_Init(POSITION,2.5,2,0,1,1000,0);
	RB_PID = PID_Init(POSITION,2.5,2,0,1,1000,0);
	//初始化4个电机
	motorLF = MotorInit(GPIOC,GPIO_PIN_7,
						GPIOC,GPIO_PIN_6,
						&htim1,
						&htim8, TIM_CHANNEL_2,
						&htim8, TIM_CHANNEL_1);
						if(motorLF==NULL) printf("Error: motorLF NULL\n");
	motorLB = MotorInit(GPIOC,GPIO_PIN_9,
						GPIOC,GPIO_PIN_8,
						&htim3,
						&htim8, TIM_CHANNEL_4,
						&htim8, TIM_CHANNEL_3);
						if(motorLB==NULL) printf("Error: motorLB NULL\n");
	motorRF = MotorInit(GPIOE,GPIO_PIN_6,
						GPIOE,GPIO_PIN_5,
						&htim2,
						&htim9, TIM_CHANNEL_2,
						&htim9, TIM_CHANNEL_1);
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
	
	//步进电机初始化
	stepperFront = StepperInit(STP_GPIO_Port,STP_Pin,DIR_GPIO_Port,DIR_Pin);
	
	//ATK-MPU6050设置
	
	printf("MPU6050 TEST\r\n");
    while(mpu_dmp_init())//MPU DMP初始化
	{
	    printf("MPU6050 Error!!!\r\n");
		HAL_Delay(500);
	}
    printf("MPU6050 OK\r\n");

	
	//定时时钟开启
	HAL_TIM_Base_Start(&htim6);
    __HAL_TIM_SET_COUNTER(&htim6,0x0000);
	HAL_TIM_Base_Start_IT(&htim7);
	
	//超声波初始化
	sonarF = SonarInit(SONARF_TRIG_GPIO_Port, SONARF_TRIG_Pin,
						SONARF_ECHO_GPIO_Port, SONARF_ECHO_Pin, 1);
	if(sonarF == NULL) printf("Error: sonarF NULL\n");
	
	sonarB = SonarInit(SONARB_TRIG_GPIO_Port, SONARB_TRIG_Pin,
						SONARB_ECHO_GPIO_Port, SONARB_ECHO_Pin, 2);
	if(sonarB == NULL) printf("Error: sonarB NULL\n");
	
	sonarL = SonarInit(SONARL_TRIG_GPIO_Port, SONARL_TRIG_Pin,
						SONARL_ECHO_GPIO_Port, SONARL_ECHO_Pin, 3);
	if(sonarL == NULL) printf("Error: sonarL NULL\n");
	
	sonarR = SonarInit(SONARR_TRIG_GPIO_Port, SONARR_TRIG_Pin,
						SONARR_ECHO_GPIO_Port, SONARR_ECHO_Pin, 4);
	if(sonarR == NULL) printf("Error: sonarR NULL\n");

	//保存openmv接受的数据
	RxData = (ProtocolData *)pvPortMalloc(sizeof(ProtocolData));

	//开启接受中断
	//HAL_UART_Receive_IT(&huart1, (uint8_t *)&FrontBuffer, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&FrontBuffer, 1);
	
	//初始化机器人姿态
	myRob = MotionStateInit();
	
}

