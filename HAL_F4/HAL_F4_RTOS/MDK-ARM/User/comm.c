#include "comm.h"

int state = 0;					//匹配状态
uint8_t UART2_RxBuff[128];		//接收缓冲
uint8_t UART2_Rx_Cnt = 0;		//接收缓冲计数

/*
int fputc(int ch, FILE *f) 
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
*/

void SerialTransmission(UART_HandleTypeDef *ToUart)
{
	HAL_UART_Transmit(ToUart,(uint8_t *)&IRQBuffer,1,0xFFFF);
}

void RxDecode(uint8_t RxBuffer[])
{
	if(RxBuffer[2]==0xFF)
	{
		#ifdef INFO
		printf("color = RED\n");
		#endif
		RxData->color = RED;
	}
	else if(RxBuffer[2]==0x3F)
	{
		#ifdef INFO
		printf("color = GREEN\n");
		#endif
		RxData->color = GREEN;
	}
	else if(RxBuffer[2]==0x0F)
	{
		#ifdef INFO
		printf("color = BLUE\n");
		#endif 
		RxData->color = BLUE;
	}
	else 
	{
		#ifdef WARNING
		printf("Warning:An Unknown color!");
		#endif
	}
	RxData->dx = RxBuffer[3]<<8 | RxBuffer[4];
	RxData->dy = RxBuffer[5]<<8 | RxBuffer[6];
	#ifdef INFO
	printf("dx = %d,dy = %d\n",RxData->dx,RxData->dy);
	#endif
	#ifdef DEBUG
		printf("%d\n",RxData->dx);
	#endif
	StepperAutoRotate(StepperFront,PID_Calc(PID_Front,0,RxData->dx));
}

void RxConfirm(uint8_t Data)
{
	//第一帧
	if(state == 0 && Data == 0x1D)
	{
		state = 1;
		UART2_RxBuff[UART2_Rx_Cnt++] = Data;
	}
	//第二帧
	else if(state == 1 && Data == 0xD1)
	{
		state = 2;
		UART2_RxBuff[UART2_Rx_Cnt++] = Data;
	}
	else if(state == 2)
	{
		UART2_RxBuff[UART2_Rx_Cnt++] = Data;
		if(UART2_Rx_Cnt>=PTCLen-1)
		{
			state = 3;
		}
	}
	//结束帧确认，否则为无效帧
	else if(state == 3)
	{
		if(Data == 0xED)
		{
			UART2_RxBuff[UART2_Rx_Cnt++] = Data;
			RxDecode(UART2_RxBuff);
			state = 0;
			UART2_Rx_Cnt = 0;
		}
		else 
		{
			state = 0;
			memset(UART2_RxBuff,0,128);
			#ifdef WARNING
			printf("Warning: Receive a frame without ending\n");
			#endif
		}
	}
	//防止意外
	else 
	{
		state = 0;
		UART2_Rx_Cnt = 0;
		memset(UART2_RxBuff,0,128);
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	#ifdef ST
	//串口透传
	SerialTransmission(&huart1);
	#else
	//解析数据
	RxConfirm(IRQBuffer);
	#endif
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&IRQBuffer, 1);   //再开启接收中断
}

