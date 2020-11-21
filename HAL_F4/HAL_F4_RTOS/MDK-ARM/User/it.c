#include "it.h"

/*
unsigned char flagAC = 0;	//加速度
unsigned char flagOG = 0;	//角速度
unsigned char flagAG = 0;	//角度
unsigned char tmpBuf = 0;	//串口缓冲区
unsigned char RX_Buf[11], RX_Buf_AC[11], RX_Buf_OG[11], RX_Buf_AG[11];	//数据包缓冲区
unsigned char counter=0;	//计数器


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)	// 判断是由哪个串口触发的中断
	{
		RX_Buf[counter] = tmpBuf;
		if(RX_Buf[0]==0x55)
		{
			counter++;
		}
		if(counter == 11)
		{
			counter = 0;
			switch(RX_Buf[1])
			{
				case 0x51:
					flagAC = 1;
					memcpy(RX_Buf_AC,RX_Buf,11);
					break;
				case 0x52:
					flagOG = 1;
					memcpy(RX_Buf_OG,RX_Buf,11);
					break;
				case 0x53:
					flagAG = 1;
					memcpy(RX_Buf_AG,RX_Buf,11);
					break;
				default:
					break;
			}
		}
		HAL_UART_Receive_IT(&huart4,&tmpBuf,1);		// 重新使能串口4接收中断
	}
}
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_15)
	{
		if(HAL_GPIO_ReadPin(sonarF->GPIO_Echo,sonarF->PIN_Echo) == GPIO_PIN_SET)
		{
			sonarF->startTime = __HAL_TIM_GET_COUNTER(&htim6);
		}
		else
		{
			sonarF->endTime = __HAL_TIM_GET_COUNTER(&htim6);
			sonarF->state = HAL_OK;
		}
	}
}
