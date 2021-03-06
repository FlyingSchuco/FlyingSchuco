#ifndef __COMM_H
#define __COMM_H
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f1xx_hal.h"
#include "stepper.h"
#include "pid.h"
#define PTCLen 8
#define DEBUG
typedef enum
{
	RED,
	GREEN,
	BLUE
}COLOR;

typedef struct
{
	char color;
	short dx;
	short dy;
}ProtocolData;

extern uint8_t IRQBuffer;
extern UART_HandleTypeDef huart1,huart2;
extern ProtocolData *RxData;
extern PID_Typedef *PID_Front; 
extern Stepper *StepperFront;
/*
openmv 通讯协议
0x1D 0xD1 (u8)color (short)dx (short)dy 0xED
1    1    1          2         2        1
--------------------------------------------
共八位
*/
//int fputc(int ch, FILE *f);
void SerialTransmission(UART_HandleTypeDef *ToUart);
void RxDecode(uint8_t RxBuffer[]);
void RxConfirm(uint8_t Data);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif
