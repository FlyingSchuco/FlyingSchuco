#ifndef __OPTICALFLOW_H
#define __OPTICALFLOW_H
#include "stm32f4xx_hal.h"
#include "delay.h"
#define Product_ID		0x00
#define Motion			0x02
#define	DX				0X03
#define	DY				0X04
#define	SQUAL			0x05
#define	Pixel_Sum		0x06
#define	Configuration_bits		0x0a
#define	Extended_Config			0x0b
#define	Frame_Period_Lower		0x10
#define	Frame_Period_Upper		0x11
#define	Motion_Clear			0x12
#define	Frame_Capture			0x13
#define	SROM_Enable				0x14
#define	Frame_Period_Max_Bound_Lower	0x19
#define	Frame_Period_Max_Bound_Upper    0x1a
#define	Frame_Period_Min_Bound_Lower    0x1b
#define	Frame_Period_Min_Bound_Upper    0x1c
#define	Shutter_Max_Bound_Lower         0x1d
#define	Shutter_Max_Bound_Upper         0x1e
#define	SROM_ID			0x1f
#define	Pixel_Burst		0x40
#define	Motion_Burst	0x50
#define	SROM_Load		0x60
#define  ON_CS()    {HAL_GPIO_WritePin(GPIOA,15,GPIO_PIN_RESET);}
#define  OFF_CS()   {HAL_GPIO_WritePin(GPIOA,15,GPIO_PIN_SET);}

extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim14;

typedef struct
{
	uint8_t state;
	int8_t deltaX;
	int8_t deltaY;
}OPFMotionData;
typedef struct
{
	uint8_t state[2];
	uint8_t deltaX[2];
	uint8_t deltaY[2];
	uint8_t configuration[2];		
}OPFRxData;
uint8_t SPI_SendReceive(uint8_t data);
uint8_t read_register(uint8_t address);
void GetMotionData(SPI_HandleTypeDef *hspix,OPFMotionData *buffer);
#endif
