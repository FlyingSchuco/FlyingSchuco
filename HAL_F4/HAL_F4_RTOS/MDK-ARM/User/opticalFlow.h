#ifndef __OPTICALFLOW_H
#define __OPTICALFLOW_H
#include "stm32f4xx_hal.h"
#include "delay.h"
#include "stdlib.h"
#include "srom.h"
#include "comm.h"

#define TSW 50  //写入后下一次操作的间隔时间
#define TSRAD 50  //读取地址后等待时间
#define TSRAD_MOT 75  //读取地址后等待时间
#define TSR 1  //读取后下一次操作的间隔时间
#define TBEXIT 4 //退出Burst模式
#define FrameRate 5000 //帧率
#define FramePeriod 1000000/FrameRate //一帧的时间

#define Product_ID   	0x00
#define Revision_ID		0x01
#define Motion 	     	0x02
#define Delta_X		 	0X03
#define Delta_Y		 	0X04
#define SQUAL		 	0x05
#define Pixel_Sum	 	0x06
#define Maximum_Pixel 	0x07
#define Configuration_bits	0x0a
#define Extended_Config     0x0b
#define Data_Out_Lower		0x0c
#define Data_Out_Upper		0x0d
#define Shutter_Lower		0x0e
#define Shutter_Upper		0x0f
#define Frame_Period_Lower	 0x10
#define Frame_Period_Upper	 0x11
#define	Motion_Clear	     0x12
#define	Frame_Capture	     0x13
#define	SROM_Enable		     0x14
#define Frame_Period_Max_Bound_Lower	0x19
#define Frame_Period_Max_Bound_Upper    0x1a
#define Frame_Period_Min_Bound_Lower    0x1b
#define Frame_Period_Min_Bound_Upper    0x1c
#define Shutter_Max_Bound_Lower         0x1d
#define Shutter_Max_Bound_Upper         0x1e
#define SROM_ID	         		0x1f
#define	Pixel_Burst	         	0x40
#define	Motion_Burst         	0x50
#define	SROM_Load	         	0x60

typedef enum
{
	ON,
	OFF
}NCS_State;

typedef enum
{
	AUTO,
	MAN
}NCS_MODE;
typedef struct
{
	uint8_t _Motion;
	uint8_t _DeltaX;
	uint8_t _DeltaY;
	uint8_t _SQUAL;
	uint8_t _Shutter_Upper;
	uint8_t _Shutter_Lower;
	uint8_t _Maxium_Pixel;
}BurstData;
extern SPI_HandleTypeDef hspi1;
extern const uint8_t Srom[1986];

void NCS(NCS_State state);
void WriteReg(uint8_t addr, uint8_t data,NCS_MODE state);
uint8_t ReadReg(uint8_t addr);
void SetFP(void);
void OPTF_Reset(void);
void SromReload(void);
void OpticalFlowInit(void);
void BurstRead(BurstData *data);
void GetOneFrame(uint8_t buffer[]);
#endif
