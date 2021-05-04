#ifndef __MOTION_H
#define __MOTION_H
#include "freeRTOS.h"
/*
这个文件介绍机体坐标系
正朝向为机体x轴正方向，
右朝向为机体y轴正方向，
指向地面为机体z轴正方向，
omega俯视顺时针为正方向。


*/

typedef struct 
{
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度
	float x;						//地面坐标系x轴
	float y;						//地面坐标系y轴
	float dF;
	float dB;
	float dL;
	float dR;
	uint8_t target;				//是否发现目标
	short pixel_dx;				//像素x偏移
	short pixel_dy;				//像素y偏移
}MotionState;
MotionState *MotionStateInit(void);
void Cruise(void);

#endif
