#ifndef __PID_H
#define __PID_H
#include "stdlib.h"
typedef struct
{
	float Expection;
	float kp,ki,kd;
	float LastErr;
	float integral;
	float output;
}PID_Data;
PID_Data* PID_Init(float Expection, float kp, float ki, float kd);
float CMN_PID(PID_Data *PID_x,float err);
#endif
