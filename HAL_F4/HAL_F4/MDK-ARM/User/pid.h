#ifndef __PID_H
#define __PID_H
#include "stdlib.h"
#include "math.h"
#define KP 1
#define KI 2
#define KD 3
typedef struct
{
	float Expection;
	float kp,ki,kd;
	float LastErr;
	float integral;
	float output;
	float deadband;
}PID_Data;
PID_Data* PID_Init(float Expection, float kp, float ki, float kd, float deadband);
float CMN_PID(PID_Data *PID_x,float err);
float PieceWise_PID(PID_Data *PID_x,float err);
float PIDGetGain(float err, int k);
#endif
