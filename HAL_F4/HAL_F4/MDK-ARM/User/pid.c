#include "pid.h"
PID_Data* PID_Init(float Expection, float kp, float ki, float kd)
{
	PID_Data* PID_x = (PID_Data *)malloc(sizeof(PID_Data));
	PID_x->Expection = Expection;
	PID_x->kp = kp;
	PID_x->ki = ki;
	PID_x->kd = kd;
	return PID_x;
}
float CMN_PID(PID_Data *PID_x,float err)
{
	float ThisErr = err-PID_x->Expection;
	PID_x->integral += ThisErr;
	float LastErr = PID_x->LastErr;
	PID_x->LastErr = ThisErr;
	return (PID_x->kp)*ThisErr + (PID_x->ki)*(PID_x->integral) + (PID_x->kd)*(ThisErr-LastErr);
}
