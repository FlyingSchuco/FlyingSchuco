#include "pid.h"
PID_Data* PID_Init(float Expection, float kp, float ki, float kd, float deadband)
{
	PID_Data* PID_x = (PID_Data *)malloc(sizeof(PID_Data));
	PID_x->Expection = Expection;
	PID_x->kp = kp;
	PID_x->ki = ki;
	PID_x->kd = kd;
	PID_x->deadband = deadband;
	return PID_x;
}
float CMN_PID(PID_Data *PID_x,float err)
{
	float ThisErr = err-PID_x->Expection;
	if(fabs(ThisErr)>PID_x->deadband)
	{
		PID_x->integral += ThisErr;
		float LastErr = PID_x->LastErr;
		PID_x->LastErr = ThisErr;
		return (PID_x->kp)*ThisErr + (PID_x->ki)*(PID_x->integral) + (PID_x->kd)*(ThisErr-LastErr);
	}
	else 
	{
		PID_x->LastErr = ThisErr;
		return 0;
	}
}
float PieceWise_PID(PID_Data *PID_x,float err)
{
	float ThisErr = err-PID_x->Expection;
	if(fabs(ThisErr)>PID_x->deadband)
	{
		PID_x->integral += ThisErr;
		float LastErr = PID_x->LastErr;
		PID_x->LastErr = ThisErr;
		return PIDGetGain(ThisErr,KP)*ThisErr + PIDGetGain(ThisErr,KI)*(PID_x->integral) + PIDGetGain(ThisErr,KD)*(ThisErr-LastErr);
	}
	else 
	{
		PID_x->LastErr = ThisErr;
		return 0;
	}
}
float PIDGetGain(float err, int k)
{
	switch(k)
	{
		case KP:
			if(fabs(err)<20)
				return 0.8;
			else return 6;
		case KI:
			if(fabs(err)<20)
				return 0.01;
			else return 0.1;
		case KD:
			if(fabs(err)<20)
				return 0.8;
			else return 0.8;
	}
	return -1;
}
