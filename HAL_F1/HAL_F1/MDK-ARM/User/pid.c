#include "pid.h"
PID_Typedef *PID_Init(
				PID_Mode mode,
				float target,
				float kp,
				float ki,
				float kd,
				float deadband)
{
	PID_Typedef *PID_Ptr = (PID_Typedef *)malloc(sizeof(PID_Typedef));
	PID_Ptr->mode = mode;
	PID_Ptr->target = target;
	PID_Ptr->kp = kp;
	PID_Ptr->ki = ki;
	PID_Ptr->kd = kd;
	PID_Ptr->iout = 0;
	PID_Ptr->deadband = deadband;
	return PID_Ptr;
}

float PID_Calc(PID_Typedef *PID_Ptr,float measure)
{
	PID_Ptr->measure = measure;
	PID_Ptr->prvErr = PID_Ptr->lastErr;
	PID_Ptr->lastErr = PID_Ptr->err;
	PID_Ptr->err = PID_Ptr->target - PID_Ptr->measure;
	//死区控制
	if(ABS(PID_Ptr->err) > PID_Ptr->deadband)
	{
		//位置型
		if(PID_Ptr->mode == POSITION)
		{
			PID_Ptr->pout = PID_Ptr->kp * PID_Ptr->err;
			PID_Ptr->iout += PID_Ptr->ki * PID_Ptr->err;
			//防止积分饱和
			PID_Ptr->iout = CEIL(PID_Ptr->iout);
			PID_Ptr->iout = FLOOR(PID_Ptr->iout);
			PID_Ptr->dout = PID_Ptr->kd * (PID_Ptr->err - PID_Ptr->lastErr);
		}
		//增量型
		else if(PID_Ptr->mode == DELTA)
		{
			PID_Ptr->pout = PID_Ptr->kp * (PID_Ptr->err-PID_Ptr->lastErr);
			PID_Ptr->iout = PID_Ptr->ki * PID_Ptr->err;
			PID_Ptr->dout = PID_Ptr->kd * (PID_Ptr->err - 2.0*PID_Ptr->lastErr + PID_Ptr->prvErr);
		}
		PID_Ptr->output = PID_Ptr->pout + PID_Ptr->iout + PID_Ptr->dout;
	}
	return PID_Ptr->output;
}
