#include "pid.h"

#define ABS(x)		((x>0)? x: -x) 

/*初版PID 死区、分段*/

#ifdef V1

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
#endif

/*
第二版PID，基于RM
特性：死区，积分饱和，输出饱和
*/
#ifdef V2

static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	int16_t  target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;             
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}


	
static float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		else if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		pid->output = pid->pout + pid->iout + pid->dout;
		

		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		else if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}


	return pid->output;
}

void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
#endif
