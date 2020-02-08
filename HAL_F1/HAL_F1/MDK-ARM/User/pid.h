#ifndef __PID_H
#define __PID_H
#include "stdlib.h"
#define MaxIntegral 45
#define CEIL(x) (x>MaxIntegral)?MaxIntegral:x
#define FLOOR(x) (x<-MaxIntegral)?-MaxIntegral:x
#define ABS(x) (x>0)?x:-x
typedef enum
{
	POSITION,
	DELTA
}PID_Mode;

typedef struct
{
	PID_Mode mode;
	float target;
	float measure;
	
	float kp;
	float ki;
	float kd;
	
	float err;
	float lastErr;
	float prvErr;
	
	float pout;
	float iout;
	float dout;
	float output;
	
	float deadband;
}PID_Typedef;

PID_Typedef *PID_Init(
				PID_Mode mode,
				float target,
				float kp,
				float ki,
				float kd,
				float deadband);
				
float PID_Calc(PID_Typedef *PID_Ptr,float measure);
#endif
