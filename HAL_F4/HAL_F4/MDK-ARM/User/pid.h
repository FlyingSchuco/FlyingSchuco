#ifndef __PID_H
#define __PID_H
#include "stdlib.h"
#include "math.h"

#ifdef V1
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

#ifdef V2

typedef enum
{

	PID_Position,
	PID_Speed
	
}PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;      //位置环或速度环
	
	float target;							
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					//反馈值
	float   err;							
	float   last_err;      		
	
	float pout;
	float iout;
	float dout;
	
	float output;						
	float last_output;			

	float MaxOutput;				
	float IntegralLimit;		
	float DeadBand;			  
	float ControlPeriod;		
	float  Max_Err;					
	
					  uint32_t thistime;
					uint32_t lasttime;
						uint8_t dtime;	
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  
				   PID_ID id,
				   uint16_t maxOutput,
				   uint16_t integralLimit,
				   float deadband,
				   uint16_t controlPeriod,
					int16_t max_err,     
					int16_t  target,
				   float kp,
				   float ki,
				   float kd);
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   
}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
#endif

#endif
