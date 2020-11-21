#include "motion.h"
MotionState *MotionStateInit(void)
{
	MotionState *Motion = (MotionState *)pvPortMalloc(sizeof(MotionState));
	Motion->aacx = 0;
	Motion->aacy = 0;
	Motion->aacz = 0;
	Motion->gyrox = 0;
	Motion->gyroy = 0;
	Motion->gyroz = 0;
	Motion->pitch = 0.0f;
	Motion->roll = 0.0f;
	Motion->yaw = 0.0f;
	Motion->temp = 0; 
	Motion->x = 0;
	Motion->y = 0;
	Motion->dF = 0.0f;
	Motion->dB = 0.0f;
	Motion->dL = 0.0f;
	Motion->dR = 0.0f;
	return Motion;
}

