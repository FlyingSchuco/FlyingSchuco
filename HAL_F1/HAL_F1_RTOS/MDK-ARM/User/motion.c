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
	Motion->pitch = 0;
	Motion->roll = 0;
	Motion->yaw = 0;
	Motion->temp = 0; 
	return Motion;
}

