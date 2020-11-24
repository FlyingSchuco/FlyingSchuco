#include "myMath.h"

int CmpFloat(const void*a,const void*b)
{
	return *(float*)a-*(float*)b;
}

float DegRange180(float angle)
{
	if(angle >= 180.0f)
	{
	  angle = angle - 360.0f;
	}
	else if(angle < -180.0f)
	{
	  angle = angle + 360.0f;
	}
	else 
	{
	  angle = angle;
	}
	return angle;
}

int sign(float x)
{
	if(x > 0.0f)
	{
		return 1;
	}
	else if(x < 0.0f)
	{
		return -1;
	}
	else 
	{
		return 0;
	}
}


