#include "motor.h"
#include "move.h"

int main()
{
	MotorInit();
	while(1)
	{
		MotorSelfCheck();
	}
}
