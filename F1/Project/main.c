#include "motor.h"
#include "move.h"
#include "delay.h"

int main()
{
	delay_init();
	MotorInit();
	while(1)
	{
		MotorSelfCheck();
	}
}
