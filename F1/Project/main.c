#include "motor.h"
#include "move.h"
#include "delay.h"

int main()
{
	delay_init();
	TIM4_PWM_Init(99,719); //arr 99 psc 719 PWM频率为1KHZ
	MotorInit();
	while(1)
	{
		MoveFront(500,10);
		delay_ms(1000);
		MoveBack(500,10);
		delay_ms(1000);
		MoveLeft(500,10);
		delay_ms(1000);
		MoveRight(500,10);
		delay_ms(1000);
	}
}
