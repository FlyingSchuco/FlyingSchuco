#include "move.h"
#include "motor.h"
#include "delay.h"
void MoveFront(u16 ms, u8 pwm)
{
	Motor0RunRV(pwm);
	Motor1Run(pwm);
	Motor2Run(pwm);
	Motor3RunRV(pwm);
	delay_ms(ms);
	AllBrake();
}

void MoveBack(u16 ms, u8 pwm)
{
	Motor0Run(pwm);
	Motor1RunRV(pwm);
	Motor2RunRV(pwm);
	Motor3Run(pwm);
	delay_ms(ms);
	AllBrake();
}

void MoveLeft(u16 ms, u8 pwm)
{
	Motor0Run(pwm);
	Motor1Run(pwm);
	Motor2RunRV(pwm);
	Motor3RunRV(pwm);
	delay_ms(ms);
	AllBrake();
}

void MoveRight(u16 ms, u8 pwm)
{
	Motor0RunRV(pwm);
	Motor1RunRV(pwm);
	Motor2Run(pwm);
	Motor3Run(pwm);
	delay_ms(ms);
	AllBrake();
}
