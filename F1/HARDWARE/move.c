#include "move.h"
#include "delay.h"
void MoveFront(u16 ms)
{
	Motor0RunRV();
	Motor1RunRV();
	Motor2Run();
	Motor3Run();
	delay_ms(1000);
}
