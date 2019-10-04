#ifndef __MOVE_H
#define __MOVE_H
#include "motor.h"
void MoveFront(u16 ms, u8 pwm);
void MoveBack(u16 ms, u8 pwm);
void MoveLeft(u16 ms, u8 pwm);
void MoveRight(u16 ms, u8 pwm);
#endif
