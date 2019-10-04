#ifndef __PWM_H
#define __PWM_H
#include "sys.h"
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM4_PwmOut(u8 channel,u8 percent);
#endif
