#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim3;
void gen_delay_us(uint16_t us);
#endif
