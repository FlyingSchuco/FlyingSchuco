#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx_hal.h"
extern TIM_HandleTypeDef htim14;
void gen_delay_us(uint16_t us);
#endif
