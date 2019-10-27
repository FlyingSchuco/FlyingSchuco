#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx_hal.h"
void gen_delay_us(uint16_t us,TIM_HandleTypeDef *htimx);
#endif
