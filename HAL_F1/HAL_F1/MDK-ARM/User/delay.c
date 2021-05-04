#include "delay.h"
void gen_delay_us(uint16_t us)
{
    uint16_t differ=0xffff-us-5;
    HAL_TIM_Base_Start(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3,differ);
    while(differ<0xffff-5)
    {
        differ=__HAL_TIM_GET_COUNTER(&htim3);
    }
    HAL_TIM_Base_Stop(&htim3);
}
