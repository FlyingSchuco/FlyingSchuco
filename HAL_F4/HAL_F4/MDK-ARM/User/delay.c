#include "delay.h"
void gen_delay_us(uint16_t us,TIM_HandleTypeDef *htimx)
{
    uint16_t differ=0xffff-us-5;
    HAL_TIM_Base_Start(htimx);
    __HAL_TIM_SetCounter(htimx,differ);
    while(differ<0xffff-5)
    {
        differ=__HAL_TIM_GetCounter(htimx);
    }
    HAL_TIM_Base_Stop(htimx);
}
