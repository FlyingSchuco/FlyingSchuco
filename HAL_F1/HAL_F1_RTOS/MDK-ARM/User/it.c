#include "it.h"
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriotityTaskWoken = pdFALSE;

	switch(GPIO_Pin)
	{

	}
	portYIELD_FROM_ISR(xHigherPriotityTaskWoken);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
}
