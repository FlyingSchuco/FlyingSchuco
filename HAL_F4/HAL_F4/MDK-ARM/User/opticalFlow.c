#include "opticalFlow.h"
uint8_t TxData[2] = {0x01,0x00};
OPFRxData RxData = {
					{(0x01<<7)|(0x02),0x00},
					{(0x01<<7)|(0x03),0x00},
					{(0x01<<7)|(0x04),0x00},
					{(0x01<<7)|(0x0a),0x00}
								};

uint8_t SPI_SendReceive(uint8_t data)
{
	while(HAL_SPI_GetState(&hspi3)!=HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi3,&data,1,100);
	while(HAL_SPI_GetState(&hspi3)!=HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi3,&data,1,100);
	return data;
}
								
uint8_t read_register(uint8_t address)
{
uint8_t temp;
ON_CS();
HAL_SPI_TransmitReceive(&hspi3,&address,&temp,1,100);	//读
gen_delay_us(75,&htim14);
address = 0xff;
HAL_SPI_TransmitReceive(&hspi3,&address,&temp,1,100);	//提供时钟信号_读
OFF_CS();
return temp;
}

void GetMotionData(SPI_HandleTypeDef *hspix,OPFMotionData *buffer)
{
	HAL_SPI_TransmitReceive(hspix,RxData.state,RxData.state,2,100);
	buffer->state = (int8_t)RxData.state[1];
	HAL_SPI_TransmitReceive(hspix,RxData.deltaX,RxData.deltaX,2,100);
	buffer->deltaX = (int8_t)RxData.deltaX[1];
	HAL_SPI_TransmitReceive(hspix,RxData.deltaY,RxData.deltaY,2,100);
	buffer->deltaY = (int8_t)RxData.deltaY[1];
	HAL_SPI_TransmitReceive(hspix,RxData.configuration,RxData.configuration,2,100);
}
	
