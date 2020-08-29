#include "opticalFlow.h"
//片选信号
void NCS(NCS_State state)
{
	if(state == ON)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
	}
	else if(state == OFF)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
	}
}

//写寄存器
void WriteReg(uint8_t addr, uint8_t data,NCS_MODE state)
{
	//写操作MSB首位为1，LSB首位为0
	if(hspi1.Init.FirstBit == SPI_FIRSTBIT_MSB)
	{
		addr |= 1<<7;
	}
	if(state == AUTO)
	{
		NCS(ON);
	}
	HAL_SPI_Transmit(&hspi1,&addr,1,0xFFFF);
	HAL_SPI_Transmit(&hspi1,&data,1,0xFFFF);
	if(state == AUTO)
	{
		NCS(OFF);
	}
	gen_delay_us(TSW);
}

//读寄存器
uint8_t ReadReg(uint8_t addr)
{
	uint8_t TxNull = 0x00;
	uint8_t RxData;
	//读操作MSB首位为0，LSB首位为1
	if(hspi1.Init.FirstBit == SPI_FIRSTBIT_LSB)
	{
		addr |= 1<<7;
	}
	NCS(ON);
	HAL_SPI_Transmit(&hspi1,&addr,1,0xFFFF);
	gen_delay_us(TSRAD);
	HAL_SPI_TransmitReceive(&hspi1,&TxNull,&RxData,1,0xFFFF);
	NCS(OFF);
	gen_delay_us(TSR);
	return RxData;
}

void SetFP(void)
{
	//锁定帧率
	WriteReg(0x0b,0x01,AUTO);

    uint8_t lower = 0xC0;//(uint8_t)(24000000/FrameRate);
    uint8_t upper = 0x12;//(uint8_t)((24000000/FrameRate)>>8);
	if(ReadReg(0x0b)>>7==0)
	{
		WriteReg(Frame_Period_Max_Bound_Lower,lower,AUTO);
		WriteReg(Frame_Period_Max_Bound_Upper,upper,AUTO);
		HAL_Delay(2);
		printf("Info: Frame Period is set sucessfully\n");
	}
	else
	{
		printf("Error: Set Frame Period Error\n");
	}
}

void OPTF_Reset(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
}

void SromReload(void)
{
	OPTF_Reset();
	NCS(ON);
	WriteReg(0x20,0x44,MAN);
	WriteReg(0x23,0x07,MAN);
	WriteReg(0x24,0x88,MAN);
	NCS(OFF);
	
	gen_delay_us(FramePeriod);

	WriteReg(SROM_Enable,0x18,AUTO);
	gen_delay_us(40);
	
	NCS(ON);
	//写操作
	uint8_t addr = SROM_Load+0x80;
	HAL_SPI_Transmit(&hspi1,&addr,1,0xFFFF);
	gen_delay_us(10);
	for(int i=0;i<1986;++i)
	{
		HAL_SPI_Transmit(&hspi1,(uint8_t *)(&Srom[i]),1,0XFFFF);
		gen_delay_us(10);
	}
	
	NCS(OFF);
	gen_delay_us(TBEXIT);
	gen_delay_us(100);
}

void OpticalFlowInit(void)
{
	SromReload();
	printf("Info: Srom is Reloaded\n");
	HAL_Delay(3);
	SetFP();
	HAL_Delay(3);
	WriteReg(Configuration_bits,0x10,AUTO);
	HAL_Delay(3);
	printf("Info: Set Resolution = 1600\n");
	printf("OPticalFlowInit Over\n");
}

//Burst模式读移动数据
void BurstRead(BurstData *data)
{
	uint8_t addr = Motion_Burst;
	uint8_t TxNull = 0x00;
	uint8_t *tmp = (uint8_t *)data;
	//读操作MSB首位为0，LSB首位为1
	if(hspi1.Init.FirstBit == SPI_FIRSTBIT_LSB)
	{
		addr |= 1<<7;
	}
	NCS(ON);
	HAL_SPI_Transmit(&hspi1,&addr,1,0xFFFF);
	gen_delay_us(TSRAD_MOT);
	for(int i=0;i<7;++i)
	{
		HAL_SPI_TransmitReceive(&hspi1,&TxNull,tmp,1,0xFFFF);
		++tmp;
	}
	NCS(OFF);
	gen_delay_us(TBEXIT);
}

//Busrt模式获取当前帧
void GetOneFrame(uint8_t buffer[])
{
	WriteReg(Frame_Capture,0x83,AUTO);
	
	gen_delay_us(10+3*200);
	uint8_t addr = Pixel_Burst;
	uint8_t TxData = 0x00;
	//读操作MSB首位为0，LSB首位为1
	if(hspi1.Init.FirstBit == SPI_FIRSTBIT_LSB)
	{
		addr |= 1<<7;
	}
	
	NCS(ON);
	HAL_SPI_Transmit(&hspi1,&addr,1,0xFFFF);
		
	gen_delay_us(TSRAD_MOT);
	for(int i = 0;i<900;++i)
	{
		HAL_SPI_TransmitReceive(&hspi1,&TxData,(&buffer[i]),1,0xFFFF);
		if(i==0 && (buffer[i])>>6!=0x03)
		{
			i--;
		}
		else
		{
			buffer[i] &= 0x3F;
		}
		gen_delay_us(10);
	}
	NCS(OFF);
	
	gen_delay_us(TBEXIT);
}
