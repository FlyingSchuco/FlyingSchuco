#include "sys.h"
#define PA 'A'
#define PB 'B'
#define PC 'C'
#define PD 'D'
#define PE 'E'
#define PF 'F'
#define PG 'G'

GPIO_InitTypeDef  GPIO_InitStructure;
void out(unsigned char i)
{
	switch(i){
		case 0:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;break;
		case 1:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;break;
		case 2:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;break;
		case 3:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;break;
		case 4:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;break;
		case 5:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;break;
		case 6:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;break;
		case 7:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;break;
		case 8:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;break;
		case 9:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;break;
		case 10:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;break;
		case 11:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;break;
		case 12:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;break;
		case 13:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;break;
		case 14:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;break;
		case 15:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;break;
	}
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
}

void OUT(char c,unsigned char i)
{
	out(i);
	switch(c){
		case 'A':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);GPIO_Init(GPIOA, &GPIO_InitStructure);break;
		case 'B':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);GPIO_Init(GPIOB, &GPIO_InitStructure);break;
		case 'C':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);GPIO_Init(GPIOC, &GPIO_InitStructure);break;
		case 'D':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);GPIO_Init(GPIOD, &GPIO_InitStructure);break;
		case 'E':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);GPIO_Init(GPIOE, &GPIO_InitStructure);break;
		case 'F':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);GPIO_Init(GPIOF, &GPIO_InitStructure);break;
		case 'G':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);GPIO_Init(GPIOG, &GPIO_InitStructure);break;
	}
}

void in(unsigned char i)
{
	switch(i){
		case 0:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;break;
		case 1:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;break;
		case 2:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;break;
		case 3:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;break;
		case 4:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;break;
		case 5:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;break;
		case 6:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;break;
		case 7:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;break;
		case 8:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;break;
		case 9:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;break;
		case 10:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;break;
		case 11:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;break;
		case 12:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;break;
		case 13:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;break;
		case 14:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;break;
		case 15:GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;break;
	}
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
}

void IN(char c,unsigned char i)
{
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	in(i);
	switch(c){
		case 'A':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);GPIO_Init(GPIOA, &GPIO_InitStructure);break;
		case 'B':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);GPIO_Init(GPIOB, &GPIO_InitStructure);break;
		case 'C':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);GPIO_Init(GPIOC, &GPIO_InitStructure);break;
		case 'D':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);GPIO_Init(GPIOD, &GPIO_InitStructure);break;
		case 'E':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);GPIO_Init(GPIOE, &GPIO_InitStructure);break;
		case 'F':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);GPIO_Init(GPIOF, &GPIO_InitStructure);break;
		case 'G':	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);GPIO_Init(GPIOG, &GPIO_InitStructure);break;
	}
}

void delayms(int i)
{
	int j;
	for(;i>0;i--) for(j=0;j<9000;j++);
}

void delayus(int i)
{
	int j;
	for(;i>0;i--) for(j=0;j<10;j++);
}

unsigned char key()
{
	if(PAin(0)==0)
	{
		delayms(50);
		if(PAin(0)==0)
		{
			while((PAin(0)==0));
			return 1;
		}
	}
	return 0;
}
