#include "mpu.h"

#ifdef	_USE_ATK_MPU6050
uint8_t MPU_Init(void)
{ 
	uint8_t res = 0; 
	uint8_t state = 0;
	state = MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data = 0;
	if(lpf>=188) data=1;
	else if(lpf>=98) data=2;
	else if(lpf>=42) data=3;
	else if(lpf>=20) data=4;
	else if(lpf>=10) data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000) rate = 1000;
	if(rate<4) rate = 4;
	data = 1000/rate-1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res = MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res == HAL_OK)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res = MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res == HAL_OK)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	}
    return res;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
HAL_StatusTypeDef MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	/*
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c1,(addr<<1)|0,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100);
	if (status != HAL_OK)  
	{                   //I2C故障处理
      HAL_I2C_DeInit(&hi2c1);        //释放IO口为GPIO，复位句柄状态标志
      HAL_I2C_Init(&hi2c1);          //这句重新初始化I2C控制器
    }
	*/
	HAL_StatusTypeDef status = HAL_OK;
	i2c_Start();
	i2c_SendByte((addr<<1)|0);	//发送器件地址+写命令
	i2c_Ack();
	i2c_SendByte(reg);
	i2c_WaitAck();
	for(int i=0;i<len;i++)
	{
		i2c_SendByte(buf[i]);	//发送数据
		i2c_WaitAck();	//等待ACK
	}    
    i2c_Stop();
	return status;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
HAL_StatusTypeDef MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
	/*
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1,(addr<<1)|1,reg,I2C_MEMADD_SIZE_8BIT,buf,len,100);
    if (status != HAL_OK)  
	{                   //I2C故障处理
      HAL_I2C_DeInit(&hi2c1);        //释放IO口为GPIO，复位句柄状态标志
      HAL_I2C_Init(&hi2c1);          //这句重新初始化I2C控制器
    }
	*/
	HAL_StatusTypeDef status = HAL_OK;
	
 	i2c_Start(); 
	i2c_SendByte((addr<<1)|0);//发送器件地址+写命令	
	i2c_WaitAck();	//等待应答
    i2c_SendByte(reg);	//写寄存器地址
    i2c_WaitAck();		//等待应答
	
    i2c_Start();
	i2c_SendByte((addr<<1)|1);//发送器件地址+读命令	
    i2c_WaitAck();		//等待应答 
	while(len)
	{
		if(len==1) 
		{
			*buf = i2c_ReadByte();//读数据,发送nACK 
			i2c_NAck();
		}
		else
		{
			*buf = i2c_ReadByte();		//读数据,发送ACK  
			i2c_Ack();
		}
		len--;
		buf++; 
	}    
    i2c_Stop();	//产生一个停止条件 
	return status;
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
HAL_StatusTypeDef MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
	/*
	HAL_StatusTypeDef status = HAL_OK;
	//发送器件地址+写命令	
	status = HAL_I2C_Mem_Write(&hi2c1,((MPU_ADDR<<1)|0),reg,I2C_MEMADD_SIZE_8BIT,&data,sizeof(uint8_t),100);
	*/
	HAL_StatusTypeDef status = HAL_OK;
    i2c_Start(); 
	i2c_SendByte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	i2c_Stop();		//等待应答 
    i2c_SendByte(reg);	//写寄存器地址
    i2c_WaitAck();		//等待应答 
	i2c_SendByte(data);//发送数据
	i2c_WaitAck();	//等待ACK
    i2c_Stop();	
	return status;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
	/*
	//发送器件地址+写命令	
	uint8_t data = 0;
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1,((MPU_ADDR<<1)|1),reg,I2C_MEMADD_SIZE_8BIT,&data,sizeof(uint8_t),100);
	if (status != HAL_OK)  
	{                   //I2C故障处理
      HAL_I2C_DeInit(&hi2c1);        //释放IO口为GPIO，复位句柄状态标志
      HAL_I2C_Init(&hi2c1);          //这句重新初始化I2C控制器
    }
	*/
	uint8_t data;
    i2c_Start(); 
	i2c_SendByte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	i2c_WaitAck();		//等待应答 
    i2c_SendByte(reg);	//写寄存器地址
    i2c_WaitAck();		//等待应答
	
    i2c_Start();
	i2c_SendByte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    i2c_WaitAck();		//等待应答 
	data = i2c_ReadByte();//读取数据,发送nACK,通知从设备停止发送
	i2c_NAck();
    i2c_Stop();			//产生一个停止条件 
	return data;
}
#endif

#ifdef	_USE_MPU6050
void MPU_Init(enum SPEED speed)
{
	MPU_SetSpeed(speed);
	MPU_SendByte(MPU_RST);
}
void MPU_SetSpeed(enum SPEED speed)
{
	switch(speed)
	{
		case FAST:
			MPU_SendByte(MPU_FAST);
		break;
		case SLOW:
			MPU_SendByte(MPU_SLOW);
		break;
		default:
			break;
	}
}
void MPU_SendByte(uint8_t data)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_UART_Transmit(&huart4,&data,1,10);
	if(status != HAL_OK)
	{
		printf("Error: Send Byte to MPU Error\n");
		printf("Error: Error Code %d\n",status);
	}
}

#endif

