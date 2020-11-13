//i2c.h
#ifndef __I2C_H
#define __I2C_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define GPIO_PORT_I2C	GPIOB			     // GPIO端口 
#define I2C_SCL_PIN		IMU_SCL_Pin			 // 连接到SCL时钟线的GPIO 
#define I2C_SDA_PIN		IMU_SDA_Pin			 // 连接到SDA数据线的GPIO 
 
 
/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN						// SCL = 1 
#define I2C_SCL_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SCL_PIN << 16U  	// SCL = 0 
 
#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN   					// SDA = 1 
#define I2C_SDA_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SDA_PIN << 16U		// SDA = 0 
 
#define I2C_SDA_READ()  (GPIO_PORT_I2C->IDR & I2C_SDA_PIN)		// 读SDA口线状态 
#define I2C_SCL_READ()  (GPIO_PORT_I2C->IDR & I2C_SCL_PIN)		// 读SCL口线状态 
 
 
void i2c_Start(void);
//停止信号
void i2c_Stop(void);
//发送一个字节
void i2c_SendByte(uint8_t _ucByte);
//读取一个字节
uint8_t i2c_ReadByte(void);
//等待应答信号
uint8_t i2c_WaitAck(void);
//应答信号
void i2c_Ack(void);
//非应答信号
void i2c_NAck(void);
//检查设备
uint8_t i2c_CheckDevice(uint8_t _Address);
//发送数据
void  Device_WriteData(uint8_t DeciveAddr,uint8_t *Data,int size);

#endif
