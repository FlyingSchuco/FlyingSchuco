#ifndef _MOTOR_H
#define _MOTOR_H
#include "sys.h"
#include "delay.h"
#include "pwm.h"
//MOTOR0
#define EN0 PBout(6)
#define MOTOR0in1 PBout(1)
#define MOTOR0in2 PBout(2)
//MOTOR1
#define EN1 PBout(7)
#define MOTOR1in1 PBout(3)
#define MOTOR1in2 PBout(4)
//MOTOR2
#define EN2 PBout(8)
#define MOTOR2in1 PBout(10)
#define MOTOR2in2 PBout(11)
//MOTOR3
#define EN3 PBout(9)
#define MOTOR3in1 PBout(12)
#define MOTOR3in2 PBout(13)

void Motor0Init(void);
void Motor1Init(void);
void Motor2Init(void);
void Motor3Init(void);
void MotorInit(void);
void Motor0Run(u8 pwm);
void Motor0RunRV(u8 pwm);
void Motor1Run(u8 pwm);
void Motor1RunRV(u8 pwm);
void Motor2Run(u8 pwm);
void Motor2RunRV(u8 pwm);
void Motor3Run(u8 pwm);
void Motor3RunRV(u8 pwm);
void MotorSelfCheck(void);
void AllBrake(void);
#endif
