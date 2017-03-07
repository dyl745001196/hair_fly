#ifndef _I2C_MPU6050_H_
#define _I2C_MPU6050_H_
#include "stm32f10x.h"


/******************************************************************************
							宏定义
*******************************************************************************/ 
#define Debug1_Pin	GPIO_Pin_5	//用于测量程序运行速率
#define	Debug1_H   	GPIOB->BSRR = Debug1_Pin //高电平
#define	Debug1_L   	GPIOB->BRR  = Debug1_Pin //低电平

#define Debug2_Pin	GPIO_Pin_9		
#define	Debug2_H   	GPIOB->BSRR = Debug2_Pin 
#define	Debug2_L   	GPIOB->BRR  = Debug2_Pin 

#define Debug3_Pin	GPIO_Pin_8		
#define	Debug3_H   	GPIOB->BSRR = Debug3_Pin 
#define	Debug3_L   	GPIOB->BRR  = Debug3_Pin 

/******************************************************************************
							全局变量声明
*******************************************************************************/ 	
extern uint8_t	GYRO_Offset;	
extern uint8_t	ACC_Offset;
extern uint32_t I2C_Erro;
extern uint8_t	MPU6050_Buffer[14];
				
/******************************************************************************
							全局函数声明
*******************************************************************************/ 
uint8_t InitMPU6050(void);
uint8_t MPU6050_SequenceRead(void);
void I2C2_Int(void);
void MPU6050_SingleRead(void);
void MPU6050_Compose(void);
void MPU6050_Printf(void);
void Do_ACC_Offset(void);
void Do_GYRO_Offset(void);	
void MPU6050_Offset(void);

uint8_t I2C_Start(void);
void I2C_Stop(void);
void I2C_ACK(void);
void I2C_NACK(void);
uint8_t I2C_WaitAck(void);
void I2C_WriteByte(unsigned char SendByte);
uint8_t I2C_ReadByte(void);
uint8_t Single_WriteI2C(unsigned char Regs_Addr,unsigned char Regs_Data);
uint8_t Single_ReadI2C(unsigned char Regs_Addr);
/******************************************************************************
							定义MPU6050寄存器地址
*******************************************************************************/ 
#define	SMPLRT_DIV		0x19	
#define	CONFIG2			0x1A	
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C	
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器的地址(默认数值0x68，只读!!!)

//定义器件在IIC总线中的从地址
//I2C器件地址是7bit而不是8bit，而模拟I2C用8bit传送地址，因此是0xD0（器件AD0引脚接地）
#define	MPU6050Address	0xD0

#endif

