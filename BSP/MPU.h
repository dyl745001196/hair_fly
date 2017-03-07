#ifndef _I2C_MPU6050_H_
#define _I2C_MPU6050_H_
#include "stm32f10x.h"


/******************************************************************************
							�궨��
*******************************************************************************/ 
#define Debug1_Pin	GPIO_Pin_5	//���ڲ���������������
#define	Debug1_H   	GPIOB->BSRR = Debug1_Pin //�ߵ�ƽ
#define	Debug1_L   	GPIOB->BRR  = Debug1_Pin //�͵�ƽ

#define Debug2_Pin	GPIO_Pin_9		
#define	Debug2_H   	GPIOB->BSRR = Debug2_Pin 
#define	Debug2_L   	GPIOB->BRR  = Debug2_Pin 

#define Debug3_Pin	GPIO_Pin_8		
#define	Debug3_H   	GPIOB->BSRR = Debug3_Pin 
#define	Debug3_L   	GPIOB->BRR  = Debug3_Pin 

/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 	
extern uint8_t	GYRO_Offset;	
extern uint8_t	ACC_Offset;
extern uint32_t I2C_Erro;
extern uint8_t	MPU6050_Buffer[14];
				
/******************************************************************************
							ȫ�ֺ�������
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
							����MPU6050�Ĵ�����ַ
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
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ����ĵ�ַ(Ĭ����ֵ0x68��ֻ��!!!)

//����������IIC�����еĴӵ�ַ
//I2C������ַ��7bit������8bit����ģ��I2C��8bit���͵�ַ�������0xD0������AD0���Žӵأ�
#define	MPU6050Address	0xD0

#endif

