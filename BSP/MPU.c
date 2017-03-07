/***************************include����*********************************************/
#include "stdint.h"
#include "MPU.h"
#include "Printf.h"
#include "systick.h"
#include "struct_all.h"
#include "math.h"


/***************************�궨��**************************************************/
#define SCL_Pin 	GPIO_Pin_6
#define SDA_Pin 	GPIO_Pin_7

#define	SCL_H   	{GPIOB->BSRR = SCL_Pin;I2C_delay(4);}	 //SCL�ߵ�ƽ
#define	SCL_L   	{GPIOB->BRR  = SCL_Pin;I2C_delay(4);}	 //SCL�͵�ƽ
#define	SDA_H   	{GPIOB->BSRR = SDA_Pin;I2C_delay(4);}	 //SDA�ߵ�ƽ
#define	SDA_L   	{GPIOB->BRR  = SDA_Pin;I2C_delay(4);}	 //SDA�͵�ƽ
#define	SDA_Read	GPIOB->IDR  & SDA_Pin	 //SDA������

#define q30  1073741824.0f

/***************************extern�����ⲿ������************************************/


/***************************Struct����ṹ��****************************************/



/***************************��ͨ��������********************************************/
uint8_t	GYRO_Offset=1 ;//1ΪУ��
uint8_t	ACC_Offset=1 ; //1ΪУ��
uint32_t I2C_Erro=0;
float testx;
float testy;
float testz;
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float Pitch;
uint8_t	MPU6050_Buffer[14];	//I2C��ȡ���ݻ���
//Ӳ������˵��
//PB6 for the SCL
//PB7 for the SDA

/******************************************************************************
����ԭ��:	void I2C2_Int(void)
��������:	��ʼ��I2C����
˵		����
*******************************************************************************/ 
void I2C2_Int(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //������B��ʱ��

	GPIO_InitStructure.GPIO_Pin = Debug1_Pin | Debug2_Pin | Debug3_Pin; //���ڲ���������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
	GPIO_Init(GPIOB, &GPIO_InitStructure);		

	GPIO_InitStructure.GPIO_Pin = SCL_Pin; //SCL
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //��©���
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = SDA_Pin; //SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; //��©���
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
//	printf("software i2c initialized succussfully ! \n");
}


/******************************************************************************
����ԭ��:	static void I2C_delay(int delay)
��������:	I2C������ʱ
˵		����
*******************************************************************************/ 
static void I2C_delay(int delay)
{
  while (delay)
    delay--;
}

/******************************************************************************
����ԭ�ͣ�	static uint8_t I2C_Start(void)	
��    �ܣ�	I2C���߿�ʼ�ź�
˵		����  ����ֵ��1�ɹ���0ʧ��
*******************************************************************************/ 
uint8_t I2C_Start(void)	
{
	SCL_L;					
	SDA_H;
	SCL_H;
	
	if(!SDA_Read)
		return 0;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L;
	if(SDA_Read) 
		return 0;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	
	SDA_L;
	return 1;
}

/******************************************************************************
����ԭ�ͣ�	static void I2C_Stop(void)	
��    �ܣ�	I2C����ֹͣ�ź�
˵		����
*******************************************************************************/ 
void I2C_Stop(void)
{
	SCL_L;					
	SDA_L;
	SCL_H;
	SDA_H;
}

/******************************************************************************
����ԭ�ͣ�	static void I2C_ACK(void)	
��    �ܣ�	I2C����Master����ACK�ź�
˵		����	
*******************************************************************************/ 
void I2C_ACK(void)
{
	SCL_L; 	
	__nop();
	SDA_L;	//дӦ���ź�
	__nop();
    SCL_H; 
	__nop();
    SCL_L;      
}

/******************************************************************************
����ԭ�ͣ�	static void I2C_NACK(void)	
��    �ܣ�	I2C����Master����NACK�ź�
˵		����
*******************************************************************************/ 
void I2C_NACK(void)
{
	SCL_L; 		
//	__nop();
	SDA_H;	//��дӦ���ź�
//	__nop();
    SCL_H; 
//	__nop();
    SCL_L;  
}

/******************************************************************************
����ԭ�ͣ�	static uint8_t I2C_WaitAck(void)	
��    �ܣ�	I2C����Master����SlaveӦ��
˵		����	1���յ�ACK��0�յ�NACK  
*******************************************************************************/ 
uint8_t I2C_WaitAck(void)
{
	SCL_L;				
	SDA_H;
	SCL_H;
		
	if(SDA_Read)
	{
		SCL_L;
		return 0;	
	}
	
	SCL_L;
	return 1;	  
}
/******************************************************************************
����ԭ�ͣ�	static void I2C_WriteByte(unsigned char SendByte) 	
��    �ܣ�	I2Cдһ���ֽ�����,��д��λ
˵		��:
*******************************************************************************/ 
void I2C_WriteByte(unsigned char SendByte) 
{
	uint8_t i=8; 
	
	while(i--)
	{
		SCL_L;
		if( SendByte&0x80 ) 
			SDA_H
		else 
			SDA_L
		SendByte <<= 1;
		SCL_H;
	}   
	SCL_L;  
}

/******************************************************************************
����ԭ�ͣ�	static uint8_t I2C_ReadByte(void)	
��    �ܣ�	I2C��һ���ֽ�����,�ȶ���λ
˵		����	���ض�����һ���ֽ�����
*******************************************************************************/ 
uint8_t I2C_ReadByte(void)
{
	uint8_t i=8;
	uint8_t ReceiveByte =0;
	
	SDA_H;
	while(i--)
	{
		SCL_L;  
		__nop();
		SCL_H; 		
		ReceiveByte <<= 1;
		if(SDA_Read) 
			ReceiveByte |= 0x01;
	}
	SCL_L;
	return ReceiveByte;
}

/******************************************************************************
����ԭ�ͣ�	static uint8_t Single_WriteI2C(unsigned char Regs_Addr,unsigned char Regs_Data) 	
��    �ܣ�	дMPU6050�Ĵ���
˵		����	0����1�ɹ�
*******************************************************************************/ 
uint8_t Single_WriteI2C(unsigned char Regs_Addr,unsigned char Regs_Data) 
{  
	if( !I2C_Start() )
		return 0;	//I2C��ʼ���󣬷���
	I2C_WriteByte(MPU6050Address); //дSlave��ַ�������ó�дģʽ
	if( !I2C_WaitAck() )
	{
		I2C_Stop(); 
		return 0;	//��ACK������
	}
	I2C_WriteByte(Regs_Addr);    //д�Ĵ�����ַ
	I2C_WaitAck();
	I2C_WriteByte(Regs_Data);    //д�Ĵ�������
	I2C_WaitAck();
	I2C_Stop();         
	return 1;
}
/******************************************************************************
����ԭ�ͣ�	static uint8_t Single_ReadI2C(unsigned char Regs_Addr) 	
��    �ܣ�	��MPU6050���ֽ�
˵		����	�Ĵ���ֵ
*******************************************************************************/ 
uint8_t Single_ReadI2C(unsigned char Regs_Addr) 
{
	uint8_t ret;

	if( !I2C_Start() )
		return 0;	//I2C��ʼ���󣬷���
	I2C_WriteByte(MPU6050Address); 	//дSlave��ַ�������ó�дģʽ
	if( !I2C_WaitAck() )
	{
		I2C_Stop(); 
		return 0;	//��ACK������
	}
	
	I2C_WriteByte(Regs_Addr);    	//д�Ĵ�����ַ
	I2C_WaitAck();
	I2C_Start();
	I2C_WriteByte(MPU6050Address+1);//дSlave��ַ�������óɶ�ģʽ
	I2C_WaitAck();
	
	ret=I2C_ReadByte();				//�Ӵ������ж�������

	I2C_NACK();						//��Ӧ��
	I2C_Stop();                  	//��������IIC���� 
	return ret;	
} 

/******************************************************************************
����ԭ�ͣ�	uint8_t MPU6050_SequenceRead(void) 	
��    �ܣ�	������MPU6050���ݼĴ���
˵		����	0����1�ɹ�
*******************************************************************************/ 
uint8_t MPU6050_SequenceRead(void)
{
	uint8_t index=14;
	
	if( !I2C_Start() )
		return 0;	//I2C��ʼ���󣬷���
	I2C_WriteByte(MPU6050Address); 	//дSlave��ַ�������ó�дģʽ
	if( !I2C_WaitAck() )
	{
		I2C_Stop(); 
		return 0;	//��ACK������
	}
	I2C_WriteByte(ACCEL_XOUT_H);    //д�Ĵ�����ַ
	I2C_WaitAck();
	
	I2C_Start();
	I2C_WriteByte(MPU6050Address+1);//дSlave��ַ�������óɶ�ģʽ
	I2C_WaitAck();

	while(--index)	//����13λ�Ĵ���
	{
		MPU6050_Buffer[13 - index] = I2C_ReadByte();
		I2C_ACK();
	}

	MPU6050_Buffer[13] = I2C_ReadByte();	//����14�Ĵ���
	I2C_NACK();	
	I2C_Stop();	
	
	return 1;
}
/******************************************************************************
����ԭ�ͣ�	void MPU6050_SingleRead(void)
��    �ܣ�	�ִζ�MPU6050���ݼĴ���
˵		����
*******************************************************************************/ 
void MPU6050_SingleRead(void)
{
  MPU6050_Buffer[0] = Single_ReadI2C(ACCEL_XOUT_H);
	MPU6050_Buffer[1] = Single_ReadI2C(ACCEL_XOUT_L);		
	MPU6050_Buffer[2] = Single_ReadI2C(ACCEL_YOUT_H);	
	MPU6050_Buffer[3] = Single_ReadI2C(ACCEL_YOUT_L);
	MPU6050_Buffer[4] = Single_ReadI2C(ACCEL_ZOUT_H);
	MPU6050_Buffer[5] = Single_ReadI2C(ACCEL_ZOUT_L);
	
	MPU6050_Buffer[8]  = Single_ReadI2C(GYRO_XOUT_H);
	MPU6050_Buffer[9]  = Single_ReadI2C(GYRO_XOUT_L);
	MPU6050_Buffer[10] = Single_ReadI2C(GYRO_YOUT_H);
	MPU6050_Buffer[11] = Single_ReadI2C(GYRO_YOUT_L);
	MPU6050_Buffer[12] = Single_ReadI2C(GYRO_ZOUT_H);
	MPU6050_Buffer[13] = Single_ReadI2C(GYRO_ZOUT_L);
}
/******************************************************************************
����ԭ�ͣ�	void MPU6050_Offset(void)
��    �ܣ�	MPU6050��ƫУ��
˵		����  MPU_composeÿ�ε������������ǰ250�ε��õľ�ֵ��ΪУ�������˺�ÿһ�ε��ö����ȥ��ֵ
*******************************************************************************/ 
void MPU6050_Offset(void)
{
	if(ACC_Offset)
	{
		static int32_t ACC_X=0,ACC_Y=0,ACC_Z=0;
		static uint8_t count_acc=0;
		if(count_acc==0)
		{
			offset_acc.x = 0;
			offset_acc.y = 0;
			offset_acc.z = 0;
			ACC_X = 0;
			ACC_Y = 0;
			ACC_Z = 0;
			count_acc = 1;
			return;
		}
		else
		{
			count_acc++;
			ACC_X += acc.x;
			ACC_Y += acc.y;
			ACC_Z += acc.z;
		}
		if(count_acc==251)
		{
			count_acc--;
			offset_acc.x = ACC_X / count_acc;
			offset_acc.y = ACC_Y / count_acc;
			offset_acc.z = ACC_Z / count_acc - 2048;
			count_acc = 0;
			ACC_Offset = 0;       //�رոó���
		}
	}
	if(GYRO_Offset)
	{
		static int32_t GYRO_X=0,GYRO_Y=0,GYRO_Z=0;
		static uint8_t count_gyro=0;
		if(count_gyro==0)
		{
			offset_gyro.x = 0;
			offset_gyro.y  = 0;
			offset_gyro.z   = 0;
			GYRO_X = 0;
			GYRO_Y = 0;
			GYRO_Z = 0;
			count_gyro = 1;
			return;
		}
		else
		{
			count_gyro++;
			GYRO_X += gyro.x;
			GYRO_Y += gyro.y;
			GYRO_Z += gyro.z;
		}
		if(count_gyro==251)
		{
			count_gyro--;
			offset_gyro.x = GYRO_X / count_gyro;
			offset_gyro.y = GYRO_Y / count_gyro;
			offset_gyro.z = GYRO_Z / count_gyro;
			count_gyro = 0;
			GYRO_Offset = 0;      //�رոó���
		}
	}
}

/******************************************************************************
����ԭ�ͣ�	void MPU6050_Compose(void)
��    �ܣ�	�ϳ�MPU6050��16λ����
˵		����
*******************************************************************************/ 
void MPU6050_Compose(void)
{
	acc.x  = ((((int16_t)MPU6050_Buffer[0]) << 8) | MPU6050_Buffer[1]) - offset_acc.x;	//��ȥ��ƫ
	acc.y  = ((((int16_t)MPU6050_Buffer[2]) << 8) | MPU6050_Buffer[3]) - offset_acc.y;
	acc.z  = ((((int16_t)MPU6050_Buffer[4]) << 8) | MPU6050_Buffer[5]) - offset_acc.z;

	gyro.x = ((((int16_t)MPU6050_Buffer[8])  << 8) | MPU6050_Buffer[9])  - offset_gyro.x;
	gyro.y = ((((int16_t)MPU6050_Buffer[10]) << 8) | MPU6050_Buffer[11]) - offset_gyro.y;
	gyro.z = ((((int16_t)MPU6050_Buffer[12]) << 8) | MPU6050_Buffer[13]) - offset_gyro.z;
	
	MPU6050_Offset();
}
/******************************************************************************
����ԭ�ͣ�	uint8_t MPU6050_SingleRead(void)
��    �ܣ�	��ʼ��MPU6050
�� �� ֵ��	0����1�ɹ�
*******************************************************************************/ 
uint8_t InitMPU6050(void)
{
	if( Single_ReadI2C(WHO_AM_I) != 0x68)//���MPU6050�Ƿ�����
	{
		return 0;
	}
	
//	printf("MPU6050 is normal\n");
	Single_WriteI2C(PWR_MGMT_1, 0x00);	//��Դ��������ֵ��0x00������ģʽ
	I2C_delay(20000); //Լ2.5ms��ʱ
	Single_WriteI2C(SMPLRT_DIV, 0x00);	//�����ǲ����ʣ�����ֵ��0x00������Ƶ��8KHZ��
	I2C_delay(20000);            
	Single_WriteI2C(CONFIG2, 0x00);   	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x00��������MPU6050�Դ��˲�
	I2C_delay(20000);
	Single_WriteI2C(GYRO_CONFIG, 0x18);	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	I2C_delay(20000);
	Single_WriteI2C(ACCEL_CONFIG, 0x1F);//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x1F(���Լ죬16G)
	I2C_delay(20000);
	
	Single_WriteI2C(0x37,0x02);     //��MPU��·I2C���أ���MPU6505  0x37�Ĵ�����bit1
//	printf("MPU6050  initialized successful!\n");
	return 1;
}

