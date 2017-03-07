/***************************include����*********************************************/
#include "BSP_init.h"


/***************************�궨��**************************************************/
/***************************extern�����ⲿ������************************************/
/***************************Struct����ṹ��****************************************/
/***************************��ͨ��������********************************************/


/******************************************************************************
����ԭ��:	void BSP_init(void)
��������:	��ʼ������Ӳ��
˵		����
*******************************************************************************/
void BSP_init(void)
{
	LED_init();
	Nvic_Init();
  printf_init();
	Timer3_Init();
	
	I2C2_Int();
	InitMPU6050();
	MPU6050_Offset();
	
//	MS5611_Init();
	
	SPI1_Init();
  NRF24L01_Check(); //���2401�Ƿ�����
	
	pid_init();
	Motor_Init();
	
  MS5611_Init();
//	printf("BSP_init success\n");
}