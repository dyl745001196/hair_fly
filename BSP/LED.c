/***************************include����*********************************************/
#include "stm32f10x.h"
#include "LED.h"
#include "systick.h"


/***************************�궨��**************************************************/
/***************************extern�����ⲿ������************************************/
/***************************Struct����ṹ��****************************************/
/***************************��ͨ��������********************************************/



/******************************************************************************
Ӳ�����ӣ�
PA11 --- LED1,LED2
PA8  --- LED3,LED4
PB1  --- LED5,LED6
PB3  --- LED7,LED8
*******************************************************************************/


/******************************************************************************
����ԭ��:	LED_init(void)
��������:	LED��ʼ��
˵		��: PA11 --- LED1,LED2
					PA8  --- LED3,LED4
					PB1  --- LED5,LED6
					PB3  --- LED7,LED8
*******************************************************************************/ 
void LED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	delay_ms(200);
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
	delay_ms(200);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_1);
	delay_ms(200);
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_3);
	delay_ms(200);
	GPIO_ResetBits(GPIOB,GPIO_Pin_3);
}
