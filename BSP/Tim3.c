/***************************include����*********************************************/
#include "Tim3.h"


/***************************�궨��**************************************************/
/***************************extern�����ⲿ������************************************/
/***************************Struct����ṹ��****************************************/
/***************************��ͨ��������********************************************/


/******************************************************************************
����ԭ��:	void Timer3_Init(void)
��������:	��ʼ��Tim3
˵		��: �����������
*******************************************************************************/ 
void Timer3_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 1000 ;//װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	//��Ƶϵ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //���ָ�ʱ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//����жϱ�־
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);//ʹ�ܶ�ʱ��3
}
