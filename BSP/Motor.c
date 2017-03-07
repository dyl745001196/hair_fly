/***************************include����*********************************************/
#include "Motor.h"


/***************************�궨��**************************************************/
/***************************extern�����ⲿ������************************************/
/***************************Struct����ṹ��****************************************/
/***************************��ͨ��������********************************************/



/******************************************************************************
����ԭ�ͣ�	static void Tim2_init(void)
��    �ܣ�	Tim2��ʼ��
˵		����	����PWMģʽ��Ӳ������ֱ��������ĵ����ȥ
*******************************************************************************/ 
static void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//PWMƵ�� = 72000000 / 4 / 1000 = 18Khz
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1; //PWM��������	 
	TIM_TimeBaseStructure.TIM_Prescaler = 4 - 1; //����������ΪTIM2ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ��4��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseStructure��ָ���Ĳ�����ʼ������TIM2
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR4�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //ʹ��TIM2��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2����
}

/******************************************************************************
����ԭ�ͣ�	void Motor_Init(void)
��    �ܣ�	PWM��ʼ��
˵		����
*******************************************************************************/ 
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//ʹ�ܵ���õ�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	//���õ��ʹ�õ��ùܽ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	Tim2_init();
	
}

/******************************************************************************
����ԭ�ͣ�	void Motor_Out(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4)
��    �ܣ�	�������
˵		����  PWMֵ�޽�
*******************************************************************************/ 
void Motor_Out(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4)
{
	if(duty1>1000)	duty1=1000;
	if(duty1<0)		duty1=0;
	if(duty2>1000)	duty2=1000;
	if(duty2<0)		duty2=0;
	if(duty3>1000)	duty3=1000;
	if(duty3<0)		duty3=0;
	if(duty4>1000)	duty4=1000;
	if(duty4<0)		duty4=0;
	
	TIM2->CCR1 = duty1;
	TIM2->CCR2 = duty2;
	TIM2->CCR3 = duty3;
	TIM2->CCR4 = duty4;
}

