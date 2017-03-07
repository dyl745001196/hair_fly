/***************************include包含*********************************************/
#include "Tim3.h"


/***************************宏定义**************************************************/
/***************************extern定义外部引用量************************************/
/***************************Struct定义结构体****************************************/
/***************************普通变量定义********************************************/


/******************************************************************************
函数原型:	void Timer3_Init(void)
功　　能:	初始化Tim3
说		明: 用于任务调度
*******************************************************************************/ 
void Timer3_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 1000 ;//装载值
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	//分频系数
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //不分割时钟
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//清除中断标志
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);//使能定时器3
}
