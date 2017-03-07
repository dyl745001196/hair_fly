/***************************include包含*********************************************/
#include "Motor.h"


/***************************宏定义**************************************************/
/***************************extern定义外部引用量************************************/
/***************************Struct定义结构体****************************************/
/***************************普通变量定义********************************************/



/******************************************************************************
函数原型：	static void Tim2_init(void)
功    能：	Tim2初始化
说		明：	采用PWM模式，硬件连接直接输出到四电机上去
*******************************************************************************/ 
static void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//PWM频率 = 72000000 / 4 / 1000 = 18Khz
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1; //PWM计数上限	 
	TIM_TimeBaseStructure.TIM_Prescaler = 4 - 1; //设置用来作为TIM2时钟频率除数的预分频值，4分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseStructure中指定的参数初始化外设TIM2
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR1上的预装载寄存器
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR2上的预装载寄存器
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR3上的预装载寄存器
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR4上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIM2在ARR上的预装载寄存器
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2外设
}

/******************************************************************************
函数原型：	void Motor_Init(void)
功    能：	PWM初始化
说		明：
*******************************************************************************/ 
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	Tim2_init();
	
}

/******************************************************************************
函数原型：	void Motor_Out(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4)
功    能：	电机驱动
说		明：  PWM值限界
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

