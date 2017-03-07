/***************************include包含*********************************************/
#include "Printf.h"


/***************************宏定义**************************************************/
/***************************extern定义外部引用量************************************/
/***************************Struct定义结构体****************************************/
/***************************普通变量定义********************************************/


/******************************************************************************
函数原型：	int fputc(int ch,FILE *p)
功    能：	printf调用
说		明：	函数默认的，在使用printf函数时自动调用
*******************************************************************************/ 
int fputc(int ch,FILE *p) 
{
	USART_SendData(USART1,(u8)ch);	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

/******************************************************************************
函数原型：	printf_init()
功    能：	IO口及串口初始化
说		明：	在72M系统时钟下才是1ms延时
*******************************************************************************/ 
void printf_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	//声明一个结构体变量，用来初始化GPIO
	USART_InitTypeDef  USART_InitStructure;	  //串口结构体定义

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//TX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//RX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
 
	USART_InitStructure.USART_BaudRate=115200;   //波特率设置为115200
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//使能或者失能指定的USART中断 接收中断
	USART_ClearFlag(USART1,USART_FLAG_TC);//清除USARTx的待处理标志位
}
