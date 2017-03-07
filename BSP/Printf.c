/***************************include����*********************************************/
#include "Printf.h"


/***************************�궨��**************************************************/
/***************************extern�����ⲿ������************************************/
/***************************Struct����ṹ��****************************************/
/***************************��ͨ��������********************************************/


/******************************************************************************
����ԭ�ͣ�	int fputc(int ch,FILE *p)
��    �ܣ�	printf����
˵		����	����Ĭ�ϵģ���ʹ��printf����ʱ�Զ�����
*******************************************************************************/ 
int fputc(int ch,FILE *p) 
{
	USART_SendData(USART1,(u8)ch);	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

/******************************************************************************
����ԭ�ͣ�	printf_init()
��    �ܣ�	IO�ڼ����ڳ�ʼ��
˵		����	��72Mϵͳʱ���²���1ms��ʱ
*******************************************************************************/ 
void printf_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;	//����һ���ṹ�������������ʼ��GPIO
	USART_InitTypeDef  USART_InitStructure;	  //���ڽṹ�嶨��

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//TX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//RX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
 
	USART_InitStructure.USART_BaudRate=115200;   //����������Ϊ115200
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//ʹ�ܻ���ʧ��ָ����USART�ж� �����ж�
	USART_ClearFlag(USART1,USART_FLAG_TC);//���USARTx�Ĵ������־λ
}
