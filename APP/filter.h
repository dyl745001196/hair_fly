#ifndef _filter_H_
#define _filter_H_
#include "stm32f10x.h"
#include "struct_all.h"

/******************************************************************************
							�ṹ������
*******************************************************************************/ 
typedef struct 
{
  double Q_angle;  //����������Э����
	double Q_gyro;	 //������Ư������Э����
	double R_angle;	 //���ٶȼ�Э����
	double dt;	     //����ʱ��
	char H_0; 			 //��������
	double Q_bias;	 //������Ư��
	double PP[2][2]; //����Э����
	double Angle;  //�ó��ĽǶ�
	double Gyro;   //�ó��Ľ��ٶ�
}Kalmanfilter;


void ACC_IIR_Filter(struct _acc *Acc_in,struct _acc *Acc_out);
void Gyro_Filter(struct _gyro *Gyro_in,struct _gyro *Gyro_out);
void Calculate_FilteringCoefficient(float Time, float Cut_Off);
void Kalman_init(void);
void Kalupdater(Kalmanfilter *filter,double gyro_kal,double acc_kal);

#endif
