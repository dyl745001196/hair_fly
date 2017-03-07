#ifndef _filter_H_
#define _filter_H_
#include "stm32f10x.h"
#include "struct_all.h"

/******************************************************************************
							结构体声明
*******************************************************************************/ 
typedef struct 
{
  double Q_angle;  //陀螺仪噪声协方差
	double Q_gyro;	 //陀螺仪漂移噪声协方差
	double R_angle;	 //加速度计协方差
	double dt;	     //采样时间
	char H_0; 			 //测量增益
	double Q_bias;	 //陀螺仪漂移
	double PP[2][2]; //先验协方差
	double Angle;  //得出的角度
	double Gyro;   //得出的角速度
}Kalmanfilter;


void ACC_IIR_Filter(struct _acc *Acc_in,struct _acc *Acc_out);
void Gyro_Filter(struct _gyro *Gyro_in,struct _gyro *Gyro_out);
void Calculate_FilteringCoefficient(float Time, float Cut_Off);
void Kalman_init(void);
void Kalupdater(Kalmanfilter *filter,double gyro_kal,double acc_kal);

#endif
