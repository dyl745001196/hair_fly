#include "filter.h"

static float ACC_IIR_FACTOR;
float gyroangle;  //测试用
Kalmanfilter K_roll,K_pitch,K_yaw;
/******************************************************************************
函数原型：	void Calculate_FilteringCoefficient(float Time, float Cut_Off)
功    能：	iir低通滤波参数计算
*******************************************************************************/ 
void Calculate_FilteringCoefficient(float Time, float Cut_Off)
{
	ACC_IIR_FACTOR = Time /( Time + 1/(2.0f*3.14159f*Cut_Off) );
}

/******************************************************************************
函数原型：	void ACC_IIR_Filter(struct _acc *Acc_in,struct _acc *Acc_out)
功    能：	iir低通滤波
*******************************************************************************/ 
void ACC_IIR_Filter(struct _acc *Acc_in,struct _acc *Acc_out)
{
	Acc_out->x = Acc_out->x + ACC_IIR_FACTOR*(Acc_in->x - Acc_out->x); 
	Acc_out->y = Acc_out->y + ACC_IIR_FACTOR*(Acc_in->y - Acc_out->y); 
	Acc_out->z = Acc_out->z + ACC_IIR_FACTOR*(Acc_in->z - Acc_out->z); 
}

#define Filter_Num 2
/******************************************************************************
函数原型：	void Gyro_Filter(struct _gyro *Gyro_in,struct _gyro *Gyro_out)
功    能：	gyro窗口滑动滤波
*******************************************************************************/ 
void Gyro_Filter(struct _gyro *Gyro_in,struct _gyro *Gyro_out)
{
	static int16_t Filter_x[Filter_Num],Filter_y[Filter_Num],Filter_z[Filter_Num];
	static uint8_t Filter_count;
	int32_t Filter_sum_x=0,Filter_sum_y=0,Filter_sum_z=0;
	uint8_t i=0;
	
	Filter_x[Filter_count] = Gyro_in->x;
	Filter_y[Filter_count] = Gyro_in->y;
	Filter_z[Filter_count] = Gyro_in->z;

	for(i=0;i<Filter_Num;i++)
	{
		Filter_sum_x += Filter_x[i];
		Filter_sum_y += Filter_y[i];
		Filter_sum_z += Filter_z[i];
	}	
	
	Gyro_out->x = Filter_sum_x / Filter_Num;
	Gyro_out->y = Filter_sum_y / Filter_Num;
	Gyro_out->z = Filter_sum_z / Filter_Num;
	
	Filter_count++;
	if(Filter_count == Filter_Num)
		Filter_count=0;
}

void Kalman_init(void)
{
	K_roll.dt=0.0018;
	K_roll.Q_angle=0.002;
	K_roll.Q_bias=0;
	K_roll.Q_gyro=0.003;
	K_roll.R_angle=0.015;
	K_roll.H_0=1;
	K_roll.Angle=0;
	K_roll.PP[0][0]=1;
	K_roll.PP[1][1]=1;
	
	K_pitch.dt=0.0018;
	K_pitch.Q_angle=0.002;
	K_pitch.Q_bias=0;
	K_pitch.Q_gyro=0.003;
	K_pitch.R_angle=0.015;
	K_pitch.H_0=1;
	K_pitch.Angle=0;
	K_pitch.PP[0][0]=1;
	K_pitch.PP[1][1]=1;
}


void Kalupdater(Kalmanfilter *filter,double gyro_kal,double acc_kal)//使用卡尔曼滤波器更新四元数
{
	  double Pdot[4];//先验协方差倒数
		double Angle_err;//预测误差
		double PHt_0;		 //PH' 
		double PHt_1;
		double E;				 //E = H P H' + R
		double K_0;			 //卡尔曼增益
		double K_1;
		double Y_0;			 // Y = H P
		double Y_1;
		//状态预测,就漂移来说认为Q_bias恒定
		//  |angle | |1 -dt ||angle | |dt|
		//	|      |=|      ||      |+|  |gyro   
		//  |Q_bias| |0   1 ||Q_bias| |0 |
		filter->Angle+=(gyro_kal-filter->Q_bias)*filter->dt;
//    printf("acc %f  ",acc_kal*Radian_to_Angle);
	  //P(k|k-1)=A P(k-1| k-1) AT+Q

		filter->PP[0][0]+=(filter->Q_angle+-filter->PP[0][1]-filter->PP[1][0])*filter->dt;   // Pk-先验估计误差协方差微分的积分
		filter->PP[0][1]+=-filter->PP[1][1]*filter->dt;   // =先验估计误差协方差
		filter->PP[1][0]+=-filter->PP[1][1]*filter->dt;
		filter->PP[1][1]+=filter->Q_gyro*filter->dt;
			
		//第三个公式
		PHt_0=filter->H_0*filter->PP[0][0];
		PHt_1=filter->H_0*filter->PP[1][0];
		E=filter->R_angle+filter->H_0*PHt_0;//这里很奇怪，不懂H_0作用，按公式没有这项
		K_0=PHt_0/E;
		K_1=PHt_1/E;
		
		//第四个公式，得到滤波后结果
		Angle_err=acc_kal-filter->Angle; //测量角度与估计角度的误差
		filter->Angle+=K_0*Angle_err;	 //后验估计
		filter->Q_bias+=K_1*Angle_err;
		filter->Gyro=gyro_kal-filter->Q_bias;	 //输出值(后验估计)的微分=角速度

		gyroangle+=filter->Gyro*filter->dt;
//		printf("gyro %f angle %f\n",gyroangle*Radian_to_Angle,filter->Angle*Radian_to_Angle);

		//第五条公式P(k|k)=( I-Kg(k) H) P(k|k-1)
		Y_0 = PHt_0;
		Y_1 = filter->H_0*filter->PP[0][1];

		filter->PP[0][0]-=K_0 * Y_0;		 //后验估计误差协方差
		filter->PP[0][1]-=K_0 * Y_1;
		filter->PP[1][0]-=K_1 * Y_0;
		filter->PP[1][1]-=K_1 * Y_1;
}


