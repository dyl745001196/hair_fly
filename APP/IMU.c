/***************************include����*********************************************/
#include "IMU.h"
#include "math.h"


/***************************�궨��**************************************************/
#define Pi	3.1415927f
#define Radian_to_Angle	   57.2957795f
#define RawData_to_Angle	0.0610351f	//���²�����Ӧ2000��ÿ��
#define RawData_to_Radian	0.0010653f

#define Kp 	30.0f    // ��������
#define Ki 	0.005f  // ���ֳ���
#define halfT 0.0015f//������
#define T	0.003f  // ����Ϊ1ms


/***************************extern�����ⲿ������************************************/
extern struct _out_angle acc_angle;
/***************************Struct����ṹ��****************************************/
/***************************��ͨ��������********************************************/
float exInt , eyInt , ezInt ;    	// �������ۼ����


/******************************************************************************
����ԭ��:	float invSqrt(float)
��������:	y=1/sqrt(x)
˵		��:
*******************************************************************************/ 
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/******************************************************************************
����ԭ�ͣ�	void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
��    �ܣ�	���ٶ���ԭʼ����תΪ����
˵		��:
*******************************************************************************/ 
void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
{
	Gyro_out->x = (float)(Gyro_in->x * RawData_to_Radian);
	Gyro_out->y = (float)(Gyro_in->y * RawData_to_Radian);
	Gyro_out->z = (float)(Gyro_in->z * RawData_to_Radian);
}


/******************************************************************************
����ԭ�ͣ�	void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
��    �ܣ�	�����˲�������̬����
˵		��:		����--���������ݼ����ٶȼ����ݣ��������Ԫ��
*******************************************************************************/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float delta2 = (gx*gx + gy*gy + gz*gz)*T*T;
	
  //��Ԫ�����֣���õ�ǰ����̬
	float q0_last = q0;	
	float q1_last = q1;	
	float q2_last = q2;	
	float q3_last = q3;	

	//�Ѽ��ٶȼƵ���ά����ת�ɵ�λ����
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//�����������ٶȷ����ڷ���������ϵ�еı�ʾ��Ϊ��Ԫ����ʾ����ת����ĵ�����
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//���ٶȼƶ�ȡ�ķ������������ٶȷ���Ĳ�ֵ����������˼���
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;

	//����ۻ���������ֳ������
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//�ò���������PI����������ƫ�����������ݶ����е�ƫ����	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	//һ�׽����㷨
//	q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
//	q1 = q1_last + ( q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
//	q2 = q2_last + ( q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
//	q3 = q3_last + ( q0_last*gz + q1_last*gy - q2_last*gx)*halfT; 
	
	//�Ľ׽����㷨
	
	q0 = q0_last*(1 - delta2/8 + delta2*delta2/384) + (-q1_last*gx - q2_last*gy - q3_last*gz)*T*(0.5 - delta2/48);
	q1 = q1_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gx + q2_last*gz - q3_last*gy)*T*(0.5 - delta2/48);
	q2 = q2_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gy - q1_last*gz + q3_last*gx)*T*(0.5 - delta2/48);
	q3 = q3_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gz + q1_last*gy - q2_last*gx)*T*(0.5 - delta2/48);
	
	//��Ԫ���淶��
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
}

/******************************************************************************
����ԭ�ͣ�	void Get_Eulerian_Angle(struct _out_angle *angle)
��    �ܣ�	��Ԫ��תŷ����
˵		����
*******************************************************************************/ 
void Get_Eulerian_Angle(struct _out_angle *angle)
{	
	angle->pitch = -atan2(2.0f*(q0*q1 + q2*q3),q0*q0 - q1*q1 - q2*q2 + q3*q3)*Radian_to_Angle;
	angle->roll  =  asin (2.0f*(q0*q2 - q1*q3))*Radian_to_Angle;
	angle->yaw 		=atan2(2*(q0*q1+q2*q3),q0*q0+q1*q1-q2*q2-q3*q3)*Radian_to_Angle;
}

void acc_to_rad(int16_t ax,int16_t ay,int16_t az)
{
	float norm;
	float axfloat,ayfloat,azfloat;
	//��һ��
	norm = invSqrt(ax*ax + ay*ay + az*az);
	axfloat = (float)ax * norm;
	ayfloat = (float)ay * norm;
	azfloat = (float)az * norm;
	acc_angle.roll=(float)asin(axfloat);
	acc_angle.pitch=(float)asin(ayfloat);
}