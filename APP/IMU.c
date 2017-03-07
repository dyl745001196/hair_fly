/***************************include包含*********************************************/
#include "IMU.h"
#include "math.h"


/***************************宏定义**************************************************/
#define Pi	3.1415927f
#define Radian_to_Angle	   57.2957795f
#define RawData_to_Angle	0.0610351f	//以下参数对应2000度每秒
#define RawData_to_Radian	0.0010653f

#define Kp 	30.0f    // 比例常数
#define Ki 	0.005f  // 积分常数
#define halfT 0.0015f//半周期
#define T	0.003f  // 周期为1ms


/***************************extern定义外部引用量************************************/
extern struct _out_angle acc_angle;
/***************************Struct定义结构体****************************************/
/***************************普通变量定义********************************************/
float exInt , eyInt , ezInt ;    	// 误差积分累计误差


/******************************************************************************
函数原型:	float invSqrt(float)
功　　能:	y=1/sqrt(x)
说		明:
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
函数原型：	void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
功    能：	角速度由原始数据转为弧度
说		明:
*******************************************************************************/ 
void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out)
{
	Gyro_out->x = (float)(Gyro_in->x * RawData_to_Radian);
	Gyro_out->y = (float)(Gyro_in->y * RawData_to_Radian);
	Gyro_out->z = (float)(Gyro_in->z * RawData_to_Radian);
}


/******************************************************************************
函数原型：	void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
功    能：	互补滤波进行姿态解算
说		明:		输入--陀螺仪数据及加速度计数据，输出：四元数
*******************************************************************************/
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float delta2 = (gx*gx + gy*gy + gz*gz)*T*T;
	
  //四元数积分，求得当前的姿态
	float q0_last = q0;	
	float q1_last = q1;	
	float q2_last = q2;	
	float q3_last = q3;	

	//把加速度计的三维向量转成单位向量
	norm = invSqrt(ax*ax + ay*ay + az*az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	//估计重力加速度方向在飞行器坐标系中的表示，为四元数表示的旋转矩阵的第三行
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	//加速度计读取的方向与重力加速度方向的差值，用向量叉乘计算
	ex = ay*vz - az*vy;
	ey = az*vx - ax*vz;
	ez = ax*vy - ay*vx;

	//误差累积，已与积分常数相乘
	exInt = exInt + ex*Ki;
	eyInt = eyInt + ey*Ki;
	ezInt = ezInt + ez*Ki;

	//用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;

	//一阶近似算法
//	q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
//	q1 = q1_last + ( q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
//	q2 = q2_last + ( q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
//	q3 = q3_last + ( q0_last*gz + q1_last*gy - q2_last*gx)*halfT; 
	
	//四阶近似算法
	
	q0 = q0_last*(1 - delta2/8 + delta2*delta2/384) + (-q1_last*gx - q2_last*gy - q3_last*gz)*T*(0.5 - delta2/48);
	q1 = q1_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gx + q2_last*gz - q3_last*gy)*T*(0.5 - delta2/48);
	q2 = q2_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gy - q1_last*gz + q3_last*gx)*T*(0.5 - delta2/48);
	q3 = q3_last*(1 - delta2/8 + delta2*delta2/384) + ( q0_last*gz + q1_last*gy - q2_last*gx)*T*(0.5 - delta2/48);
	
	//四元数规范化
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
}

/******************************************************************************
函数原型：	void Get_Eulerian_Angle(struct _out_angle *angle)
功    能：	四元数转欧拉角
说		明：
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
	//归一化
	norm = invSqrt(ax*ax + ay*ay + az*az);
	axfloat = (float)ax * norm;
	ayfloat = (float)ay * norm;
	azfloat = (float)az * norm;
	acc_angle.roll=(float)asin(axfloat);
	acc_angle.pitch=(float)asin(ayfloat);
}