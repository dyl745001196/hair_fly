#ifndef _IMU_H
#define _IMU_H

#include "stdint.h"
#include "struct_all.h"
/******************************************************************************
							结构体定义
*******************************************************************************/ 


extern float q0 , q1 , q2 , q3 ;     	    // 四元数


void Get_Radian(struct _gyro *Gyro_in,struct _SI_float *Gyro_out);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void Get_Eulerian_Angle(struct _out_angle *angle);
void acc_to_rad(int16_t ax,int16_t ay,int16_t az);
#endif
