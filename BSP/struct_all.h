#ifndef _struct_all_H_
#define _struct_all_H_
#include "stm32f10x.h"


/******************************************************************************
							结构体声明
*******************************************************************************/ 
/* MPU6050--加速度计结构体 */
struct _acc
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _acc acc;
extern struct _acc filter_acc;
extern struct _acc offset_acc;

/* MPU6050--陀螺仪结构体 */
struct _gyro
{
	int16_t x;
	int16_t y;
	int16_t z;
};
extern struct _gyro gyro;
extern struct _gyro filter_gyro;
extern struct _gyro offset_gyro;

/* float结构体 */
struct _SI_float
{
	float x;
	float y;
	float z;
};
extern struct _SI_float SI_acc;	
extern struct _SI_float SI_gyro;

/* 姿态解算--角度值 */
struct _out_angle
{
	float yaw;
	float roll;
	float pitch;
};
extern struct _out_angle out_angle;

#endif
