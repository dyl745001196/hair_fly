#ifndef _MotorControl_H
#define _MotorControl_H
#include "stm32f10x.h"

typedef struct _PID
{
	float	kp;
	float ki;
	float kd;
	float pout;
	float iout;
	float dout;
	float out;
	float error_now;
	float error_last;
	float error_sum;
	float error_satur;
	
}PID;




void pid_init(void);
void out_loop_control(float roll, float pitch);
void in_loop_control(float gyro_x,float gyro_y,float gyro_z,float throttle);
void thrust_control(float height);
void inthrust_incontrol(void);


#endif
