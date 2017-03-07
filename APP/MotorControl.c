/***************************include包含*********************************************/
#include "MotorControl.h"
#include "IMU.h"   //引入gyro_rad定义
#include "Motor.h"
#include "Printf.h"


/***************************宏定义**************************************************/
/***************************extern定义外部引用量************************************/
extern u16 THROTTLE;	//油门量
extern u8 data[32];           //要发送的32位数据
extern u8 Sdata[32];
extern volatile float MS5611_Altitude;

/***************************Struct定义结构体****************************************/
PID out_loop_roll,out_loop_pitch,out_loop_yaw;	//外环控制量,角度
PID in_loop_roll,in_loop_pitch,in_loop_yaw;
PID thrust,inthrust;

/***************************普通变量定义********************************************/
int16_t motorPWM[4];   //四电机PWM值

/******************************************************************************
函数原型:	pid_init()
功　　能:	初始化双环PID的参数
说		明:
*******************************************************************************/ 
void pid_init()
{
	out_loop_roll.kp=3.6;
	out_loop_roll.ki=0.02;
	out_loop_roll.kd=0;
	out_loop_roll.error_sum=0;
	out_loop_roll.error_last=0;

	out_loop_pitch.kp=3.6;
	out_loop_pitch.ki=0.02;
	out_loop_pitch.kd=0;
	out_loop_pitch.error_sum=0;
	out_loop_pitch.error_last=0;

	out_loop_yaw.kp=0;
	out_loop_yaw.ki=0;
	out_loop_yaw.kd=0;
	out_loop_yaw.error_sum=0;
	out_loop_yaw.error_last=0;//外环参数的初始化

	in_loop_roll.kp=0.76;
	in_loop_roll.ki=0;
	in_loop_roll.kd=0.6;
	in_loop_roll.error_sum=0;
	in_loop_roll.error_last=0;

	in_loop_pitch.kp=0.76;
	in_loop_pitch.ki=0;
	in_loop_pitch.kd=0.6;
	in_loop_pitch.error_sum=0;
	in_loop_pitch.error_last=0;

	in_loop_yaw.kp=1.5;
	in_loop_yaw.ki=0;
	in_loop_yaw.kd=0;		
	in_loop_yaw.error_sum=0;
	in_loop_yaw.error_last=0;//内环参数的初始化
	
	thrust.kp=80;
	thrust.ki=1;
	thrust.kd=0;
	thrust.error_sum=0;
	thrust.error_last=0;
	
	inthrust.kp=0;
	inthrust.ki=0;
	inthrust.kd=0;
	inthrust.error_sum=0;
	inthrust.error_last=0;
	
}




/******************************************************************************
函数原型：	void thrust_control(float height)
功    能：	高度控制
说		明：
*******************************************************************************/ 
void thrust_control(float height)	
{
///////////////////////Thrust////////////////////////////	
	thrust.error_now=height-MS5611_Altitude;           //P
	
	if (thrust.error_now>0.3f)      //积分上限控制
		thrust.error_sum+=0.3f;
	else if (thrust.error_now<-0.3f)
		thrust.error_sum+=-0.3;
	else
		thrust.error_sum+=thrust.error_now;  //I
	
	if(thrust.error_sum>30.0f)
		thrust.error_sum=30.0f;
	else if(thrust.error_sum<-30.0f)
		thrust.error_sum=-30.0f;

///////////////////////PID////////////////////////////////	
	thrust.pout=thrust.kp*thrust.error_now;
	thrust.iout=thrust.ki*thrust.error_sum;
  thrust.dout=thrust.kd*(thrust.error_now-thrust.error_last);
	thrust.out= thrust.pout  + thrust.iout + thrust.dout;
	
  
///////////////////////更新Error//////////////////////////
	thrust.error_last=thrust.error_now;
}

float thrust_last=0;
void inthrust_incontrol()
{
	inthrust.error_now=thrust.out-(thrust.error_now-thrust.error_last)*100;
	
//	printf("%f\n",(thrust.error_now-thrust.error_last)*100);
	if (inthrust.error_now>3.0f)      //积分上限控制
		inthrust.error_sum+=3.0f;
	else if (inthrust.error_now<-3.0f)
		inthrust.error_sum+=-3.0;
	else
		inthrust.error_sum+=inthrust.error_now;  //I
	
	if(inthrust.error_sum>300.0f)
		inthrust.error_sum=300.0f;
	else if(inthrust.error_sum<-300.0f)
		inthrust.error_sum=-300.0f;  
	
	inthrust.pout=inthrust.kp*inthrust.error_now;
	inthrust.iout=inthrust.ki*inthrust.error_sum;
  inthrust.dout=inthrust.kd*(inthrust.error_now-inthrust.error_last);
	inthrust.out =inthrust.pout+inthrust.iout+inthrust.dout;
	
///////////////////////更新Error//////////////////////////
	inthrust.error_last=inthrust.error_now;

	
}

/******************************************************************************
函数原型：	void out_loop_control(float roll, float pitch)
功    能：	外环控制，输入角度量，控制频率为250HZ
说		明：
*******************************************************************************/ 
void out_loop_control(float roll, float pitch)	
{
///////////////////////Roll////////////////////////////	
	out_loop_roll.error_now=0-roll;           //P
	
	if (out_loop_roll.error_now>10.0f)      //积分上限控制
		out_loop_roll.error_sum+=10.0f;
	else if (out_loop_roll.error_now<-10.0f)
		out_loop_roll.error_sum+=-10.0f;
	else
		out_loop_roll.error_sum+=out_loop_roll.error_now;  //I
	
	if(out_loop_roll.error_sum>1000.0f)
		out_loop_roll.error_sum=1000.0f;
	else if(out_loop_roll.error_sum<-1000.0f)
		out_loop_roll.error_sum=-1000.0f;
	
	
///////////////////////Pitch////////////////////////////	
	out_loop_pitch.error_now=0-pitch;           //P
	
	if (out_loop_pitch.error_now>10.0f)       //积分上限控制
	{
		out_loop_pitch.error_sum+=10.0f;
	}
	else if (out_loop_pitch.error_now<-10.0f)
	{
		out_loop_pitch.error_sum+=-10.0f;
	}
	else
		out_loop_pitch.error_sum+=out_loop_pitch.error_now;  //I

	if(out_loop_pitch.error_sum>1000.0f)
		out_loop_pitch.error_sum=1000.0f;
	else if(out_loop_pitch.error_sum<-1000.0f)
		out_loop_pitch.error_sum=-1000.0f;


///////////////////////PID////////////////////////////////	
	out_loop_roll.pout=out_loop_roll.kp*out_loop_roll.error_now;
	out_loop_roll.iout=out_loop_roll.ki*out_loop_roll.error_sum;
  out_loop_roll.dout=out_loop_roll.kd*(out_loop_roll.error_now-out_loop_roll.error_last);
	out_loop_roll.out =out_loop_roll.pout+out_loop_roll.iout+out_loop_roll.dout;
	
	out_loop_pitch.pout=out_loop_pitch.kp*out_loop_pitch.error_now;
	out_loop_pitch.iout=out_loop_pitch.ki*out_loop_pitch.error_sum;
	out_loop_pitch.dout=out_loop_pitch.kd*(out_loop_pitch.error_now-out_loop_pitch.error_last);
	out_loop_pitch.out =out_loop_pitch.pout+out_loop_pitch.iout+out_loop_pitch.dout;
///////////////////////更新Error//////////////////////////
	out_loop_roll.error_last=out_loop_roll.error_now;
	out_loop_pitch.error_last=out_loop_pitch.error_now;
}

/******************************************************************************
函数原型：	void in_loop_control(float gyro_x,float gyro_y,float gyro_z,float throttle)	
功    能：	内环控制，输入角度量，控制频率为500HZ
说		明：
*******************************************************************************/ 
#define Radian_to_Angle 57.2957795f
void in_loop_control(float gyro_x,float gyro_y,float gyro_z,float throttle)	
{
///////////////////////Roll////////////////////////////	
	in_loop_roll.error_now=out_loop_roll.out-gyro_y*Radian_to_Angle;
	in_loop_pitch.error_now=out_loop_pitch.out+gyro_x*Radian_to_Angle;
	in_loop_yaw.error_now=-gyro_z*Radian_to_Angle;
	
	if(in_loop_roll.error_now>50.0f)
		in_loop_roll.error_sum+=50.0f;
	else if(in_loop_roll.error_now<-50.0f)
		in_loop_roll.error_sum+=-50.0f;
	else
		in_loop_roll.error_sum+=in_loop_roll.error_now;
	
	if(in_loop_roll.error_sum>5000.0f)
		in_loop_roll.error_sum=5000.0f;
	else if(in_loop_roll.error_sum<-5000.0f)
		in_loop_roll.error_sum=-5000.0f;
///////////////////////Pitch////////////////////////////	
	if(in_loop_pitch.error_now>50.0f)
		in_loop_pitch.error_sum+=50.0f;
	else if(in_loop_pitch.error_now<-50.0f)
		in_loop_pitch.error_sum+=-50.0f;
	else
		in_loop_pitch.error_sum+=in_loop_pitch.error_now;
	
	if(in_loop_pitch.error_sum>5000.0f)
		in_loop_pitch.error_sum=5000.0f;
	else if(in_loop_pitch.error_sum<-5000.0f)
		in_loop_pitch.error_sum=-5000.0f;
///////////////////////Yaw////////////////////////////	
	if(in_loop_yaw.error_now>50.0f)
		in_loop_yaw.error_sum+=50.0f;
	else if(in_loop_yaw.error_now<-50.0f)
		in_loop_yaw.error_sum+=-50.0f;
	else
		in_loop_yaw.error_sum+=in_loop_yaw.error_now;
	
	if(in_loop_yaw.error_sum>5000.0f)
		in_loop_yaw.error_sum=5000.0f;
	else if(in_loop_yaw.error_sum<-5000.0f)
		in_loop_yaw.error_sum=-5000.0f;

///////////////////////PID////////////////////////////////	
	in_loop_roll.pout=in_loop_roll.kp*in_loop_roll.error_now;
	in_loop_roll.iout=in_loop_roll.ki*in_loop_roll.error_sum;
  in_loop_roll.dout=in_loop_roll.kd*(in_loop_roll.error_now-in_loop_roll.error_last);
	in_loop_roll. out=in_loop_roll.pout+in_loop_roll.iout+in_loop_roll.dout;
	
	in_loop_pitch.pout=in_loop_pitch.kp*in_loop_pitch.error_now;
	in_loop_pitch.iout=in_loop_pitch.ki*in_loop_pitch.error_sum;
	in_loop_pitch.dout=in_loop_pitch.kd*(in_loop_pitch.error_now-in_loop_pitch.error_last);
	in_loop_pitch.out= in_loop_pitch.pout+in_loop_pitch.iout+in_loop_pitch.dout;

	in_loop_yaw.pout=in_loop_yaw.kp*in_loop_yaw.error_now;
	in_loop_yaw.iout=in_loop_yaw.ki*in_loop_yaw.error_sum;
	in_loop_yaw.dout=in_loop_yaw.kd*(in_loop_yaw.error_now-in_loop_yaw.error_last);
	in_loop_yaw.out= in_loop_yaw.pout+in_loop_yaw.iout+in_loop_yaw.dout;
///////////////////////更新Error//////////////////////////
	in_loop_roll.error_last=in_loop_roll.error_now;
	in_loop_pitch.error_last=in_loop_pitch.error_now;
	in_loop_yaw.error_last=in_loop_yaw.error_now;
	
	motorPWM[0] = throttle + in_loop_roll.out - in_loop_pitch.out + in_loop_yaw.out; //后右
	motorPWM[1] = throttle + in_loop_roll.out + in_loop_pitch.out - in_loop_yaw.out; //前右
	motorPWM[2] = throttle - in_loop_roll.out + in_loop_pitch.out + in_loop_yaw.out; //后左
	motorPWM[3] = throttle - in_loop_roll.out - in_loop_pitch.out - in_loop_yaw.out; //前左

	data[1]=(uint8_t)(motorPWM[0]/256);
	data[2]=(uint8_t)(motorPWM[0]%256);
	
	data[3]=(uint8_t)(motorPWM[1]/256);
	data[4]=(uint8_t)(motorPWM[1]%256);
	
	data[5]=(uint8_t)(motorPWM[2]/256);
	data[6]=(uint8_t)(motorPWM[2]%256);
	
	data[7]=(uint8_t)(motorPWM[3]/256);
	data[8]=(uint8_t)(motorPWM[3]%256);
}

