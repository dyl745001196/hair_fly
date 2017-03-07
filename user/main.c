/***************************include包含*********************************************/
#include "stm32f10x.h"
#include "BSP_init.h"
#include "IMU.h"
#include "struct_all.h"
#include "filter.h"


/**************************宏定义:无***********************************************/
/**************************extern定义外部引用量************************************/
extern uint16_t Count_1000ms,Count_1ms,Count_2ms,Count_4ms,Count_10ms,Count_50ms;  
extern int16_t motorPWM[4];
extern PID out_loop_roll,out_loop_pitch,out_loop_yaw;
extern PID in_loop_roll,in_loop_pitch,in_loop_yaw;
extern volatile float MS5611_Altitude;
extern PID thrust,inthrust;
extern Kalmanfilter K_roll,K_pitch,K_yaw;
/**************************Struct定义结构体****************************************/
struct _SI_float	gyro_rad;    //float结构体
struct _out_angle out_angle;
struct _out_angle acc_angle;


/**************************普通变量定义********************************************/
u8 data[32];        //NRF收到的数据
u8 Sdata[32];       //NRF发送的数据
u8 Rdata[32];
char tempdata[32];
u8 NRFcount=0;
int8_t roll,pitch,yaw,start=0;
int8_t height;
u16 T_init;

/*******************************************************************************
主函数
*******************************************************************************/
void RCC_Configuration()   //时钟初始化设置
{
  ErrorStatus HSEstartupstate;
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEstartupstate=RCC_WaitForHSEStartUp();
	if(HSEstartupstate == SUCCESS)
	{
		//高速总线
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		
		//外部总线1
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		
		//外部总线2
		RCC_PCLK2Config(RCC_HCLK_Div1);
		
		//flash延时
		FLASH_SetLatency(FLASH_Latency_2);
		//flash指令预取
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
		
		RCC_PLLCmd(ENABLE);
		
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );
		
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);


		while(RCC_GetSYSCLKSource()!=0X08);
		
	}
	
}

int main()
{
	RCC_Configuration();
	//SystemInit();
	BSP_init();
/*	
	while(1)
	{
		Calculate_FilteringCoefficient(0.003f,1.0f);//计算IIR滤波器参数
		if(Count_1ms>=1)                  
		{
			MPU6050_SequenceRead();
			MPU6050_Compose();
			ACC_IIR_Filter(&acc,&filter_acc);//对acc做IIR滤波
			Gyro_Filter(&gyro,&filter_gyro);//对gyro做窗口滤波
			Get_Radian(&filter_gyro,&SI_gyro); //角速度转弧度
			IMUupdate(SI_gyro.x,SI_gyro.y,SI_gyro.z,filter_acc.x,filter_acc.y,filter_acc.z);
			Get_Eulerian_Angle(&out_angle);
			printf("%f\n",out_angle.roll);
			Count_1ms=0;
		}
	}
*/
	Calculate_FilteringCoefficient(0.002f,10.f);//计算IIR滤波器参数
	Kalman_init();
	while(1)
	{
//////////////////////////1000HZ读取姿态角/////////////////////////////
	  if(Count_1ms>=1)                  
		{
				MPU6050_SequenceRead();
				MPU6050_Compose();
				ACC_IIR_Filter(&acc,&filter_acc);//对acc做IIR滤波
				Gyro_Filter(&gyro,&filter_gyro);//对gyro做窗口滤波
				Get_Radian(&filter_gyro,&SI_gyro); //角速度转弧度
			 
		//		IMUupdate(SI_gyro.x,-SI_gyro.y,SI_gyro.z,filter_acc.x,filter_acc.y,filter_acc.z);
		//		Get_Eulerian_Angle(&out_angle);
			
			  acc_to_rad(filter_acc.x,filter_acc.y,filter_acc.z);    //Kal
			  Kalupdater(&K_roll,-SI_gyro.y,acc_angle.roll);
			  Kalupdater(&K_pitch,SI_gyro.x,acc_angle.pitch);
			  out_angle.roll =-K_roll.Angle*57.2957795f;
			  out_angle.pitch=-K_pitch.Angle*57.2957795f;
			
//				printf("gyroz %f\n",SI_gyro.z);
				roll=(int8_t)out_angle.roll;
				pitch=(int8_t)out_angle.pitch;
				yaw=(int8_t)out_angle.yaw;
			  
			  NRF24L01_RX_Mode();
				NRF24L01_RxPacket(Rdata);
				if (Rdata[0]=='u')       //start
				{
					start=1;
					T_init=700;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='j')       //stop
				{
					start=0;
					in_loop_roll.error_sum=0;
					
					in_loop_pitch.error_sum=0;
					in_loop_yaw.error_sum=0;
					out_loop_roll.error_sum=0;
					out_loop_roll.iout=out_loop_roll.ki*out_loop_roll.error_sum;
					out_loop_pitch.error_sum=0;
					out_loop_pitch.iout=out_loop_pitch.ki*out_loop_pitch.error_sum;
					out_loop_yaw.error_sum=0;
					Rdata[0]=0x00;
				}		
				else if (Rdata[0]=='l')          //land
				{
					T_init=400;
					Rdata[0]=0x00;
				}
	
				else if (Rdata[0]=='q')       //外环P+
				{
//					out_loop_roll.kp+=0.1;
  				out_loop_pitch.kp+=0.1;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='a')       //外环P-
				{
//					out_loop_roll.kp-=0.1;
		  		out_loop_pitch.kp-=0.1;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='w')        //外环I+
				{
//					out_loop_roll.ki+=0.001;
   				out_loop_pitch.ki+=0.001;
				Rdata[0]=0x00;
				}
				else if (Rdata[0]=='s')         //外环I-
				{
					out_loop_roll.ki-=0.001;
//			  	out_loop_pitch.ki-=0.001;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='e')         //外环D+
				{
					out_loop_roll.kd+=0.01;
//	    			out_loop_pitch.kd+=0.01;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='d')          //外环D-
				{
					out_loop_roll.kd-=0.01;
//			  	out_loop_pitch.kd-=0.01;
					Rdata[0]=0x00;
				}				
			

				else if (Rdata[0]=='r')       //内环P+
				{
					in_loop_roll.kp+=0.01;
//  				in_loop_pitch.kp+=0.01;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='f')       //内环P-
				{
					in_loop_roll.kp-=0.01;
//  				in_loop_pitch.kp-=0.01;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='t')        //内环I+
				{
					in_loop_roll.ki+=0.001;
//				  in_loop_pitch.ki+=0.001;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='g')         //内环I-
				{
					in_loop_roll.ki-=0.001;
				  
//					in_loop_pitch.ki-=0.001;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='y')         //内环D+
				{
					in_loop_roll.kd+=0.01;
//  				in_loop_pitch.kd+=0.01;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='h')          //内环D-
				{
					in_loop_roll.kd-=0.01;
//  				in_loop_pitch.kd-=0.01;
					Rdata[0]=0x00;
				}

				else if (Rdata[0]=='x')          //内环yaw I+
				{
					in_loop_yaw.kp+=0.01;
					Rdata[0]=0x00;
				}
				else if (Rdata[0]=='c')          //内环yaw I+
				{
					in_loop_yaw.kp-=0.01;
					Rdata[0]=0x00;
				}
		    Count_1ms=0;
			}
/////////////////////////500HZ内环控制////////////////////////////////
			if (Count_2ms>=2)
			{
				
				if(start)
				{
				in_loop_control(SI_gyro.x,SI_gyro.y,-SI_gyro.z,T_init);
	//			in_loop_control(SI_gyro.x,SI_gyro.y,-SI_gyro.z,T_init+thrust.out);
			  Motor_Out(motorPWM[0],motorPWM[1],motorPWM[2],motorPWM[3]);
				}
				Count_2ms=0;
			}
/////////////////////////250HZ外环控制////////////////////////////////
			if (Count_4ms>=4)
			{
				
				if(start)
				{
					if(T_init>550)
						T_init--;
					out_loop_control(out_angle.roll,out_angle.pitch);
					
				}
				if(start==0)
				{
					Motor_Out(0,0,0,0);
				}
				Count_4ms=0;
			}

/////////////////////////100HZ高度控制////////////////////////////////
		
		if (Count_10ms>=10)
		{
			
//		  MS5611_Thread();
//		  height=(int8_t)(MS5611_Altitude*30.0f);
			//printf("%f  \n",MS5611_Altitude);
			//thrust_control(1.0f);
			//inthrust_incontrol();
			Count_10ms=0;
		}

////////////////////////40HZ通信//////////////////////////////////////////
		if(Count_50ms>=25)
		{ 
			NRF24L01_TX_Mode();
		  Sdata[0]='A';
			sprintf(tempdata,"%f",out_angle.roll);
			Sdata[1]=tempdata[0];
			Sdata[2]=tempdata[1];
			Sdata[3]=tempdata[2];
			Sdata[4]=tempdata[3];
			Sdata[5]=tempdata[4];
		  sprintf(tempdata,"%f",out_angle.pitch);
			Sdata[6]=tempdata[0];
			Sdata[7]=tempdata[1];
			Sdata[8]=tempdata[2];
			Sdata[9]=tempdata[3];
			Sdata[10]=tempdata[4];			
		  sprintf(tempdata,"%f",out_angle.yaw);
			Sdata[11]=tempdata[0];
			Sdata[12]=tempdata[1];
			Sdata[13]=tempdata[2];
			Sdata[14]=tempdata[3];
			Sdata[15]=tempdata[4];
		
			Sdata[16]=data[1];    //PWM
			Sdata[17]=data[2];
			Sdata[18]=data[3];	
			Sdata[19]=data[4];	
			Sdata[20]=data[5];
			Sdata[21]=data[6];
			Sdata[22]=data[7];
			Sdata[23]=data[8];
			
			Sdata[24]=(int8_t)out_loop_roll.iout;
			Sdata[25]=(int8_t)out_loop_pitch.iout;
			
			sprintf(tempdata,"%f",MS5611_Altitude);
			Sdata[26]=tempdata[0];
			Sdata[27]=tempdata[1];
			Sdata[28]=tempdata[2];
			Sdata[29]=tempdata[3];
			Sdata[30]=tempdata[4];

			
			NRF24L01_TxPacket(Sdata);
			NRF24L01_RX_Mode();
		  Count_50ms=0;
		} 
//////////////////2HZ调整PID值////////////////////////////////////////////////
		if(Count_1000ms>=500)
		{
			Count_1000ms=0;

			NRF24L01_TX_Mode(); //发送当前PID
		  Sdata[0]='B';	
			Sdata[1]=in_loop_roll.kp*100;
			Sdata[2]=in_loop_roll.ki*1000;
			Sdata[3]=in_loop_roll.kd*100;
			
			Sdata[4]=in_loop_pitch.kp*100;
			Sdata[5]=in_loop_pitch.ki*1000;
			Sdata[6]=in_loop_pitch.kd*100;	
				
			Sdata[7]=in_loop_yaw.kp*100;
			Sdata[8]=in_loop_yaw.ki*1000;
				
			Sdata[9]=out_loop_roll.kp*10;
			Sdata[10]=out_loop_roll.ki*1000;
			Sdata[11]=out_loop_roll.kd*100;
				
		  Sdata[12]=out_loop_pitch.kp*10;
			Sdata[13]=out_loop_pitch.ki*1000;
			Sdata[14]=out_loop_pitch.kd*100;

			NRF24L01_TxPacket(Sdata);
			NRF24L01_RX_Mode();
			
		}
	}

}
