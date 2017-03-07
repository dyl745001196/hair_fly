/***************************include包含*********************************************/
#include "MS5611.h"
#include "MPU.h"
#include "math.h"


/***************************宏定义**************************************************/
#define MOVAVG_SIZE  2	         //均值滤波缓冲数
#define PA_OFFSET_INIT_NUM 300	   //用于气压计转高度值时校正计数

/*高度转换状态机*/
#define SCTemperature  0x01	   //开始温度转换
#define CTemperatureing  0x02  //正在转换温度
#define SCPressure  0x03	     //开始气压转换
#define SCPressureing  0x04	   //正在转换气压

#define MS5611Press_OSR  MS561101BA_OSR_4096  //气压采样精度
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //温度采样精度
/***************************extern定义外部引用量************************************/
/***************************Struct定义结构体****************************************/
/***************************普通变量定义********************************************/

/* 延时表  本程序用OSR4098*/
uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
};  

static uint8_t  Now_doing = SCTemperature;	               //当前状态初始化
static int32_t  tempCache;                                 //温度值
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;

/*FIFO  用于均值滤波*/
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0;                 //队列指针
static float Alt_Offset_m = 0;
static float Alt_offset_Pa=0;                              //存放着0米对应的气压值,即上电值
double paOffsetNum = 0; 
uint16_t  paInitCnt=0;
uint8_t paOffsetInited=0;
uint8_t Baro_ALT_Updated = 0;                              //气压计高度更新完成标志


void MS561101BA_NewTemp(float val)                         //添加新值到温度队列
{
	Temp_buffer[temp_index] = val;
	temp_index = (temp_index + 1) % MOVAVG_SIZE;
}


void MS561101BA_NewPress(float val)                        //添加新值到气压队列
{
	Press_buffer[press_index] = val;
	press_index = (press_index + 1) % MOVAVG_SIZE;
}


void MS561101BA_NewAlt(float val)                         //添加新值到高度队列
{
	int16_t i;
	for(i=1;i<MOVAVG_SIZE;i++)
		Alt_buffer[i-1] = Alt_buffer[i];
	if(val-Alt_buffer[MOVAVG_SIZE-2]>0.3 || val-Alt_buffer[MOVAVG_SIZE-2]<-0.3)
		Alt_buffer[MOVAVG_SIZE-1] = Alt_buffer[MOVAVG_SIZE-2];
	else Alt_buffer[MOVAVG_SIZE-1] = val;
}


float MS561101BA_getAvg(float * buff, int size)          //均值滤波
{
	float sum = 0.0;
	int i;
	for(i=0; i<size; i++) 
	{
		sum += buff[i];
	}
	return sum / size;
}



/******************************************************************************
函数原型：	void MS_reset(void)
功    能：	复位MS气压计
说		明：	
*******************************************************************************/
void MS_reset(void)
{
	if(!I2C_Start())
		printf("start error");
	
	I2C_WriteByte(0xEE);  //MS5611 ADDR地址
	if(!I2C_WaitAck())
		printf("wait error"); 
	
	I2C_WriteByte(0x1E);  //复位命令
	if(!I2C_WaitAck())
		printf("wait error");
	
	I2C_Stop();
}

/******************************************************************************
函数原型：	void MS_readprom(void)
功    能：	读取出厂标定值
说		明：	
*******************************************************************************/
static uint16_t PROM_C[6];
void MS_readprom(void)
{
	u8  inth,intl;
	uint8_t i2cret[2];
	int i;
	for (i=0;i<6;i++) 
	{
			I2C_Start();
			I2C_WriteByte(0xEE);
			I2C_WaitAck();
			I2C_WriteByte(MS561101BA_PROM_BASE_ADDR + (i *MS561101BA_PROM_REG_SIZE));
			I2C_WaitAck();	
			I2C_Stop();
			delay_us(5);
			I2C_Start();
			I2C_WriteByte(MS5611_ADDR+1);  //
			delay_us(1);
			I2C_WaitAck();
			inth = I2C_ReadByte();  //
			I2C_ACK();
			delay_us(1);
			intl = I2C_ReadByte();	 //
		  I2C_NACK();
			I2C_Stop();
			
			PROM_C[i] = (((uint16_t)inth << 8) | intl);
	}
	//printf("%d  %d  %d  %d  %d  %d ",PROM_C[0],PROM_C[1],PROM_C[2],PROM_C[3],PROM_C[4],PROM_C[5]);
}



/******************************************************************************
函数原型：	void MS5611_Init(void)
功    能：	气压计初始化
说		明：	
*******************************************************************************/
void MS5611_Init(void) 
{  
	MS_reset();  
	delay_ms(100);  
	MS_readprom(); 
}

/******************************************************************************
函数原型：	void MS_startConversion(uint8_t command)
功    能：	发送启动命令道MS
说		明：	可选的命令为MS5611101BA_D1转换气压，MS5611101BA_D2转换温度
*******************************************************************************/
void MS_startConversion(uint8_t command)
{
	I2C_Start();
	I2C_WriteByte(MS5611_ADDR);
	I2C_WaitAck();
	I2C_WriteByte(command);
	I2C_WaitAck();
	I2C_Stop();
}

uint32_t MS_getConversion(void)
{
	uint32_t conversion=0;
	u8 temp[3];
	
	I2C_Start();
	I2C_WriteByte(MS5611_ADDR);
	I2C_WaitAck();
	I2C_WriteByte(0);   //开始读取
	I2C_WaitAck();
	I2C_Stop();
	
	I2C_Start();
	I2C_WriteByte(MS5611_ADDR+1);  //进入接受模式
	I2C_WaitAck();
	
	temp[0] = I2C_ReadByte();  //  bit 23-16
	I2C_ACK();
	temp[1] = I2C_ReadByte();  //  bit 8-15
	I2C_ACK();
	temp[2] = I2C_ReadByte();  //  bit 0-7
	I2C_NACK();
	
	I2C_Stop();
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
	return conversion;
}

/******************************************************************************
函数原型：	void MS561101BA_GetTemperature(void)
功    能：	
说		明：	
*******************************************************************************/
void MS_GetTemperature(void)
{	
	tempCache = MS_getConversion();	
}

float MS561101BA_get_altitude(void)
{
	static float Altitude,AltPre;
	float dz,dt;
	uint32_t current=0;
	static uint32_t tp=0;

	if(Alt_offset_Pa == 0)                        //未初始化时
	{ 
		if(paInitCnt > PA_OFFSET_INIT_NUM)          //校正次数大于设置次数
		{
			Alt_offset_Pa = paOffsetNum / paInitCnt;	//取平均值作为0米时气压值，赋值后脱离循环
		}
		else
			paOffsetNum += MS5611_Pressure;
		
		paInitCnt++;                        
		Altitude = 0;                       //校正阶段返回高度为0
		return Altitude;
	}
	

	//此时校正已完成
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903))*0.01f;
	Altitude = Altitude + Alt_Offset_m ;  //???
	
	return Altitude; 
}



/******************************************************************************
函数原型：	void MS561101BA_GetTemperature(void)
功    能：	
说		明：	
						这里是使用MS5611官方提供的二阶温度补偿方法，crazypony官网也有
						1.读取原始数据
							D1--气压值，D2--温度值，
						2.计算温度
							dT=D2-C5*2^8，TEMP=20+dT*C6/2^23
						3.计算压力补偿值
							off=C2*2^16+(C4*dT)/2^7
							sens=C1*2^25+(C3*dT)/2^8
							P=(D1*SENS/2^21*OFF)/2^15
*******************************************************************************/
void MS_getPressure(void)
{
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS_getConversion();   //未补偿修正的气压值
	int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);  //dT=D2-C5*2^8
	float temp_H;
	
  TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;  //TEMP=20+dT*C6/2^23
	off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);
	sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);
	
	if (TEMP < 2000)
	{   // second order temperature compensation
		T2 = (((int64_t)dT)*dT) >> 31;
		Aux_64 = (TEMP-2000)*(TEMP-2000);
		OFF2 = (5*Aux_64)>>1;
		SENS2 = (5*Aux_64)>>2;
		TEMP = TEMP - T2;
		off = off - OFF2;
		sens = sens - SENS2;
	}
	
	MS5611_Pressure=(((((int64_t)rawPress)*sens)>>21)-off)/32768;  //P
	
	MS561101BA_NewTemp(TEMP*0.01f);                                //温度队列，温度很稳定     
//	printf("T %f",TEMP*0.01);
	MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c
	
	MS5611_Altitude = MS561101BA_get_altitude(); // ??:m 
	temp_H=MS561101BA_get_altitude();
	if(temp_H-Alt_buffer[MOVAVG_SIZE-1]>0.1)
		MS561101BA_NewAlt(Alt_buffer[MOVAVG_SIZE-1]+0.1);
	else if(temp_H-Alt_buffer[MOVAVG_SIZE-1]<-1)
		MS561101BA_NewAlt(Alt_buffer[MOVAVG_SIZE-1]-0.1);
	else MS561101BA_NewAlt(MS561101BA_get_altitude());
	MS5611_Altitude=MS561101BA_getAvg(Alt_buffer,MOVAVG_SIZE);
	//printf("H %f\n",MS5611_Altitude);
}


/**************************????********************************************

*******************************************************************************/
void MS5611_Thread(void) 
{
	switch(Now_doing)
	{ 
		case SCTemperature:  
			MS_startConversion(0x58);       //D2  开启温度转换
			Now_doing = CTemperatureing;    //状态更新
			break;
		case CTemperatureing:             //限制该程序调用的频率，超过9.1ms，直接可读取温度
				MS_GetTemperature();          //读取温度
				Now_doing = SCPressure;	
			break;
		case SCPressure:                  //D1  启动气压转换
			MS_startConversion(0x48);
			Now_doing = SCPressureing;      //状态更新
			break;
		case SCPressureing:	              
				MS_getPressure();            //读取气压
				Baro_ALT_Updated = 0xff; 	   //高度更新完成
				Now_doing = SCTemperature;   //进入下一个循环
			break;
		
		default: 
			Now_doing = SCTemperature;
			break;
	}
	
	
}
