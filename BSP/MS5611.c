/***************************include����*********************************************/
#include "MS5611.h"
#include "MPU.h"
#include "math.h"


/***************************�궨��**************************************************/
#define MOVAVG_SIZE  2	         //��ֵ�˲�������
#define PA_OFFSET_INIT_NUM 300	   //������ѹ��ת�߶�ֵʱУ������

/*�߶�ת��״̬��*/
#define SCTemperature  0x01	   //��ʼ�¶�ת��
#define CTemperatureing  0x02  //����ת���¶�
#define SCPressure  0x03	     //��ʼ��ѹת��
#define SCPressureing  0x04	   //����ת����ѹ

#define MS5611Press_OSR  MS561101BA_OSR_4096  //��ѹ��������
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //�¶Ȳ�������
/***************************extern�����ⲿ������************************************/
/***************************Struct����ṹ��****************************************/
/***************************��ͨ��������********************************************/

/* ��ʱ��  ��������OSR4098*/
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

static uint8_t  Now_doing = SCTemperature;	               //��ǰ״̬��ʼ��
static int32_t  tempCache;                                 //�¶�ֵ
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;

/*FIFO  ���ھ�ֵ�˲�*/
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0;                 //����ָ��
static float Alt_Offset_m = 0;
static float Alt_offset_Pa=0;                              //�����0�׶�Ӧ����ѹֵ,���ϵ�ֵ
double paOffsetNum = 0; 
uint16_t  paInitCnt=0;
uint8_t paOffsetInited=0;
uint8_t Baro_ALT_Updated = 0;                              //��ѹ�Ƹ߶ȸ�����ɱ�־


void MS561101BA_NewTemp(float val)                         //�����ֵ���¶ȶ���
{
	Temp_buffer[temp_index] = val;
	temp_index = (temp_index + 1) % MOVAVG_SIZE;
}


void MS561101BA_NewPress(float val)                        //�����ֵ����ѹ����
{
	Press_buffer[press_index] = val;
	press_index = (press_index + 1) % MOVAVG_SIZE;
}


void MS561101BA_NewAlt(float val)                         //�����ֵ���߶ȶ���
{
	int16_t i;
	for(i=1;i<MOVAVG_SIZE;i++)
		Alt_buffer[i-1] = Alt_buffer[i];
	if(val-Alt_buffer[MOVAVG_SIZE-2]>0.3 || val-Alt_buffer[MOVAVG_SIZE-2]<-0.3)
		Alt_buffer[MOVAVG_SIZE-1] = Alt_buffer[MOVAVG_SIZE-2];
	else Alt_buffer[MOVAVG_SIZE-1] = val;
}


float MS561101BA_getAvg(float * buff, int size)          //��ֵ�˲�
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
����ԭ�ͣ�	void MS_reset(void)
��    �ܣ�	��λMS��ѹ��
˵		����	
*******************************************************************************/
void MS_reset(void)
{
	if(!I2C_Start())
		printf("start error");
	
	I2C_WriteByte(0xEE);  //MS5611 ADDR��ַ
	if(!I2C_WaitAck())
		printf("wait error"); 
	
	I2C_WriteByte(0x1E);  //��λ����
	if(!I2C_WaitAck())
		printf("wait error");
	
	I2C_Stop();
}

/******************************************************************************
����ԭ�ͣ�	void MS_readprom(void)
��    �ܣ�	��ȡ�����궨ֵ
˵		����	
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
����ԭ�ͣ�	void MS5611_Init(void)
��    �ܣ�	��ѹ�Ƴ�ʼ��
˵		����	
*******************************************************************************/
void MS5611_Init(void) 
{  
	MS_reset();  
	delay_ms(100);  
	MS_readprom(); 
}

/******************************************************************************
����ԭ�ͣ�	void MS_startConversion(uint8_t command)
��    �ܣ�	�������������MS
˵		����	��ѡ������ΪMS5611101BA_D1ת����ѹ��MS5611101BA_D2ת���¶�
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
	I2C_WriteByte(0);   //��ʼ��ȡ
	I2C_WaitAck();
	I2C_Stop();
	
	I2C_Start();
	I2C_WriteByte(MS5611_ADDR+1);  //�������ģʽ
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
����ԭ�ͣ�	void MS561101BA_GetTemperature(void)
��    �ܣ�	
˵		����	
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

	if(Alt_offset_Pa == 0)                        //δ��ʼ��ʱ
	{ 
		if(paInitCnt > PA_OFFSET_INIT_NUM)          //У�������������ô���
		{
			Alt_offset_Pa = paOffsetNum / paInitCnt;	//ȡƽ��ֵ��Ϊ0��ʱ��ѹֵ����ֵ������ѭ��
		}
		else
			paOffsetNum += MS5611_Pressure;
		
		paInitCnt++;                        
		Altitude = 0;                       //У���׶η��ظ߶�Ϊ0
		return Altitude;
	}
	

	//��ʱУ�������
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.1903))*0.01f;
	Altitude = Altitude + Alt_Offset_m ;  //???
	
	return Altitude; 
}



/******************************************************************************
����ԭ�ͣ�	void MS561101BA_GetTemperature(void)
��    �ܣ�	
˵		����	
						������ʹ��MS5611�ٷ��ṩ�Ķ����¶Ȳ���������crazypony����Ҳ��
						1.��ȡԭʼ����
							D1--��ѹֵ��D2--�¶�ֵ��
						2.�����¶�
							dT=D2-C5*2^8��TEMP=20+dT*C6/2^23
						3.����ѹ������ֵ
							off=C2*2^16+(C4*dT)/2^7
							sens=C1*2^25+(C3*dT)/2^8
							P=(D1*SENS/2^21*OFF)/2^15
*******************************************************************************/
void MS_getPressure(void)
{
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS_getConversion();   //δ������������ѹֵ
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
	
	MS561101BA_NewTemp(TEMP*0.01f);                                //�¶ȶ��У��¶Ⱥ��ȶ�     
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
			MS_startConversion(0x58);       //D2  �����¶�ת��
			Now_doing = CTemperatureing;    //״̬����
			break;
		case CTemperatureing:             //���Ƹó�����õ�Ƶ�ʣ�����9.1ms��ֱ�ӿɶ�ȡ�¶�
				MS_GetTemperature();          //��ȡ�¶�
				Now_doing = SCPressure;	
			break;
		case SCPressure:                  //D1  ������ѹת��
			MS_startConversion(0x48);
			Now_doing = SCPressureing;      //״̬����
			break;
		case SCPressureing:	              
				MS_getPressure();            //��ȡ��ѹ
				Baro_ALT_Updated = 0xff; 	   //�߶ȸ������
				Now_doing = SCTemperature;   //������һ��ѭ��
			break;
		
		default: 
			Now_doing = SCTemperature;
			break;
	}
	
	
}
