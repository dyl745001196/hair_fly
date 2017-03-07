#include "struct_all.h"

struct _acc  acc;			//原始数据
struct _gyro gyro;
////////////////////////////////////////////
struct _acc  filter_acc;	//滤波后数据
struct _gyro filter_gyro;
////////////////////////////////////////////
struct _acc  offset_acc;	//零偏数据
struct _gyro offset_gyro;
////////////////////////////////////////////
struct _SI_float  SI_acc;	//加速度数据（m/s2）
struct _SI_float  SI_gyro;	//角速度数据（rad）