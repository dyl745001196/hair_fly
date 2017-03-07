#ifndef _Motor_H
#define _Motor_H

#include "stm32f10x.h"

void Motor_Out(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4);
void Motor_Init(void);

#endif
