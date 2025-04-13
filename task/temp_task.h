#ifndef __TEMP_CONTROL_TASK_H__
#define __TEMP_CONTROL_TASK_H__

#include "stm32f4xx_hal.h"

#define TEMP_PERIOD  200

void Temp_Usart_Handle(uint8_t* screen_buf);
void Temp_Task(void const *argu);


#endif
