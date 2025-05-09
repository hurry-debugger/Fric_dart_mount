#ifndef __TEMP_H__
#define __TEMP_H__

#include "stm32f4xx_hal.h"

#define TEMP_PERIOD 500

#define TIM2_CCR3_MAX  7500

void Temp_Task(void const* argu);
void Temp_Usart_Handle(uint8_t *temp_buf);

#endif
