#ifndef __DEBUG_TASK_H__
#define __DEBUG_TASK_H__

#include "stm32f4xx_hal.h"
#include "bsp_usart.h"

#define DEBUG_PERIOD 1

void Debug_Task(void const* argu);

#endif
