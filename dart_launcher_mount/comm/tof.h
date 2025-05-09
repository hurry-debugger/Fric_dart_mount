#ifndef __TOF_H__
#define __TOF_H__

#include "stm32f4xx_hal.h"

typedef enum{
	CLOSE,
	OPEN,
}gate_state_e;

__packed typedef struct{
	uint16_t dist;
	uint16_t amp;
	uint16_t temp;
	uint8_t check_sum;
}tof_data_t;

extern gate_state_e tof_gate_state;

void TOF_Tick(void);
void TOF_Usart_Handle(uint8_t* screen_buf);


#endif
