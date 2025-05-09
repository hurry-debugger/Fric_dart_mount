#ifndef __SHT30_H
#define __SHT30_H

#include "usart.h"

typedef struct __sht30_t
{
	uint8_t buff[3];
	float temp_fdb[3];
	float temp_ref;
}sht30_t;

extern sht30_t sht30;

void Temperture_Analy(uint8_t* string,uint8_t num);
void move_from_fbd_to_buff(void);
uint8_t my_strncmp(uint8_t *string1,uint8_t *string2,size_t count);
uint8_t hex_to_dec(uint8_t hex);

#endif
