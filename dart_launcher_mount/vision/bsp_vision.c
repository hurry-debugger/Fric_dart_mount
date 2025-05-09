#include "bsp_vision.h"
#include "string.h"

float vision_found_pos;
uint8_t vision_flag;

static uint16_t found_cnt;
static uint16_t nofound_cnt;
void Vision_Usart_Handle(uint8_t *vision_buf)
{
	memcpy(&vision_found_pos, vision_buf, 4);
	if(vision_buf[4]==1)
	{
		if(found_cnt++>15)
		{
			vision_flag = 1;	
			nofound_cnt = 0;
		}
	}else if(vision_buf[4]==0)
	{
		if(nofound_cnt++>15)
		{
			vision_flag = 0;	
			found_cnt = 0;
		}
	}
}
