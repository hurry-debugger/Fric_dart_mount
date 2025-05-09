#include "tof.h"

#include "string.h"

#include "modeswitch_task.h"
#include "bsp_serial_screen.h"
#include "bsp_remote_ET08.h"
#include "bsp_judge.h"

extern uint8_t dart_num;

uint8_t gate_open_cnt = 0;//闸门打开次数。reload_task.h使用

tof_data_t tof_data;
gate_state_e tof_gate_state = OPEN;
uint8_t tof_close_time_cnt;
uint8_t tof_open_time_cnt;

void TOF_Usart_Handle(uint8_t* tof_buf)//10ms进一次
{
	if(tof_buf[0] == 0x59 && tof_buf[1] == 0x59)//֡
	{
		memcpy(&tof_data, tof_buf+2, 7);
		if(tof_data.dist < 100 && tof_data.dist!=0)			//假设闸门就离测距仪小于100cm
			{//相当于只捕获跳变沿
			if(tof_gate_state != CLOSE && tof_close_time_cnt++ > 20)	//连续20帧都为小于60cm
			{							
				tof_gate_state = CLOSE;
				tof_close_time_cnt  = 0;
			}
		}else 
		{
			tof_close_time_cnt = 0;
		}
		
		if(tof_data.dist==0 || tof_data.dist>300)//大于九米时就等于900
		{
			if(tof_gate_state!=OPEN && tof_open_time_cnt++>20)//连续20帧都为小于60cm
			{
				tof_gate_state = OPEN;
				tof_open_time_cnt = 0;
//	Game_State.game_progress		//0未开始比赛，1准备阶段，2自检阶段，3五秒倒计时，4对战中，5比赛结算中    无裁判系统
				if((Game_State.game_progress==4 && ctrl_mode != INIT_MODE && ctrl_mode != PROTECT_MODE)||((SBUS.SD == SW_UP|| gate_open_cnt!=0) && ctrl_mode != INIT_MODE && ctrl_mode != PROTECT_MODE))
				{
					if(dart_num == 0)
						gate_open_cnt = 1;
					if(dart_num == 2)
						gate_open_cnt = 2;				
				}
			}
		}else
		{
			tof_open_time_cnt = 0;
		}

		if(dart_num == 4)
			gate_open_cnt = 0;
	}
}

void TOF_Tick(void)
{
	
}
