#include "reload_task.h"

#include <stdlib.h>
#include "math.h"

#include "cmsis_os.h"

#include "pid.h"
#include "bsp_remote_ET08.h"
#include "bsp_serial_screen.h"

#include "can_comm.h"
#include "micro_swicth.h"
#include "tof.h"

#include "usart.h"
extern osThreadId can_send_task_handle;

uint8_t dart_num;
uint8_t last_dart_num;
uint8_t one_dart_out_flag;
uint8_t fric_run_flag;

reload_t reload;

reload_fsm_t reload_fsm;

PID_TypeDef reload_spd_pid_t;//速度环

PID_TypeDef reload_spd_Cpid_t;//位置串级环
PID_TypeDef reload_pos_Cpid_t;

uint8_t rel_one_flag;
uint8_t rel_all_flag;

static void Reload_Motor_Init(void);
static void Reload_Control(void);
static void Reload_Data_Update(void);
static void Reload_Current_Set(void);

static void Relaod_Found_Zero(void);
static void Reload_Reinit(void);
static void Reload_Remote_Single(void);
static void Reload_Remote_Serial(void);
static void Reload_Vision_Single(void);
static void Reload_Vision_Serial(void);

static void reload_mode_sw(reload_fsm_t *reload_fsm); 
static void reload_status_transition(reload_fsm_t *reload_fsm);
			
static void Reload_Status_Init(void);
static void reload_protect_handler(void);
static void reload_remote_handler(void);
static void reload_auto_handler(void);
static void reload_init_handler(void);

static void Reload_Mode_Change(reload_mode_e reload_mode);

void Reload_Task(void const* argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	Reload_Motor_Init();
	while (1)
	{
//		HAL_UART_Transmit(&huart6, (uint8_t*)"ddd", 3, 0xff);
		reload_mode_sw(&reload_fsm);
		Reload_Control();
		osSignalSet( can_send_task_handle, RELAOD_MOTOR_MSG_SEND);
		osDelayUntil(&mode_wake_time, RELOAD_PERIOD);
	}
}

static void Reload_Data_Update(void)
{
	reload.now_ecd = reload_motor_data.total_ecd - reload.start_ecd;
	reload.spd_fbd = reload_motor_data.speed_rpm;
}

ramp_calu_t reload_ramp;

static void Reload_Pid_Cal(void)
{
	Ramp_Calu(&reload_ramp, reload.spd_ref, reload_ramp.present_ref, 200);
	reload.current = PID_Calculate(&reload_spd_pid_t, reload.spd_fbd, reload_ramp.present_ref);
}

/*每次都会到达运动方向的下一个dart   
			|	dart0	|	dart1	|	dart2	|	dart3	|stop
			---------------------------------------------------
dart_num：	00			l11111111111222222222222333333333333444
			  为重装填设计的
*/
static void Dart_Num_Update(void)
{
	last_dart_num = dart_num;
	
		if(reload.now_ecd > 4*ONE_DART_ECD)
			dart_num = 4;
	else if(reload.now_ecd > 3*ONE_DART_ECD)
			dart_num = 3;
	else if(reload.now_ecd > 2*ONE_DART_ECD)
			dart_num = 2;
	else if(reload.now_ecd > 1*ONE_DART_ECD)
			dart_num = 1;
	else if(reload.now_ecd < 50000)
			dart_num = 0;
	else if(reload.reload_mode == RELOAD_REINIT_MODE)
		dart_num = 1;
	else 
		dart_num = 0;
	
	if(last_dart_num != dart_num && last_dart_num < dart_num && dart_num != 4)//正向变化  大于4就不用变化了
		one_dart_out_flag = 1;//一定要云台接收到他的变化再变
}

static void Reload_Control(void)
{
	Reload_Data_Update();
	Dart_Num_Update();
	switch (reload.reload_mode)
	{
//保护
		case RELOAD_PROTECT_MODE:
		{
			reload.current = 0;			
		}	break;
//上电初始化	
		case RELOAD_INIT_MODE:
		{
			Relaod_Found_Zero();
		}	break;
//遥控模式单发
		case RELOAD_REMOTE_SINGLE_MODE:
		{
			Reload_Remote_Single();
		}	break;
//遥控模式连发
		case RELOAD_REMOTE_SERIES_MODE:
		{
			Reload_Remote_Serial();
		}	break;
//视觉模式单发
		case RELOAD_VISION_SINGLE_MODE:
		{
			Reload_Vision_Single();
		}	break;
//视觉模式连发
		case RELOAD_VISION_SERIES_MODE:
		{
			Reload_Vision_Serial();
		}	break;
//重装填
		case RELOAD_REINIT_MODE:
		{
			Reload_Reinit();
		}	break;
		
		default:
			break;
	}
	
	Reload_Current_Set();
}

uint16_t reload_tick_cnt = 0;
uint8_t reload_lanuch_flag, reload_tick_start;

static void Reload_Remote_Single(void)
{
	if (dart_num<=2){
		fric_run_flag = 0;
		reload.spd_ref = RELOAD_SPEED;//摩擦轮不动，装填电机动
	}else if (dart_num==3){
		reload_tick_start = 0;
		fric_run_flag = 1;
		reload.spd_ref = RELOAD_SPEED;//摩擦轮动，装填电机动
	}else if(dart_num == 4)
	{
		reload.spd_ref = 0;//摩擦轮再动两秒，装填电机不动
		if(fric_run_flag != 0)
			reload_tick_start = 1;
		if(reload_lanuch_flag)
		{
			fric_run_flag = 0;
			reload_lanuch_flag = 0;
		}
	}
	Reload_Pid_Cal();
}

extern uint8_t gate_open_cnt;//打开闸门次数

static void Reload_Remote_Serial(void)
{
	switch (dart_num)
	{
		case 0:
		{
			if(gate_open_cnt == 1)	//闸门第一次开启
			{
				reload_tick_start = 1;	//开启定时器计时
				fric_run_flag = 1;		//先开启摩擦轮达到转速
				if(reload_lanuch_flag)	//等待定时器计时两秒置标志位
				{
					reload.spd_ref = RELOAD_SPEED;
				}else
				{
					reload.spd_ref = 0;
				}
			}else						//闸门关闭
			{
				reload_tick_start = 0;
				fric_run_flag = 0;
				reload.spd_ref = 0;
			}	
		}	break;
		case 1:
		{
			reload_lanuch_flag = 0;			//重置定时器置的标志位
			fric_run_flag = 1;				//直接推上去,感觉这两句话可要可不要
			reload.spd_ref = RELOAD_SPEED;
		}	break;
		case 2://每次开闸门的第一发镖
		{
			if(gate_open_cnt == 2)	//闸门第二次开启
			{
				reload_tick_start = 1;	//开启定时器计时
				fric_run_flag = 1;		//先开启摩擦轮达到转速
				if(reload_lanuch_flag)	//等待定时器计时两秒置标志位
				{
					reload.spd_ref = RELOAD_SPEED;//摩擦轮加好速度再推
				}else
				{
					reload.spd_ref = 0;
				}
			}else						//闸门关闭
			{
				reload_tick_start = 0;
				fric_run_flag = 0;
				reload.spd_ref = 0;
			}	
		}	break;
		case 3://每次开闸门的第二发镖
		{
			reload_tick_start = 0;
			reload_lanuch_flag = 0;			//重置定时器置的标志位
			fric_run_flag = 1;
			reload.spd_ref = RELOAD_SPEED;
		}	break;
		case 4:
		{
			reload.spd_ref = 0;//摩擦轮再动两秒，装填电机不动
			if(fric_run_flag!=0)
				reload_tick_start = 1;
			if(reload_lanuch_flag)
			{
				fric_run_flag = 0;
				reload_lanuch_flag = 0;
			}
		}	break;
		default:
			break;
	}
	Reload_Pid_Cal();
}

void Reload_Tick(void)//放入1ms定时器中
{
	if (reload_tick_start)
	{
		if (reload_tick_cnt ++ >= 2500)//每隔两秒发一镖
		{
			reload_lanuch_flag = 1;
			reload_tick_cnt = 0;
			reload_tick_start = 0;
		}
	}else
	{
		reload_tick_cnt = 0;
	}
}

extern uint8_t gimbal_auto_ready_flag;//云台是否已经根据视觉瞄准好了

static void Reload_Vision_Single(void)
{
	if (dart_num<=2){
		fric_run_flag = 0;
		reload.spd_ref = RELOAD_SPEED;//摩擦轮不动，装填电机动
	}else if (dart_num==3 && gimbal_auto_ready_flag){//云台已经移动到视觉指定位置 && 到单发位置
		reload_tick_start = 0;
		fric_run_flag = 1;
		reload.spd_ref = RELOAD_SPEED;//摩擦轮动，装填电机动
	}else
	{
		reload.spd_ref = 0;//摩擦轮再动两秒，装填电机不动
		if(fric_run_flag != 0)
			reload_tick_start = 1;
		if(reload_lanuch_flag)
		{
			fric_run_flag = 0;
			reload_lanuch_flag = 0;
		}
	}
	Reload_Pid_Cal();
}

static void Reload_Vision_Serial(void)
{
	switch (dart_num)
	{
		case 0:
		{
			if(gate_open_cnt== 1)	//不管视觉有没有瞄到了&&裁判系统开始比赛测距已检测到闸门打开
			{
				reload_tick_start = 1;	//开启定时器计时
				fric_run_flag = 1;		//先开启摩擦轮达到转速
				if(reload_lanuch_flag)	//等待定时器计时两秒置标志位
				{
					reload.spd_ref = RELOAD_SPEED;
				}else
				{
					reload.spd_ref = 0;
				}
			}else						
			{
				reload_tick_start = 0;	//开启定时器计时
				fric_run_flag = 0;
				reload.spd_ref = 0;
			}	
		}	break;
		case 1:
		{
			reload_lanuch_flag = 0;			//重置定时器置的标志位
			fric_run_flag = 1;				//直接推上去,感觉这两句话可要可不要
			reload.spd_ref = RELOAD_SPEED;
		}	break;
		case 2://每次开闸门的第一发镖
		{
			if(gate_open_cnt== 2)	//不管视觉有没有瞄到
			{
				reload_tick_start = 1;	//开启定时器计时
				fric_run_flag = 1;		//先开启摩擦轮达到转速
				if(reload_lanuch_flag)	//等待定时器计时两秒置标志位
				{
					reload.spd_ref = RELOAD_SPEED;//摩擦轮加好速度再推
				}else
				{
					reload.spd_ref = 0;
				}
			}else						
			{
				fric_run_flag = 0;
				reload.spd_ref = 0;
			}	
		}	break;
		case 3://每次开闸门的第二发镖
		{
			reload_lanuch_flag = 0;			//重置定时器置的标志位
			fric_run_flag = 1;
			reload.spd_ref = RELOAD_SPEED;
		}	break;
		case 4:
		{
			reload.spd_ref = 0;//摩擦轮再动两秒，装填电机不动
			if(fric_run_flag != 0)
				reload_tick_start = 1;
			if(reload_lanuch_flag)
			{
				fric_run_flag = 0;
				reload_lanuch_flag = 0;
			}
		}	break;
		default:
			break;
	}
	Reload_Pid_Cal();
}

static void Reload_Reinit(void)
{
	reload_tick_start = 0;
	reload_tick_cnt = 0;
	fric_run_flag = 0;
	switch (SBUS.SD)
	{
		case SW_UP:{//重装填一发
			reload.spd_ref = -6000;
			if(dart_num < 3)//小于第三发镖的位置即为第二发镖的位置
				reload.spd_ref = 0;
		}	break;
		case SW_DN:{//重装填四发
			reload.spd_ref = -6000;
			if(dart_num == 0)
				reload.spd_ref = 0;
		}	break;
		default:
			break;
	}
	Reload_Pid_Cal();
}

static void Reload_Current_Set(void)
{
	motor_current.reload = reload.current;
}

uint8_t reload_init_finish_flag;
static void Relaod_Found_Zero(void)
{
	if(reload_zero == NOFOUND)
	{
		reload.spd_ref = RELOAD_INIT_SPEED;
		Reload_Pid_Cal();
	}
	else if(reload_zero == FOUND)
	{
//		taskENTER_CRITICAL();
//		reload_motor_data.offset_ecd = reload_motor_data.ecd;
//		reload_motor_data.round_cnt = 0;
//		taskEXIT_CRITICAL();
		
		reload.spd_ref = 0;
		Reload_Pid_Cal();
		
		if(abs(reload_motor_data.speed_rpm) < 100)
		{
			reload.current = 0;
			
			reload.start_ecd = reload_motor_data.total_ecd;
			reload_init_finish_flag = 1;//一定要等我设置完零点,ctrl_moed再退出init_mode			
		}
	}
}

static void Reload_Motor_Init(void)
{
	Reload_Status_Init();
	PID_Init(&reload_spd_pid_t, MG2006_MAX_CURRENT, 0, 0, \
				3, 0.319999993, 1.79999995, \
				500, 1000, \
				0, 0, ChangingIntegralRate | Derivative_On_Measurement);	
////位置环
//	PID_Init(&reload_pos_Cpid_t, MG2006_MAX_ECD/2, 0, 100, \
//				0.0649, 0.0008, 0, \
//				3000, 1000, \
//				0, 0, ChangingIntegralRate);	
//	PID_Init(&reload_spd_Cpid_t, MG2006_MAX_CURRENT, 0, 0, \
//				2.7, 0, 0, \
//				100, 300, \
//				0, 0, ChangingIntegralRate);	

//速度环	
}

static void reload_mode_sw(reload_fsm_t *fsm) //SC上保护中单发下连发
{
	fsm->reload_status_handler();
	reload_status_transition(fsm);
    /* 模式切换 */
    switch (fsm->status)
	{
		case RELOAD_LOCK:
        case RELOAD_PROTECT: 
		{
			Reload_Mode_Change(RELOAD_PROTECT_MODE);
		}	break;
		case RELOAD_INIT:
		{
			Reload_Mode_Change(RELOAD_INIT_MODE);			
		}	break;
        case RELOAD_REMOTE:
		{
			switch(SBUS.SC)
			{
				case SW_UP:
					Reload_Mode_Change(RELOAD_PROTECT_MODE);
					break;
				case SW_MI:
				{
					Reload_Mode_Change(RELOAD_REMOTE_SINGLE_MODE);
					if(SBUS.SA == SW_UP)
						Reload_Mode_Change(RELOAD_REINIT_MODE);				
				}	break;
				case SW_DN:
				{
					Reload_Mode_Change(RELOAD_REMOTE_SERIES_MODE);
					if(SBUS.SA == SW_UP)
						Reload_Mode_Change(RELOAD_REINIT_MODE);				
				}	break;
				default:
					break;
			}
		}	break;
        case RELOAD_AUTO: //视觉自动
		{
			switch(SBUS.SC)
			{
				case SW_UP:
					Reload_Mode_Change(RELOAD_PROTECT_MODE);
					break;
				case SW_MI:
				{
					Reload_Mode_Change(RELOAD_VISION_SINGLE_MODE);
					if(SBUS.SA == SW_UP)
						Reload_Mode_Change(RELOAD_REINIT_MODE);				
				}	break;
				case SW_DN:
				{
					Reload_Mode_Change(RELOAD_VISION_SERIES_MODE);
					if(SBUS.SA == SW_UP)
						Reload_Mode_Change(RELOAD_REINIT_MODE);				
				}	break;
				default:
					break;
			}
		}	break;		
		default:
			break;	
	}
}	

static void reload_status_transition(reload_fsm_t *fsm)
{
	switch(fsm->status)
	{
		case RELOAD_LOCK:
			if(fsm->event == RELOAD_UNLOCK_OPERATION){
				fsm->status = RELOAD_PROTECT;
				fsm->reload_status_handler = reload_protect_handler;
			}
			break;
		case RELOAD_PROTECT:
			if (fsm->event == RELOAD_INIT_OPERATION){
				fsm->status = RELOAD_INIT;
				fsm->reload_status_handler = reload_init_handler;
			}else if(fsm->event == RELOAD_REMOTE_OPERATION){
				fsm->status = RELOAD_REMOTE;
				fsm->reload_status_handler = reload_remote_handler;
			}else if(fsm->event == RELOAD_AUTO_OPERATION){
				fsm->status = RELOAD_AUTO;
				fsm->reload_status_handler = reload_auto_handler;
			}	break;
		case RELOAD_INIT:
			if (fsm->event == RELOAD_PROTECT_OPERATION){
				fsm->status = RELOAD_PROTECT;
				fsm->reload_status_handler = reload_protect_handler;
			}else if(fsm->event == RELOAD_REMOTE_OPERATION){
				fsm->status = RELOAD_REMOTE;
				fsm->reload_status_handler = reload_remote_handler;
			}else if(fsm->event == RELOAD_AUTO_OPERATION){
				fsm->status = RELOAD_AUTO;
				fsm->reload_status_handler = reload_auto_handler;
			}	break;
		case RELOAD_REMOTE:
			if(fsm->event == RELOAD_PROTECT_OPERATION){
				fsm->status = RELOAD_PROTECT;
				fsm->reload_status_handler = reload_protect_handler;
			}else if(fsm->event == RELOAD_AUTO_OPERATION){
				fsm->status = RELOAD_AUTO;
				fsm->reload_status_handler = reload_auto_handler;
			}
			break;
		case RELOAD_AUTO:
			if(fsm->event == RELOAD_PROTECT_OPERATION){
				fsm->status = RELOAD_PROTECT;
				fsm->reload_status_handler = reload_protect_handler;
			}else if(fsm->event == RELOAD_REMOTE_OPERATION){
				fsm->status = RELOAD_REMOTE;
				fsm->reload_status_handler = reload_remote_handler;
			}
			break;
		default:
			break;
	}
	fsm->event = RELOAD_NULL_EVENT;
}

static void reload_lock_handler(void)
{
	if(lock_flag == UNLOCK)
		reload_fsm.event = RELOAD_UNLOCK_OPERATION;
}

static void reload_init_handler(void)
{
	if(ctrl_mode == PROTECT_MODE)
		reload_fsm.event = RELOAD_PROTECT_OPERATION;
	
	else if(ctrl_mode == REMOTE_MODE)
		reload_fsm.event = RELOAD_REMOTE_OPERATION;

	else if (ctrl_mode == VISION_MODE)
		reload_fsm.event = RELOAD_AUTO_OPERATION;
}

static void reload_protect_handler(void)
{
	if (ctrl_mode == INIT_MODE)
		reload_fsm.event = RELOAD_INIT_OPERATION;
	
	else if(ctrl_mode == REMOTE_MODE)
		reload_fsm.event = RELOAD_REMOTE_OPERATION;
	
	else if (ctrl_mode == VISION_MODE)
		reload_fsm.event = RELOAD_AUTO_OPERATION;
}

static void reload_remote_handler(void)
{
	if(ctrl_mode == PROTECT_MODE)
		reload_fsm.event = RELOAD_PROTECT_OPERATION;
	
	else if(ctrl_mode == VISION_MODE)
		reload_fsm.event = RELOAD_AUTO_OPERATION;
}

static void reload_auto_handler(void)
{	
	if(ctrl_mode == PROTECT_MODE)
		reload_fsm.event = RELOAD_PROTECT_OPERATION;

	else if(ctrl_mode == REMOTE_MODE||ctrl_mode == KEYBOARD_MODE)
		reload_fsm.event = RELOAD_REMOTE_OPERATION;
}

static void Reload_Mode_Change(reload_mode_e relaod_mode)
{	
	reload.reload_mode = relaod_mode;
}

static void Reload_Status_Init(void)
{
	reload_fsm.status = RELOAD_LOCK;
	reload_fsm.event = RELOAD_NULL_EVENT;
	reload_fsm.reload_status_handler = reload_lock_handler;
}


float Ramp_Calu(ramp_calu_t* ramp, float target_val, float present_ref, float step_val)
{
	ramp->target_val  = target_val;
	ramp->present_ref = present_ref;
	ramp->step_val    = step_val>0?step_val:-step_val;
	
	if(fabs(ramp->present_ref - ramp->target_val) < step_val)
	{
		ramp->present_ref = ramp->present_ref;
	}else if(ramp->present_ref < ramp->target_val)
	{
		ramp->present_ref += ramp->step_val;
	}else if(ramp->present_ref > ramp->target_val)
	{
		ramp->present_ref -= ramp->step_val;
	}
	
	return ramp->present_ref;
}
