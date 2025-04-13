#include "gimbal_task.h"

#include "stdlib.h"
#include "math.h"

#include "cmsis_os.h"

#include "pid.h"
#include "bsp_remote_ET08.h"
#include "math_calcu.h"

#include "can_comm.h"
#include "micro_swicth.h"

extern osThreadId can_send_task_handle;

extern uint8_t dart_num;
extern uint8_t gimbal_remote_change_flag;//外部使用

gimbal_t gimbal;

PID_TypeDef gimbal_spd_pid_t;
PID_TypeDef gimbal_pos_pid_t;

static void Gimbal_Motor_Init(void);
static void Gimbal_Control(void);
static void Gimbal_Data_Update(void);
static void Gimbal_Current_Set(void);

static void Gimbal_Found_Zero(void);
static void Gimbal_Remote_Control(void);
static void GimbalAutoControl(void);
			
static void Gimbal_Mode_Change(ctrl_mode_e ctrl_mode);

void Gimbal_Task(void const* argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	Gimbal_Motor_Init();
	while (1)
	{
		Gimbal_Mode_Change(ctrl_mode);
		Gimbal_Control();
		osSignalSet(can_send_task_handle, GIMBAL_MOTOR_MSG_SEND);
		osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
	}
}

static void Gimbal_Data_Update(void)
{
	gimbal.spd_fbd = gimbal_motor_data.speed_rpm;
	gimbal.pos_fbd = gimbal_motor_data.total_ecd;
}

static void Gimbal_Pid_Cal(void)
{
	OUTPUT_LIMIT(gimbal.pos_ref, 0, GIMBAL_MAX_RIGHT);
	
	gimbal.spd_ref = PID_Calculate(&gimbal_pos_pid_t, gimbal.pos_fbd, gimbal.pos_ref);	
	
	gimbal.current = PID_Calculate(&gimbal_spd_pid_t, gimbal.spd_fbd, gimbal.spd_ref);
}

static void Gimbal_Control(void)
{
	Gimbal_Data_Update();
	
	switch (gimbal.gimbal_mode)
	{
		case GIMBAL_INIT_MODE:
		{
			Gimbal_Found_Zero();		
		}	break;
		case GIMBAL_PROTECT_MODE:
		{
			gimbal.current = 0;
		}	break;
		case GIMBAL_REMOTE_MODE://向右为负
		{
			Gimbal_Remote_Control();
		}	break;
		case GIMBAL_KEYBOARD_MODE:			
			break;//待完善
		case GIMBAL_VISION_MODE:
		{
			GimbalAutoControl();
		}	break;
		default:
			break;
	}
	Gimbal_Current_Set();
}

/*每次都会到达运动方向的下一个dart   
			|	dart0	|	dart1	|	dart2	|	dart3	|stop
			---------------------------------------------------
dart_num：	00			l11111111111222222222222333333333333444
			  为重装填设计的
*/

static void Gimbal_Remote_Control(void)
{
	switch(gimbal.screen_gimbal_mode)
	{
		case GIMBAL_REMOTE_TEST_MODE://人为可控制
		{
			taskENTER_CRITICAL();
			serial_gimbal_pos[2] -= (GIMBAL_MAX_SPEED/500.0)*(SBUS.Ch1/670.0);
			OUTPUT_LIMIT(serial_gimbal_pos[2], 0, GIMBAL_MAX_RIGHT);
			gimbal_remote_change_flag = 1;
			taskEXIT_CRITICAL();
			
			gimbal.pos_ref = serial_gimbal_pos[2];
		}	break;
		case GIMBAL_REMOTE_GAME_MODE://遥控模式下完全自动，不用视觉
		{
			gimbal.pos_ref = serial_gimbal_pos[dart_num];
			if(dart_num >= 4)
				gimbal.pos_ref = serial_gimbal_pos[3];			
		}	break;
		default:
			break;
	}

	Gimbal_Pid_Cal();
}

typedef enum{
	RIGHT,
	LEFT,
}dirtion_type_e;


uint8_t vision_flag;
dirtion_type_e vision_turn = RIGHT;

float vision_found_pos;
float vision_nofound_pos = AUTO_LEFT_LIMIT;

uint16_t vision_step = 660;

static void GimbalAutoControl(void)//向右编码值减小
{
	if (vision_flag == NOFOUND)
	{
		if(vision_turn == RIGHT)
		{
			vision_nofound_pos -= vision_step;
			if(gimbal.pos_fbd < GIMBAL_MAX_RIGHT + 100)//+100别撞限位了
				vision_turn = LEFT;
		}else if(vision_turn == LEFT)
		{
			gimbal.pos_ref += vision_step;
			if(gimbal.pos_fbd >AUTO_LEFT_LIMIT)
				vision_turn = RIGHT;
		}
		
		gimbal.pos_ref = vision_nofound_pos;
	}else if (vision_flag == FOUND)
	{
		gimbal.pos_ref = gimbal.pos_fbd + vision_found_pos;
	}
		
	Gimbal_Pid_Cal();
}

static void Gimbal_Found_Zero(void)
{
	if(gimbal_zero == NOFOUND)
	{
		gimbal.spd_ref = 800;
		gimbal.current = PID_Calculate(&gimbal_spd_pid_t, gimbal.spd_fbd, gimbal.spd_ref);
	}
	else if(gimbal_zero == FOUND)
	{
		//设置零点
		taskENTER_CRITICAL();
		gimbal_motor_data.offset_ecd = gimbal_motor_data.ecd;
		gimbal_motor_data.round_cnt = 0;
		taskEXIT_CRITICAL();
		
		gimbal.current = 0;
		gimbal.start_ecd = gimbal_motor_data.offset_ecd;
	}
}

static void Gimbal_Current_Set(void)
{
	motor_current.gimbal = gimbal.current;
}

static void Gimbal_Motor_Init(void)
{	
	gimbal.gimbal_mode = GIMBAL_PROTECT_MODE;
	PID_Init(&gimbal_pos_pid_t, MG6020_MAX_ECD/2, 20, 1, \
				1.04999995, 0.0199999996, 0, \
				100, 20, \
				0, 0, ChangingIntegralRate);
	PID_Init(&gimbal_spd_pid_t, MG6020_MAX_CURRENT, 0, 0, \
				40.2, 0, 0, \
				0, 0, \
				0, 0, Integral_Limit);
}

static void Gimbal_Mode_Change(ctrl_mode_e gimbal_ctrl_mode)
{
	switch (gimbal_ctrl_mode)
	{
		case INIT_MODE:
		{
			gimbal.gimbal_mode = GIMBAL_INIT_MODE;
			break;
		}
		case PROTECT_MODE:
		{
			gimbal.gimbal_mode = GIMBAL_PROTECT_MODE;
			break;
		}
		case REMOTE_MODE:
		{
			switch (screen_mode)
			{
				case SCREEN_NOINIT:
				case SCREEN_INIT_MODE:
				case SCREEN_TEST_MODE:
				{
					if (SBUS.Ch4 > 600)	//测试模式下左拨杆向下拨解锁云台 
					{
						gimbal.gimbal_mode = GIMBAL_REMOTE_MODE;
						gimbal.screen_gimbal_mode = GIMBAL_REMOTE_TEST_MODE;														
					}
					else 
					{
						gimbal.gimbal_mode = GIMBAL_PROTECT_MODE;
					}
				}	break;
				case SCREEN_GAME_MODE:
				{
					gimbal.gimbal_mode = GIMBAL_REMOTE_MODE;
					gimbal.screen_gimbal_mode = GIMBAL_REMOTE_GAME_MODE;
				}	break;
				default:
					break;
			}
		}	break;
		case KEYBOARD_MODE:
		{
			gimbal.gimbal_mode = GIMBAL_KEYBOARD_MODE;
			break;
		}
		case VISION_MODE:
		{
			gimbal.gimbal_mode = GIMBAL_VISION_MODE;
			break;
		}
	}
}

