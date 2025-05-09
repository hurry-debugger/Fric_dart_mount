#include "gimbal_task.h"

#include "stdlib.h"
#include "math.h"

#include "cmsis_os.h"

#include "pid.h"
#include "bsp_remote_ET08.h"
#include "math_calcu.h"

#include "bsp_serial_screen.h"
#include "can_comm.h"
#include "micro_swicth.h"

extern osThreadId can_send_task_handle;

extern uint8_t dart_num;
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
static void Gimbal_Auto_NoFound_Ctrl(void);
static void Gimbal_Auto_Found_Ctrl(void);
		
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
		case GIMBAL_REMOTE_MODE://����Ϊ��
		{
			Gimbal_Remote_Control();
		}	break;
		case GIMBAL_KEYBOARD_MODE:			
			break;//������
		case GIMBAL_VISION_MODE:
		{
			GimbalAutoControl();
		}	break;
		default:
			break;
	}
	Gimbal_Current_Set();
}

/*ÿ�ζ��ᵽ���˶��������һ��dart   
			|	dart0	|	dart1	|	dart2	|	dart3	|stop
			---------------------------------------------------
dart_num��	00			l11111111111222222222222333333333333444
			  Ϊ��װ����Ƶ�
*/

static void Gimbal_Remote_Control(void)
{
	switch(gimbal.screen_gimbal_mode)
	{
		case GIMBAL_SCREEN_TEST_MODE://��Ϊ�ɿ���, Ĭ�ϲ���ģʽ
		{
			taskENTER_CRITICAL();
			serial_gimbal_pos[2] -= (GIMBAL_MAX_SPEED/500.0)*(SBUS.Ch1/670.0);
			OUTPUT_LIMIT(serial_gimbal_pos[2], 0, GIMBAL_MAX_RIGHT);
			gimbal_remote_change_flag = 1;
			taskEXIT_CRITICAL();
			
			gimbal.pos_ref = serial_gimbal_pos[2];
		}	break;
		case GIMBAL_SCREEN_GAME_MODE://ң��ģʽ����ȫ�Զ��������Ӿ�
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

extern uint8_t vision_flag;

static void GimbalAutoControl(void)//���ұ���ֵ��С
{
	if (vision_flag == NOFOUND)
		Gimbal_Auto_NoFound_Ctrl();
	else if (vision_flag == FOUND)
		Gimbal_Auto_Found_Ctrl();		
		
	Gimbal_Pid_Cal();
}

float vision_nofound_pos = AUTO_LEFT_LIMIT;

extern uint8_t one_dart_out_flag;

uint8_t gimbal_auto_ready_flag;//��̨�Ƿ��Ѿ������Ӿ���׼����

extern float vision_found_pos;
uint32_t err_zero_cnt;

float k = 0.2;

float vision_err;

static void Gimbal_Auto_Found_Ctrl(void)
{
	if(one_dart_out_flag)//����һ���ڣ����ñ�־λ
	{
		err_zero_cnt = 0;
		gimbal_auto_ready_flag = 0;
		
		taskENTER_CRITICAL();
		one_dart_out_flag = 0;
		taskEXIT_CRITICAL();
	}	
								//���ز�ֵ������ͷ����Ϊ��
	switch(gimbal.screen_gimbal_mode)
	{
		case GIMBAL_SCREEN_TEST_MODE:
		{
			vision_nofound_pos += -vision_found_pos * k + serial_gimbal_vision_pos[2]*0.01;//ʼ�ո���
			vision_err = -vision_found_pos * k + serial_gimbal_vision_pos[2]*0.01;			
			if(fabs(-vision_found_pos * k + serial_gimbal_vision_pos[2]*0.01) < GIMBAL_VISION_DEADBAND)
			{
				if(err_zero_cnt++ > 100)//��ֹ����
					gimbal_auto_ready_flag = 1;
			}else
			{
				err_zero_cnt = 0;				
			}
		}	break;
		case GIMBAL_SCREEN_GAME_MODE:
		{
			if(dart_num !=4 )//����4���ڵ�ʱ���ۼ���
			{
				vision_nofound_pos += -vision_found_pos * k + serial_gimbal_vision_pos[dart_num]*0.01;//serial_gimbal_vision_posԽ��Խ����
				if(fabs(-vision_found_pos * k + serial_gimbal_vision_pos[dart_num]*0.01) < GIMBAL_VISION_DEADBAND)
				{
					if(err_zero_cnt++ > 100)//��ֹ����
						gimbal_auto_ready_flag = 1;
				}else
				{
					err_zero_cnt = 0;				
				}		
			}else
				vision_nofound_pos = AUTO_LEFT_LIMIT;
		}	break;
		default:
			break;	
	}
	
	gimbal.pos_ref = vision_nofound_pos;
}

dirtion_type_e vision_turn = RIGHT;
uint16_t vision_step = 15;

static void Gimbal_Auto_NoFound_Ctrl(void)
{
	err_zero_cnt = 0;
	gimbal_auto_ready_flag = 0;
	
	if(vision_turn == RIGHT)
	{
		vision_nofound_pos -= vision_step;
		if(vision_nofound_pos < AUTO_RIGHT_LIMIT)
			vision_turn = LEFT;
	}else if(vision_turn == LEFT)
	{
		vision_nofound_pos += vision_step;
		if(vision_nofound_pos > AUTO_LEFT_LIMIT)
			vision_turn = RIGHT;
	}
	
	OUTPUT_LIMIT(vision_nofound_pos, AUTO_LEFT_LIMIT, AUTO_RIGHT_LIMIT);
	
	gimbal.pos_ref = vision_nofound_pos;
}

uint8_t gimbal_init_finish_flag;

static void Gimbal_Found_Zero(void)
{
	if(gimbal_zero == NOFOUND)
	{
		gimbal.spd_ref = 800;
		gimbal.current = PID_Calculate(&gimbal_spd_pid_t, gimbal.spd_fbd, gimbal.spd_ref);
	}
	else if(gimbal_zero == FOUND)
	{
		//�������
		taskENTER_CRITICAL();
		gimbal_motor_data.offset_ecd = gimbal_motor_data.ecd;
		gimbal_motor_data.round_cnt = 0;
		taskEXIT_CRITICAL();
		
		gimbal.current = 0;
		gimbal.start_ecd = gimbal_motor_data.total_ecd;
		
		gimbal_init_finish_flag = 1;//һ��Ҫ����������������˳���ʼ��ģʽ
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
//				{
//					gimbal.gimbal_mode = GIMBAL_PROTECT_MODE;					
//					gimbal.screen_gimbal_mode = GIMBAL_SCREEN_INIT_MODE;
//				}	break;
				case SCREEN_NOINIT:
				case SCREEN_INIT_MODE:
				case SCREEN_GAME_MODE:
				{
					gimbal.gimbal_mode = GIMBAL_REMOTE_MODE;
					gimbal.screen_gimbal_mode = GIMBAL_SCREEN_GAME_MODE;
				}	break;
				default:
					break;
				case SCREEN_TEST_MODE:
				{
					if (SBUS.Ch4 > 600)	//����ģʽ���󲦸����²�������̨ 
					{
						gimbal.gimbal_mode = GIMBAL_REMOTE_MODE;
						gimbal.screen_gimbal_mode = GIMBAL_SCREEN_TEST_MODE;														
					}
					else 
					{
						gimbal.gimbal_mode = GIMBAL_PROTECT_MODE;
					}
				}	break;
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
			switch (screen_mode)
			{
//				{
//					gimbal.gimbal_mode = GIMBAL_PROTECT_MODE;//����ģʽѹ�����ù�gimbal.screen_gimbal_mode				
//					gimbal.screen_gimbal_mode = GIMBAL_SCREEN_INIT_MODE;//��ȫ�����ж�
//				}	break;                            
				case SCREEN_TEST_MODE:                
				{					                  
					gimbal.screen_gimbal_mode = GIMBAL_SCREEN_TEST_MODE;														
				}	break;                            
				case SCREEN_NOINIT:
				case SCREEN_INIT_MODE:
				case SCREEN_GAME_MODE:                
				{                                     
					gimbal.screen_gimbal_mode = GIMBAL_SCREEN_GAME_MODE;
				}	break;
				default:
					break;
			}
			break;
		}
	}
}

