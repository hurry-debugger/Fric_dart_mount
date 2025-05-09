#include "debug_task.h"

#include "cmsis_os.h"

#include "data_scope.h"
#include "pid.h"

#include "can_comm.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "reload_task.h"
#include "dji_motor.h"
#include "tof.h"
#include "dji_motor.h"

extern shoot_t shoot;
extern gimbal_t gimbal;
extern reload_t reload;

extern PID_TypeDef fric_motor_pid_t[2];

extern PID_TypeDef gimbal_spd_pid_t;
extern PID_TypeDef gimbal_pos_pid_t;
extern PID_TypeDef reload_spd_pid_t;

extern uint8_t dart_num;

extern tof_data_t tof_data;
extern uint8_t tof_cnt1;

extern motor_measure_t reload_motor_data;

typedef enum
{
	fric_motor,		//0
	reload_motor,	//1
	gimbal_motor,	//2
	tof,
	can,
}motor_type_e;

motor_type_e motor_type = fric_motor;

extern uint64_t _1ms_cnt;

void DataWavePkg(void)
{
	switch (motor_type)
	{
		case fric_motor:
		{
			DataScope_Get_Channel_Data(-shoot.fric_spd_fbd[L]);
			DataScope_Get_Channel_Data(shoot.fric_spd_fbd[R]);
			
			DataScope_Get_Channel_Data(shoot.fric_spd_ref[L]);
			DataScope_Get_Channel_Data(shoot.fric_spd_ref[R]);
			
			DataScope_Get_Channel_Data(shoot.fric_current[R]);
			DataScope_Get_Channel_Data(shoot.fric_current[L]);
			
			DataScope_Get_Channel_Data(fric_motor_pid_t[R].Pout);
			DataScope_Get_Channel_Data(fric_motor_pid_t[R].Iout);
					
			DataScope_Get_Channel_Data(fric_motor_pid_t[L].Pout);
			DataScope_Get_Channel_Data(fric_motor_pid_t[L].Iout);			
			break;
		} 
		case reload_motor:
		{
//			DataScope_Get_Channel_Data(dart_num);
//			DataScope_Get_Channel_Data(reload.start_ecd);
//			DataScope_Get_Channel_Data(reload.now_ecd);
			
//			DataScope_Get_Channel_Data(4 * ONE_DART_ECD);
			
//			DataScope_Get_Channel_Data(reload_motor_data.offset_ecd);		
			DataScope_Get_Channel_Data(reload_motor_data.total_ecd);
			
			DataScope_Get_Channel_Data(reload_motor_data.msg_cnt);
			DataScope_Get_Channel_Data(reload_motor_data.ecd);
			DataScope_Get_Channel_Data(reload_motor_data.round_cnt);
//			DataScope_Get_Channel_Data(reload_motor_data.round_cnt);
//			DataScope_Get_Channel_Data(reload_motor_data.last_ecd);
//			DataScope_Get_Channel_Data(_1ms_cnt);
//			DataScope_Get_Channel_Data(reload_motor_data.given_current);
			break;
		}
		case gimbal_motor:
		{
			DataScope_Get_Channel_Data(gimbal.pos_fbd);
			DataScope_Get_Channel_Data(gimbal.pos_ref);
			DataScope_Get_Channel_Data(gimbal.spd_fbd);
			DataScope_Get_Channel_Data(gimbal.spd_ref);	
			
			DataScope_Get_Channel_Data(gimbal.current);
			
			DataScope_Get_Channel_Data(gimbal_spd_pid_t.Iout);
			DataScope_Get_Channel_Data(gimbal_spd_pid_t.Pout);
			DataScope_Get_Channel_Data(gimbal_pos_pid_t.Iout);
			DataScope_Get_Channel_Data(gimbal_pos_pid_t.Pout);
			break;
		}
		case tof:
		{
			DataScope_Get_Channel_Data(tof_data.dist);
			DataScope_Get_Channel_Data(tof_data.amp);
			DataScope_Get_Channel_Data(tof_data.temp);
			DataScope_Get_Channel_Data(tof_data.check_sum);
						
		}	break;
		case can:
		{
			DataScope_Get_Channel_Data(reload_motor_data.msg_cnt);
			DataScope_Get_Channel_Data(gimbal_motor_data.msg_cnt);
			DataScope_Get_Channel_Data(_1ms_cnt);
						
		}	break;
		default:
			break;
	}
}


void Debug_Task(void const* argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	while(1)
	{
		taskENTER_CRITICAL();
		
		DataWave(&DEBUG_HUART);
		
		taskEXIT_CRITICAL();
		osDelayUntil(&mode_wake_time, DEBUG_PERIOD);
	}
}

