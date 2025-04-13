/**
  * @file     modeswitch_task.c
  * @version  v1.0
  * @date     
  *
  * @brief    ң��ģʽ�л�����
  * @note     DT7
	*
  *	@author   �޼��� & ������
  *
**/

#include "modeswitch_task.h"
#include <stdio.h>
#include "bsp_remote_ET08.h"

#include "micro_swicth.h"
#include "shoot_task.h"
#include "gimbal_task.h"

ctrl_mode_e ctrl_mode = PROTECT_MODE;
lock_flag_e lock_flag = LOCK;

static void sw1_mode_handler(void);
static void unlock_init(void);

void ModeSwitch_Task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	for(;;)
	{
		//�ڰ˽���
		if(lock_flag == LOCK)
		{
			 unlock_init();
		}
		else if(lock_flag == UNLOCK)
		{                           
			sw1_mode_handler();
		}
	 
		osDelayUntil(&mode_wake_time, MODESWITCH_PERIOD);
	}
}

//��������
static void unlock_init(void)
{
	//SB���˾��У�SC��������
	if(SBUS.SB == SW_MI && SBUS.SC == SW_UP)
	{
		if( SBUS.Ch3 > 600 && SBUS.Ch4 > 600)	//��ң���ڲ�����
		{
			lock_flag = UNLOCK;  //��ҡ������
		}
	}
}

static void sw1_mode_handler(void)  
{
	taskENTER_CRITICAL();
	switch( SBUS.SB )
	{
		case SW_UP:                     //�ϣ�ң��ģʽ
		{
			ctrl_mode = REMOTE_MODE;
			break;
		}
		case SW_MI:                     //�У�����ģʽ
		{
			ctrl_mode = PROTECT_MODE;
			break;
		}
		case SW_DN:                     //�£�����ģʽ
		{
			ctrl_mode = VISION_MODE;
			break;
		}
	}
//				�ȴ������ʼ��											�������ȼ����			
	if ((gimbal_zero == NOFOUND || reload_zero == NOFOUND) && ctrl_mode != PROTECT_MODE )
		ctrl_mode = INIT_MODE;
	
	taskEXIT_CRITICAL();
}

