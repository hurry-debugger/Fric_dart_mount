#ifndef _MODESWITCH_TASK_H__
#define _MODESWITCH_TASK_H__

/* �弶֧���ļ� */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"

#define MODESWITCH_PERIOD 10

typedef enum{
	LOCK = 0,
	UNLOCK
}lock_flag_e;

typedef enum
{
    PROTECT_MODE = 0,   //����ģʽ
	INIT_MODE,			//��ʼ��ģʽ
    REMOTE_MODE,   		//ң��ģʽ
    KEYBOARD_MODE,  	//����ģʽ
    VISION_MODE     	//�Ӿ�ģʽ(����Ҽ�����)
} ctrl_mode_e;

extern lock_flag_e lock_flag;
extern ctrl_mode_e ctrl_mode;

void ModeSwitch_Task(void const *argu);


#endif 
