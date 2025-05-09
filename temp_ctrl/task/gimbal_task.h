#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "modeswitch_task.h"
#include "math_calcu.h"
#include "bsp_serial_screen.h"

#define GIMBAL_PERIOD 		2
#define MG6020_MAX_CURRENT 	30000
#define MG6020_MAX_ECD 		8191	
#define MG6020_MAX_SPEED 	350	

#define GIMBAL_MAX_RIGHT   -213000

#define VISION_SPEED_LIMIT  300

#define GIMBAL_MAX_SPEED    8192*8  

#define AUTO_LEFT_LIMIT -203000

typedef enum{
	GIMBAL_PROTECT_MODE,
	GIMBAL_INIT_MODE,
	GIMBAL_REMOTE_MODE,
	GIMBAL_KEYBOARD_MODE,
	GIMBAL_VISION_MODE,
}gimbal_mode_e;

typedef enum{
	GIMBAL_REMOTE_TEST_MODE,
	GIMBAL_REMOTE_GAME_MODE,
}screen_gimbal_mode_e;

typedef struct {
	float spd_fbd;
	float spd_ref;
	
	float pos_fbd;
	float pos_ref;
	
	float start_ecd;
	
	int16_t current;
	
	gimbal_mode_e gimbal_mode;
	
	screen_gimbal_mode_e screen_gimbal_mode;
}gimbal_t;

/*********************fsm******************/
typedef enum{
    GIMBAL_LOCK = -1,
	GIMBAL_INIT = 3,
    GIMBAL_PROTECT = 0,
    GIMBAL_REMOTE = 1,
    GIMBAL_AUTO = 2,
} gimbal_status_e;

typedef enum{
    GIMBAL_NULL_EVENT = 0,
    GIMBAL_UNLOCK_OPERATION = 1,
	GIMBAL_INIT_OPERATION = 5,
    GIMBAL_PROTECT_OPERATION = 2,
    GIMBAL_REMOTE_OPERATION = 3,
    GIMBAL_AUTO_OPERATION = 4,
} gimbal_event_e;

typedef struct{
    gimbal_status_e status;
    gimbal_event_e event;
    void (*gimbal_status_handler)(void);
} gimbal_fsm_t;
/******************************************/


extern gimbal_t gimbal;
void Gimbal_Task(void const* argu);

#endif
