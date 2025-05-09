#ifndef _SERIAL_SCREEN_TASK_H__
#define _SERIAL_SCREEN_TASK_H__

/* 板级支持文件 */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"


#define FRIC_ADDR     ((uint32_t)0x08060000) 	//摩擦轮VEL存放起始地址 	8byte
#define GIMBAL_ADDR   ((uint32_t)0x08060008) 	//云台POS起始地			16byte
#define SERIAL_SCREEN_PERIOD 10

typedef enum{
	SCREEN_NOINIT,
	SCREEN_INIT_MODE,
	SCREEN_TEST_MODE,
	SCREEN_GAME_MODE,
}screen_mode_e;

extern uint16_t serial_fric_vel[4];
extern int32_t serial_gimbal_pos[4];
extern screen_mode_e screen_mode;


void Serial_Screen_Task(const void* argu);
void Serial_Screen_Handle(uint8_t* screen_buf);


#endif 
