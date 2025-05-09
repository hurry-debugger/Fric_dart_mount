#include "temp.h"
#include "cmsis_os.h"

#include "pid.h"
#include "sht30.h"

PID_TypeDef temp_pid_t;

static void Temp_Init(void);

void Temp_Task(void const* argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	Temp_Init();
	while(1)
	{
		
		TIM2->CCR3 = PID_Calculate(&temp_pid_t, sht30.temp_fdb[0], sht30.temp_ref);
		
		osDelayUntil(&mode_wake_time, TEMP_PERIOD);
	}
}

static void Temp_Init(void)
{
	sht30.temp_ref = 30;
	
	PID_Init(&temp_pid_t, TIM2_CCR3_MAX, 0, 0, \
								20000, 0.005, 0, \
								0, 0, \
								0, 0, 0);
}
