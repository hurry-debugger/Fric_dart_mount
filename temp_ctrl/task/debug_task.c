#include "debug_task.h"

#include "cmsis_os.h"

#include "data_scope.h"

#include "sht30.h"

#include "pid.h"

typedef enum
{
	temp,
}debug_type_e;

extern PID_TypeDef temp_pid_t;

debug_type_e debug_type = temp;

void DataWavePkg(void)
{
	switch (debug_type)
	{
		case temp:
		{
			DataScope_Get_Channel_Data(temp_pid_t.Target);
			DataScope_Get_Channel_Data(temp_pid_t.Measure);
			DataScope_Get_Channel_Data(temp_pid_t.Output);
			
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

