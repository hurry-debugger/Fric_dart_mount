#include "temp_task.h"

#include "string.h"
#include "cmsis_os.h"


void Temp_Task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	
	while (1)
	{
		
		
		osDelayUntil(&mode_wake_time, TEMP_PERIOD);
	}
}

void Temp_Usart_Handle(uint8_t* temp_buf)
{
	
}
