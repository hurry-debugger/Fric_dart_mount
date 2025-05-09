#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"
#include "main.h"

#define TEMP_HUART    		huart6

#define DMA_TEMP_LEN    50 

void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);
void Uart_printf(UART_HandleTypeDef *huart,char *format, ...);

#endif

