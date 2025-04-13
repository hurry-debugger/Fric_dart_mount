#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"
#include "main.h"

#define DBUS_HUART    		huart1
#define JUDGE_HUART   		huart3
#define SERIAL_SCREEN_HUART huart5
#define TOF_HUART			huart4
#define TEMP_HUART 			huart6

#define DMA_DBUS_LEN    25 
#define DMA_JUDGE_LEN 	100
#define DMA_SCREEN_LEN 	20
#define DMA_TOF_LEN		10
#define DMA_TEMP_LEN   30

void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);
void Uart_printf(UART_HandleTypeDef *huart,char *format, ...);

#endif

