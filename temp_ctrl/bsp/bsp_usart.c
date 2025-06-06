#include "bsp_usart.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"

#include "sht30.h"

#include "temp.h"

uint8_t dma_temp_buf[DMA_TEMP_LEN];
/**
  * @brief  各个串口功能函数
  * @param 	UART_HandleTypeDef *huart
  * @retval 无
  */
  
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6) //Temp串口
    {
		Temperture_Analy(dma_temp_buf, 0);
        memset(dma_temp_buf, 0, DMA_TEMP_LEN);
        HAL_UART_Receive_DMA(huart, dma_temp_buf, DMA_TEMP_LEN);
    }

}


/**
  * @brief 串口空闲中断（需在it.c中每个串口的中断中调用该函数）
  * @param UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
		HAL_UART_AbortReceive(huart);							//停止本次DMA接收
		USER_UART_IDLECallback(huart);                        //调用串口功能函数
	}
}


/**
* @brief 串口初始化:使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
void USER_UART_Init(void)
{
	 __HAL_UART_CLEAR_IDLEFLAG(&TEMP_HUART);
    __HAL_UART_ENABLE_IT(&TEMP_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&TEMP_HUART, dma_temp_buf, DMA_TEMP_LEN);
}
/**
* @brief 多串口重定向
* @param  无
* @retval 无
*/                                                                              
HAL_StatusTypeDef state;
uint8_t cnt;
void Uart_printf(UART_HandleTypeDef *huart,char *format, ...)
{
	char buf[80]; //定义临时数组，根据实际发送大小微调
 
	va_list args;
	va_start(args, format);
	uint16_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
	va_end(args);
	state = HAL_UART_Transmit(huart,(uint8_t *)buf, len, 0xff);
	if(state == HAL_BUSY)
		cnt++;
}
