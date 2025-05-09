	#include "bsp_usart.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"

#include "bsp_remote_ET08.h"
#include "bsp_serial_screen.h"
#include "bsp_vision.h"
#include "tof.h"
#include "bsp_judge.h"

uint8_t dma_dbus_buf[DMA_DBUS_LEN];
uint8_t dma_judge_buf[DMA_JUDGE_LEN];
uint8_t dma_vision_buf[DMA_VISION_LEN];
uint8_t dma_screen_buf[DMA_SCREEN_LEN];
uint8_t dma_tof_buf[DMA_TOF_LEN];
 /**
  * @brief  各个串口功能函数
  * @param 	UART_HandleTypeDef *huart
  * @retval 无
  */
 
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
 //DBUS串口
	if (huart->Instance == USART1)
    {
		sbus_callback_handler(&SBUS, dma_dbus_buf);
        memset(dma_dbus_buf, 0, DMA_DBUS_LEN);
        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
    }
//Judge串口	
	 if (huart->Instance == USART3)
    {
		Judge_Read_Data(dma_judge_buf);
//      memset(dma_dbus_buf, 0, DMA_JUDGE_LEN);
        HAL_UART_Receive_DMA(huart, dma_judge_buf, DMA_JUDGE_LEN);
    }
//SCREEN串口
	if (huart->Instance == UART5)
    {
		Serial_Screen_Handle(dma_screen_buf);
//		memset(dma_screen_buf, 0, DMA_SCREEN_LEN);
        HAL_UART_Receive_DMA(huart, dma_screen_buf, DMA_SCREEN_LEN);
    }
//TOF串口
	if (huart->Instance == UART4)
    {
		TOF_Usart_Handle(dma_tof_buf);
        HAL_UART_Receive_DMA(huart, dma_tof_buf, DMA_TOF_LEN);
    }
//VISION串口
	if (huart->Instance == USART6)
    {
		Vision_Usart_Handle(dma_vision_buf);
        HAL_UART_Receive_DMA(huart, dma_vision_buf, DMA_VISION_LEN);
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
	//遥控器
	 __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);
	
	//裁判系统
	 __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_buf, DMA_JUDGE_LEN);
	
	//串口屏
	 __HAL_UART_CLEAR_IDLEFLAG(&SERIAL_SCREEN_HUART);
    __HAL_UART_ENABLE_IT(&SERIAL_SCREEN_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&SERIAL_SCREEN_HUART, dma_screen_buf, DMA_SCREEN_LEN);
	
	//视觉
	 __HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);
    __HAL_UART_ENABLE_IT(&VISION_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&VISION_HUART, dma_vision_buf, DMA_VISION_LEN);
	
	//TOF
	 __HAL_UART_CLEAR_IDLEFLAG(&TOF_HUART);
    __HAL_UART_ENABLE_IT(&TOF_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&TOF_HUART, dma_tof_buf, DMA_TOF_LEN);
}

/**
* @brief 多串口重定向
* @param  无
* @retval 无
*/
char buf[80]; //定义临时数组，根据实际发送大小微调
void Uart_printf(UART_HandleTypeDef *huart,char *format, ...)
{ 
	va_list args;
	va_start(args, format);
	uint16_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
	va_end(args);
	HAL_UART_Transmit_DMA(huart,(uint8_t *)buf, len);
}
