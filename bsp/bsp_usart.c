#include "bsp_usart.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"

#include "bsp_remote_ET08.h"
#include "bsp_serial_screen.h"
#include "temp_task.h"

uint8_t dma_dbus_buf[DMA_DBUS_LEN];
uint8_t dma_judge_buf[DMA_JUDGE_LEN];
uint8_t dma_screen_buf[DMA_SCREEN_LEN];
uint8_t dma_temp_buf[DMA_TEMP_LEN];
 /**
  * @brief  �������ڹ��ܺ���
  * @param 	UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	 if (huart->Instance == USART1) //DBUS����
    {
		sbus_callback_handler(&SBUS, dma_dbus_buf);
        memset(dma_dbus_buf, 0, DMA_DBUS_LEN);
        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
    }
	 if (huart->Instance == USART3) //DBUS����
    {
//		Judge_Read_Data(dma_judge_buf);
        memset(dma_dbus_buf, 0, DMA_JUDGE_LEN);
        HAL_UART_Receive_DMA(huart, dma_judge_buf, DMA_JUDGE_LEN);
    }
	if (huart->Instance == UART5) //SCREEN����
    {
		Serial_Screen_Handle(dma_screen_buf);
		memset(dma_screen_buf, 0, DMA_SCREEN_LEN);
        HAL_UART_Receive_DMA(huart, dma_screen_buf, DMA_SCREEN_LEN);
    }
//	if (huart->Instance == USART6) //TEMP����
//    {
//		Temp_Usart_Handle(dma_temp_buf);
//		memset(dma_temp_buf, 0, DMA_TEMP_LEN);
//        HAL_UART_Receive_DMA(huart, dma_screen_buf, DMA_TEMP_LEN);
//    }
}


/**
  * @brief ���ڿ����жϣ�����it.c��ÿ�����ڵ��ж��е��øú�����
  * @param UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //�ж��Ƿ��ǿ����ж�
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //��������жϱ�־�������һֱ���Ͻ����жϣ�
		HAL_UART_AbortReceive(huart);							//ֹͣ����DMA����
		USER_UART_IDLECallback(huart);                        //���ô��ڹ��ܺ���
	}
}


/**
* @brief ���ڳ�ʼ��:ʹ�ܴ��ڿ����ж�,��������DMA����
* @param  ��
* @retval ��
*/
void USER_UART_Init(void)
{
	 __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);
	
	 __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_buf, DMA_JUDGE_LEN);
	
	 __HAL_UART_CLEAR_IDLEFLAG(&SERIAL_SCREEN_HUART);
    __HAL_UART_ENABLE_IT(&SERIAL_SCREEN_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&SERIAL_SCREEN_HUART, dma_screen_buf, DMA_SCREEN_LEN);
	
//	 __HAL_UART_CLEAR_IDLEFLAG(&TEMP_HUART);
//    __HAL_UART_ENABLE_IT(&TEMP_HUART, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&TEMP_HUART, dma_temp_buf, DMA_TEMP_LEN);
	
}
/**
* @brief �മ���ض���
* @param  ��
* @retval ��
*/                                                                              
HAL_StatusTypeDef state;
uint8_t cnt;
void Uart_printf(UART_HandleTypeDef *huart,char *format, ...)
{
	char buf[80]; //������ʱ���飬����ʵ�ʷ��ʹ�С΢��
 
	va_list args;
	va_start(args, format);
	uint16_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
	va_end(args);
	state = HAL_UART_Transmit(huart,(uint8_t *)buf, len, 0xff);
	if(state == HAL_BUSY)
		cnt++;
}
