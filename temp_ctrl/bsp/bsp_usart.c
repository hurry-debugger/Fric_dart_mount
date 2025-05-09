#include "bsp_usart.h"

#include "string.h"
#include "stdio.h"
#include "stdarg.h"

#include "sht30.h"

#include "temp.h"

uint8_t dma_temp_buf[DMA_TEMP_LEN];
/**
  * @brief  �������ڹ��ܺ���
  * @param 	UART_HandleTypeDef *huart
  * @retval ��
  */
  
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART6) //Temp����
    {
		Temperture_Analy(dma_temp_buf, 0);
        memset(dma_temp_buf, 0, DMA_TEMP_LEN);
        HAL_UART_Receive_DMA(huart, dma_temp_buf, DMA_TEMP_LEN);
    }

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
	 __HAL_UART_CLEAR_IDLEFLAG(&TEMP_HUART);
    __HAL_UART_ENABLE_IT(&TEMP_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&TEMP_HUART, dma_temp_buf, DMA_TEMP_LEN);
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
