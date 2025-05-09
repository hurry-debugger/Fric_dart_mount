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
  * @brief  �������ڹ��ܺ���
  * @param 	UART_HandleTypeDef *huart
  * @retval ��
  */
 
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
 //DBUS����
	if (huart->Instance == USART1)
    {
		sbus_callback_handler(&SBUS, dma_dbus_buf);
        memset(dma_dbus_buf, 0, DMA_DBUS_LEN);
        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
    }
//Judge����	
	 if (huart->Instance == USART3)
    {
		Judge_Read_Data(dma_judge_buf);
//      memset(dma_dbus_buf, 0, DMA_JUDGE_LEN);
        HAL_UART_Receive_DMA(huart, dma_judge_buf, DMA_JUDGE_LEN);
    }
//SCREEN����
	if (huart->Instance == UART5)
    {
		Serial_Screen_Handle(dma_screen_buf);
//		memset(dma_screen_buf, 0, DMA_SCREEN_LEN);
        HAL_UART_Receive_DMA(huart, dma_screen_buf, DMA_SCREEN_LEN);
    }
//TOF����
	if (huart->Instance == UART4)
    {
		TOF_Usart_Handle(dma_tof_buf);
        HAL_UART_Receive_DMA(huart, dma_tof_buf, DMA_TOF_LEN);
    }
//VISION����
	if (huart->Instance == USART6)
    {
		Vision_Usart_Handle(dma_vision_buf);
        HAL_UART_Receive_DMA(huart, dma_vision_buf, DMA_VISION_LEN);
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
	//ң����
	 __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);
	
	//����ϵͳ
	 __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_buf, DMA_JUDGE_LEN);
	
	//������
	 __HAL_UART_CLEAR_IDLEFLAG(&SERIAL_SCREEN_HUART);
    __HAL_UART_ENABLE_IT(&SERIAL_SCREEN_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&SERIAL_SCREEN_HUART, dma_screen_buf, DMA_SCREEN_LEN);
	
	//�Ӿ�
	 __HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);
    __HAL_UART_ENABLE_IT(&VISION_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&VISION_HUART, dma_vision_buf, DMA_VISION_LEN);
	
	//TOF
	 __HAL_UART_CLEAR_IDLEFLAG(&TOF_HUART);
    __HAL_UART_ENABLE_IT(&TOF_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&TOF_HUART, dma_tof_buf, DMA_TOF_LEN);
}

/**
* @brief �മ���ض���
* @param  ��
* @retval ��
*/
char buf[80]; //������ʱ���飬����ʵ�ʷ��ʹ�С΢��
void Uart_printf(UART_HandleTypeDef *huart,char *format, ...)
{ 
	va_list args;
	va_start(args, format);
	uint16_t len = vsnprintf((char *)buf, sizeof(buf), (char *)format, args);
	va_end(args);
	HAL_UART_Transmit_DMA(huart,(uint8_t *)buf, len);
}
