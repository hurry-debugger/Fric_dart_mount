#ifndef _SERIAL_SCREEN_TASK_H__
#define _SERIAL_SCREEN_TASK_H__

/* �弶֧���ļ� */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"

#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  

#define NUM_0_POS ((uint32_t)0x08060000)   //���һ����λ����Ϣ��ŵ�λ�ã�16�ֽ�
#define NUM_1_POS ((uint32_t)0x08060010)   //��Ŷ�����λ����Ϣ��ŵ�λ�ã�16�ֽ�
#define NUM_2_POS ((uint32_t)0x08060020)   //���������λ����Ϣ��ŵ�λ�ã�16�ֽ�
#define NUM_3_POS ((uint32_t)0x08060030)   //����ĵ���λ����Ϣ��ŵ�λ�ã�16�ֽ� 

#endif 
