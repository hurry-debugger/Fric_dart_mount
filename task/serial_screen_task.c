#include "serial_screen_task.h"
#include "bsp_flash.h"

/**								u/s/p
* @brief ���մ��������ݽ���  55    75    00   00 00   FF 
* @param  ��				head  type  num   val   tail
* @retval ��
*/

uint16_t serial_val[4];
uint8_t serial_falg;

static void Serial_Screen_Handle(uint8_t* screen_buf)
{
	if(screen_buf[0]==0x55&&screen_buf[5]==0xff)
	{
		if(screen_buf[1]=='u')//update��������
		{
			serial_falg = 1;
			switch (screen_buf[2])//num
			{
				case 0:
					serial_val[0] = (screen_buf[3]<<8)|screen_buf[4];
					break;
				case 1:
					serial_val[1] = (screen_buf[3]<<8)|screen_buf[4];
					break;
				case 2:
					serial_val[2] = (screen_buf[3]<<8)|screen_buf[4];
					break;
				case 3:
					serial_val[3] = (screen_buf[3]<<8)|screen_buf[4];
					break;
				default :
					break;
			}
		}
		else if(screen_buf[1]=='s')//save�������ݽ���flash
		{			
			switch (screen_buf[2])//num
			{
				case 0:
					WriteFlashData(NUM_0_POS, &screen_buf[3], 2);//2�ֽ�
					break;
				case 1:
					WriteFlashData(NUM_1_POS, &screen_buf[3], 2);//2�ֽ�					
					break;
				case 2:
					WriteFlashData(NUM_2_POS, &screen_buf[3], 2);//2�ֽ�
					break;
				case 3:
					WriteFlashData(NUM_3_POS, &screen_buf[3], 2);//2�ֽ�
					break;
				default:
					break;
			}
		}
		else if(screen_buf[1]=='p')//����ģʽ
		{			
			switch (screen_buf[2])//num
			{
				case 0:
					serial_val[0] =  0;
					break;
				case 1:
					serial_val[1] = 0;
					break;
				case 2:
					serial_val[2] = 0;
					break;
				case 3:
					serial_val[3] = 0;
					break;
				default :
					break;
			}
		}
	}
}
