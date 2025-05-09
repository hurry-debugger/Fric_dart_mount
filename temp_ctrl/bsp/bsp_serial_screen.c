#include "cmsis_os.h"

#include "string.h"

#include "bsp_serial_screen.h"
#include "bsp_usart.h"
#include "bsp_flash.h"

uint8_t gimbal_remote_change_flag;//�ⲿʹ��
uint8_t screen_init_flag;///�ⲿʹ��

extern uint8_t dart_num;//relaod_task.h��������

uint16_t serial_fric_vel[4];/* = {5500, 5500, 5500, 5500}*/
int32_t serial_gimbal_pos[4];/*= {-200000,-205000,-210000,-220000}*/
uint8_t serial_num;
uint8_t serial_protect_flag;
uint8_t flash_flag;
uint32_t flash_temp[6];
uint8_t screen_update_flag;

screen_mode_e screen_mode = SCREEN_NOINIT;

static void Screen_Game_Handle(uint8_t* screen_buf);
static void Screen_Test_Handle(uint8_t* screen_buf);

static void Screen_Init_Mode(void);
static void Screen_Test_Mode(void);
static void Screen_Game_Mode(void);

void Serial_Screen_Task(const void* argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
	
	memcpy(serial_fric_vel, (uint32_t*)FRIC_ADDR, sizeof(serial_fric_vel));
	memcpy(serial_gimbal_pos, (uint32_t*)GIMBAL_ADDR, sizeof(serial_gimbal_pos));
	
	while (1)
	{
		taskENTER_CRITICAL();
		switch (screen_mode)
		{
			case SCREEN_INIT_MODE:
			{
				Screen_Init_Mode();
			}	break;
			case SCREEN_TEST_MODE:
			{			
				Screen_Test_Mode();
			}	break;
			case SCREEN_GAME_MODE:
			{
				Screen_Game_Mode();
			}	break;			
			default:
				break;
		}
		taskEXIT_CRITICAL();
		osDelayUntil(&mode_wake_time, SERIAL_SCREEN_PERIOD);
	}
}

void Serial_Screen_Handle(uint8_t* screen_buf)
{
	switch (screen_buf[0])
	{
		case 'i':				//��������ʼ���ɹ�
		{
			screen_mode = SCREEN_INIT_MODE;
		}	break;
		case 't': //test mode
		{			
			screen_mode = SCREEN_TEST_MODE;
			Screen_Test_Handle(screen_buf);
		}	break;
		case 'g': //game mode	
		{	
			screen_mode = SCREEN_GAME_MODE;
			Screen_Game_Handle(screen_buf);
		}	break;
		default:
			break;
	}
}

static void Screen_Init_Mode(void)
{	
	if(screen_init_flag != 1)
	{
		//������ʼ������
		Uart_printf(&SERIAL_SCREEN_HUART, "start.j0.val=100\xff\xff\xff");
		
		//��ʼ��test�����ֵ
		Uart_printf(&SERIAL_SCREEN_HUART, "test.n0.val=%d\xff\xff\xff", serial_fric_vel[3]);
		Uart_printf(&SERIAL_SCREEN_HUART, "test.n1.val=%d\xff\xff\xff", -serial_gimbal_pos[2]);
		Uart_printf(&SERIAL_SCREEN_HUART, "test.n2.val=3\xff\xff\xff");
		
		//��ʼ��gameģʽ��ֵ
		Uart_printf(&SERIAL_SCREEN_HUART, "game.n0.val=%d\xff\xff\xff", serial_fric_vel[0]);	
		Uart_printf(&SERIAL_SCREEN_HUART, "game.n1.val=%d\xff\xff\xff", -serial_gimbal_pos[0]);	
		Uart_printf(&SERIAL_SCREEN_HUART, "game.n2.val=0\xff\xff\xff");		
	}		
	screen_init_flag = 1;				
}

static void Screen_Test_Handle(uint8_t* screen_buf)
{
	switch (screen_buf[1])
	{
		case 'p':				//����ģʽ
			serial_protect_flag = 1;
			break;
		case 'e':				//exit�˳�����ģʽ
			serial_protect_flag = 0;
			break;
		case 't':				//�������������ݸ��µ�Ƭ������  �ȷ��Ͱ�λ
		{
			serial_fric_vel[3] = screen_buf[3] | screen_buf[4]<<8;
			serial_gimbal_pos[2] = -(screen_buf[5] | screen_buf[6]<<8 | screen_buf[7]<<16 | screen_buf[8]<<24);
		}	break;
/*ÿ�ζ��ᵽ���˶��������һ��dart   ����ģʽ����װ����˵�dart2��λ�ã��ʸ�serial_gimbal_pos[2]���ڷ���ǰ�ƶ���̨
			|	dart0	|	dart1	|	dart2	|	dart3	|stop
			---------------------------------------------------
dart_num��	00			l11111111111222222222222333333333333444
			  Ϊ��װ����Ƶ�
*/
		default:
			break;
	}	
}

static void Screen_Test_Mode(void)
{
	if(gimbal_remote_change_flag)
	{
		gimbal_remote_change_flag = 0;
		Uart_printf(&SERIAL_SCREEN_HUART, "test.n1.val=%d\xff\xff\xff", -serial_gimbal_pos[2]);	
	}
}

static void Screen_Game_Handle(uint8_t* screen_buf)
{
	switch (screen_buf[1])
	{
		case 'p':				//����ģʽ
			serial_protect_flag = 1;
			break;
		case 'e':				//exit�˳�����ģʽ
			serial_protect_flag = 0;
			break;
		case 't':				//�������������ݸ��µ�Ƭ������  �ȷ��Ͱ�λ
		{				//������
			serial_fric_vel[screen_buf[2]] = screen_buf[3] | screen_buf[4]<<8;
			serial_gimbal_pos[screen_buf[2]] = -(screen_buf[5] | screen_buf[6]<<8 | screen_buf[7]<<16 | screen_buf[8]<<24);
		}	break;
		case 's':				//���洮�������ݽ�flash
		{
			flash_flag = 1;
		}	break;		
		case 'r':				//��task���µ�Ƭ�����ݵ�������
		{				 //������
			serial_num = screen_buf[2];
			screen_update_flag = 1;
		}	break;	
		default:
			break;
	}	
}

static void Screen_Game_Mode(void)
{
	if(flash_flag)//���洮�������ݽ�flash
	{
		memcpy(flash_temp, serial_fric_vel, 8);
		memcpy(flash_temp+2, serial_gimbal_pos, 16);
		WriteFlashData(FRIC_ADDR, flash_temp, 6);
		flash_flag = 0;
	}
	if(screen_update_flag)//�ڴ������е���������š���Ҫ����¶�Ӧ��ŵ�����
	{
		Uart_printf(&SERIAL_SCREEN_HUART, "game.n1.val=%d\xff\xff\xff", -serial_gimbal_pos[serial_num]);
		Uart_printf(&SERIAL_SCREEN_HUART, "game.n0.val=%d\xff\xff\xff", serial_fric_vel[serial_num]);
		screen_update_flag = 0;
	}
}
