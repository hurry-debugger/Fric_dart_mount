#include "bsp_judge.h"
#include "string.h"
#include "usart.h"
#include "bsp_usart.h"
#include "cmsis_os.h"

/*****************���ݽṹ�嶨��**********************/
frame_header						FrameHeader;		//֡ͷ

ext_game_state_t					Game_State;
ext_game_result_t					Game_Result;
ext_game_robot_HP_t               	RobotHP;                   //����Ѫ��
ext_dart_status_t					Dart_Status;               //���ڷ���״̬
ext_event_data_t					Event_Data;                //�¼�����
ext_supply_projectile_action_t		Supply_Projectile_Action;  //��������
ext_referee_warning_t				Referee_Warning;           //�ܷ����
ext_dart_info_t					    Dart_Info;              //���ڷ����������
ext_game_robot_status_t				Game_Robot_Status;
ext_power_heat_data_t				Power_Heat_Data;
ext_game_robot_pos_t				Robot_Position;
ext_buff_t							Buff;
ext_aerial_robot_energy_t			Aerial_Robot_Energy;
ext_robot_hurt_t					Robot_Hurt;
ext_shoot_data_t					Shoot_Data;
ext_bullet_remaining_t				Bullet_Remaining;
ext_rfid_status_t					RFID_Status;
ext_dart_client_cmd_t				Dart_Client;                //����բ��״̬
robot_interactive_receivedata_t		Comm_getdata;
/******************************************************/

uint16_t Robot_Self_HP;//����Ѫ��

/**
  * @brief  ��ȡ�������ݺ����������жϺ�����ֱ�ӵ��ý��ж�ȡ
  * @param  ��Ӧ���ڵĻ���������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
int Judge_Read_Data(uint8_t *ReadFromUsart) 
{
	uint8_t  retval_tf = FALSE; //������ʵ�Ա�־λ,ÿ�ζ�ȡʱ��Ĭ��ΪFALSE
	uint16_t judge_length;			//�����ֽڳ���
	int      CmdID=0;				
	if(ReadFromUsart == NULL)
	{
		return -1;
		
	}
	
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);  //д��֡ͷ
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:     //0x0001
						memcpy(&Game_State, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:    //0x0002
						memcpy(&Game_Result, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_robot_HP:       //0x0003
						memcpy(&RobotHP, (ReadFromUsart + DATA), LEN_robot_HP);
					break;
					
					case ID_darts_status:		//0x0105
						memcpy(&Dart_Status, (ReadFromUsart + DATA), LEN_darts_status);
					break;
					
					case ID_event_data:    	 //0x0101
						memcpy(&Event_Data, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_action:   //0x0102
						memcpy(&Supply_Projectile_Action, (ReadFromUsart + DATA), LEN_supply_action);
					break;
					
					case ID_judge_warning:  	//0x0104
						memcpy(&Referee_Warning, (ReadFromUsart + DATA), LEN_judge_warning);
					break;
					
					case ID_dart_info:  //0x0105
						memcpy(&Dart_Info, (ReadFromUsart + DATA), LEN_darts_info);
					break;
					
					case ID_robot_status:     //0x0201
						memcpy(&Game_Robot_Status, (ReadFromUsart + DATA), LEN_robot_status);
					break;
					
					case ID_robot_power:      //0x0202
						memcpy(&Power_Heat_Data, (ReadFromUsart + DATA), LEN_robot_power);
					break;
					
					case ID_robot_position:   //0x0203
						memcpy(&Robot_Position, (ReadFromUsart + DATA), LEN_robot_position);
					break;
					
					case ID_robot_buff:      	//0x0204
						memcpy(&Buff, (ReadFromUsart + DATA), LEN_robot_buff);
					break;
					
					case ID_AerialRobotEnergy:   //0x0205
						memcpy(&Aerial_Robot_Energy, (ReadFromUsart + DATA), LEN_AerialRobotEnergy);
					break;
					
					case ID_robot_hurt:      		//0x0206
						memcpy(&Robot_Hurt, (ReadFromUsart + DATA), LEN_robot_hurt);
					
					case ID_shoot_data:      			//0x0207�������Ϣ
						memcpy(&Shoot_Data, (ReadFromUsart + DATA), LEN_shoot_data);
					break;
					
					case ID_bullet_remaining:     //0x0208
						memcpy(&Bullet_Remaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
					break;
					
					case ID_RFID_status:      		//0x0209
						memcpy(&RFID_Status, (ReadFromUsart + DATA), LEN_RFID_status);
					break;
					
					case ID_dart_client:      		//0x020A
						memcpy(&Dart_Client, (ReadFromUsart + DATA), LEN_dart_client);
					break;
					
					case ID_robot_interact:      //0x0301
						memcpy(&Comm_getdata, (ReadFromUsart + DATA), LEN_robot_interract);
					break;
				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(FrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(FrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	return retval_tf;
}
