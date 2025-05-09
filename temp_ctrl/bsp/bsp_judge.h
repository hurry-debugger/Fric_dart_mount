/**
  * @file bsp_judge.h
  * @version 1.0
  * @date  2022.3.14
	*
  * @brief ����ϵͳ����Э��v1.2�ĸ������ݽṹ��
	*        v1.3��Э��ֻ�Ǹ����˹�������ս���Ĳ��֣�������û��Ӱ��
	*				 ����û��д��ȥ
	*
  *	@author  �����鳭ǿ���
  *
  */
#ifndef BSP_JUDGE_H
#define BSP_JUDGE_H

#include "crc.h"
#include "stdint.h"

#define 	JUDGESEND_PERIOD 	10  //���ͼ����������Խ��Խ��
#define		JUDGESEND_TIME		10

#define   LEN_HEADER    5       //֡ͷ��
#define   LEN_CMDID     2       //�����볤��
#define   LEN_TAIL      2	      //֡βCRC16

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)



typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
}JudgeFrameOffset;

//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
}FrameHeaderOffset;

typedef enum
{
	ID_game_state		  =		0x0001,		//����״̬				1Hz
	ID_game_result		=		0x0002,		//�������				����ʱ����
	ID_robot_HP				=		0x0003,		//������Ѫ��			1Hz
	ID_darts_status		=		0x0105,		//���ڷ���״̬		���ڷ������
	
	ID_event_data			=		0x0101,		//�����¼�����		�¼��ı����
	ID_supply_action	=		0x0102,		//����վ������ʶ	������������
	ID_judge_warning	=		0x0104,		//���о�������		���淢������
	ID_dart_info        =		0x0105,			//�����������	1Hz
	
	ID_robot_status		=		0x0201,		//������״̬����	10Hz
	ID_robot_power		=		0x0202,		//ʵʱ��������		50Hz
	ID_robot_position =		0x0203,		//������λ������	10Hz
	ID_robot_buff			=		0x0204,		//��������������	����״̬�ı����
	ID_AerialRobotEnergy =	0x0205, //���˻�����			10Hz,ֻ���Ϳ���
	ID_robot_hurt			=		0x0206,		//�˺�״̬����		�˺���������
	ID_shoot_data			=		0x0207,		//ʵʱ�������		�ӵ��������
	ID_bullet_remaining = 0x0208,   //����ʣ�෢����	1Hz������/�ڱ�
	ID_RFID_status		=		0x0209,		//������RFID״̬	1Hz
	ID_dart_client    =   0x020A,		//���ڻ����˿ͻ���ָ������ 10Hz
	
	ID_robot_interact=		0x0301,		//�����˼佻������	���ͷ���������,����10Hz
	ID_client_map			= 	0x0303		//�ͻ���С��ͼ��������	��������
}cmd_ID;

//���ݶγ���
typedef enum
{
	LEN_game_state			=		11,		
	LEN_game_result			=		1,		
	LEN_robot_HP				=		32,		
	LEN_darts_status		=		3,		
	LEN_event_data			=		4,		
	LEN_supply_action		=		4,
	LEN_judge_warning		=		2,		
	LEN_darts_info        	=		3, 		
	LEN_robot_status		=		27,		
	LEN_robot_power			=		16,		
	LEN_robot_position 	=		16,		
	LEN_robot_buff			=   1,
	LEN_AerialRobotEnergy	=	2,
	LEN_robot_hurt			=		1,		
	LEN_shoot_data			=		7,		
	LEN_bullet_remaining=		6,		
	LEN_RFID_status			=		4,	
	LEN_dart_client			=   6,
	LEN_robot_interract	=		20,			//�����˼佻���������ݶ�
}cmd_LEN;


//����Ϊ�������ݽṹ��

/* ֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;					//��ʼ�ֽ�
	uint16_t DataLength;	//���ݳ���
	uint8_t  Seq;					//�����
	uint8_t  CRC8;				//crc8У��
} frame_header;

/* ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�*/
typedef __packed struct 
{ 
	uint8_t game_type : 4;				//1���״�ʦ����2���״�ʦ��������4RoboMaster ������3V3��5RoboMaster ������1V1
	uint8_t game_progress : 4;		//0δ��ʼ������1׼���׶Σ�2�Լ�׶Σ�3���뵹��ʱ��4��ս�У�5����������
	uint16_t stage_remain_time;		//��ǰ�׶�ʣ��ʱ�䣬��λ��
	uint64_t SyncTimeStamp;				//�����˽��յ���ָ��ľ�ȷʱ�䣬�����ض��յ���Ч�ķ�������ʱ����Ч
} ext_game_state_t; 

/* ����������ݣ�0x0002������Ƶ�ʣ������������ͣ����ͷ�Χ�����л����ˡ�*/
typedef __packed struct 
{ 
	uint8_t winner;								//0 ƽ�� 1 �췽ʤ�� 2 ����ʤ��
} ext_game_result_t; 

/* ������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t red_5_robot_HP; 
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP;
	uint16_t red_base_HP; 
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP; 	
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
}ext_game_robot_HP_t;

/* ���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л����ˡ�*/
typedef __packed struct
{
	uint8_t dart_belong;							//1�췽���ڣ�2�������� 
	uint16_t stage_remaining_time; 		//����ʱ��ʣ�����ʱ�䣬��λ��
}ext_dart_status_t;

/* �����¼����ݣ�0x0101������Ƶ�ʣ��¼��ı���ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct//v1.61�޸�
{
	uint32_t event_type;
	/*
	bit 0-2��
    bit0 ��������վǰ��Ѫ���ռ��״̬ 1Ϊ��ռ�죻
	bit1 ��������վ�ڲ���Ѫ���ռ��״̬ 1Ϊ��ռ�죻
	bit2 ������������ռ��״̬ 1Ϊ��ռ�죻����RMUL���ã�
	
	bit 3-5��������������״̬��
	bit 3 �����������ؼ�����ռ��״̬ 1Ϊ��ռ�죻 
	bit 4 ����С�������صļ���״̬��1Ϊ�Ѽ��
	bit 5 ����С�������صļ���״̬ 1Ϊ�Ѽ��
	
	bit 6-11�������ߵ�ռ��״̬��
	bit 6-7 �������θߵص�ռ��״̬ 1Ϊ������ռ�� 2Ϊ���Է�ռ�죻 
	bit 8-9 �������θߵص�ռ��״̬ 1Ϊ������ռ�� 2Ϊ���Է�ռ�죻
	bit 10-11 �������θߵص�ռ��״̬ 1Ϊ������ռ�� 2Ϊ���Է�ռ�죻 
	
	bit 12-18 �����������⻤�ܵ�ʣ��ֵ�ٷֱȣ��������룬������������
	bit 19-27 �������һ�λ��м���ǰ��վ����ص�ʱ�䣨0-420������Ĭ��Ϊ0���� 
	bit 28-29 �������һ�λ��м���ǰ��վ����صľ���Ŀ�� ����Ĭ��Ϊ0 1Ϊ����ǰ��վ 2Ϊ���л��ع̶�Ŀ�� 3Ϊ���л������Ŀ�ꣻ
	bit 30-31������������ռ����� 0Ϊδ��ռ�� 1Ϊ������ռ�� 2Ϊ���Է�ռ�� 3Ϊ��˫��ռ�죨�� RMUL ���ã���
	*/
}ext_event_data_t;

/* ����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
	uint8_t supply_projectile_id; 		//����վ�� ID��1 ��2
	uint8_t supply_robot_id; 					//���������� ID��0 Ϊ��ǰ�޻����˲�����1 Ϊ�췽Ӣ�ۻ����˲�����2 Ϊ�췽���̻����˲�����3/4/5 Ϊ�췽���������˲���������Ϊ101/102/103/104/105
	uint8_t supply_projectile_step; 	//�����ڿ���״̬��0 Ϊ�رգ�1 Ϊ�ӵ�׼���У�2 Ϊ�ӵ�����
	uint8_t supply_projectile_num;		//����������50 ��100 ��150 ��200
}ext_supply_projectile_action_t;

/* ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢�����ͣ����ͷ�Χ�����������ˡ�*/
typedef __packed struct
{
	uint8_t level; 						//����ȼ�
	uint8_t foul_robot_id;		//���������ID���и�ʱ��������IDΪ0,���ơ�����ʱ��������IDΪ���������ID
}ext_referee_warning_t;

/* ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������*/
/*���ڷ���ڵ���ʱ�Լ����ڻ���Ŀ�꣬������Ŀ��ѡ�� 1HZ*/
typedef __packed struct
{ 
	uint8_t dart_remaining_time;
	uint8_t dart_info_last_hit : 2;
	uint8_t dart_info_latest_hitcnt : 3;
	uint8_t dart_info_target : 2;
	
	uint8_t dart_info_remain_1 : 1;//����λ��������
	uint8_t dart_info_remain_2 : 8;
}ext_dart_info_t;

/* ����������״̬��0x0201������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{ 
	uint8_t robot_id; 		//����ID
	uint8_t robot_level;	//�����˵ȼ���
	uint16_t remain_HP; 	//������ʣ��Ѫ��
	uint16_t max_HP; 			//����������Ѫ��
	
	uint16_t shooter_id1_17mm_cooling_rate; 		//������1��17mmǹ��ÿ����ȴֵ
	uint16_t shooter_id1_17mm_cooling_limit;		//������1��17mmǹ����������
	uint16_t shooter_id1_17mm_speed_limit;			//������1��17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id2_17mm_cooling_rate;			//������2��17mmǹ��ÿ����ȴֵ
	uint16_t shooter_id2_17mm_cooling_limit;		//������2��17mmǹ����������
	uint16_t shooter_id2_17mm_speed_limit;			//������2��17mm ǹ�������ٶ� ��λ m/s
	uint16_t shooter_id1_42mm_cooling_rate; 		//������42mmǹ��ÿ����ȴֵ
	uint16_t shooter_id1_42mm_cooling_limit;		//������42mmǹ����������
	uint16_t shooter_id1_42mm_speed_limit;			//������42mmǹ�������ٶ� ��λ m/s
	uint16_t chassis_power_limit; 							//�����˵��̹�����������
	//���ص�Դ������
	uint8_t mains_power_gimbal_output : 1; 			//gimbal����� 	1Ϊ��24V�����0Ϊ��24v���
	uint8_t mains_power_chassis_output : 1; 		//chassis����� 1Ϊ��24V�����0Ϊ��24v���
	uint8_t mains_power_shooter_output : 1;			//shooter����� 1Ϊ��24V�����0Ϊ��24v���
} ext_game_robot_status_t;

/* ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	uint16_t chassis_volt; 					//��λ����
	uint16_t chassis_current; 			//��λ����
	float chassis_power;						//��λ�� 
	uint16_t chassis_power_buffer;	//��λ��������ע�����¸��ݹ���������250J 
	uint16_t shooter_id1_17mm_cooling_heat; 		//1��17mm ǹ������
	uint16_t shooter_id2_17mm_cooling_heat;			//2��17mmǹ������
	uint16_t shooter_id1_42mm_cooling_heat;			//42mm ǹ������
}ext_power_heat_data_t;

/* ������λ�ã�0x0203������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct//v1.61
{
	float x;		//��λ��
	float y;
	float z;
}ext_game_robot_pos_t;

/* ���������棺0x0204������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
 uint8_t power_rune_buff;
	/* 
	bit 0��������Ѫ����Ѫ״̬
	bit 1��ǹ��������ȴ�ӳ�
	bit 2�������˷����ӳ�
	bit 3�������˹����ӳ�
	*/
}ext_buff_t;

/* ���л���������״̬��0x0205������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{ 
	uint8_t attack_time;		//�ɹ���ʱ�䣬��λ s��30s �ݼ��� 0	
}ext_aerial_robot_energy_t;

/* �˺�״̬��0x0206������Ƶ�ʣ��˺��������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
	/*
	bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0��
	bit 4-7��Ѫ���仯����:
	         0x0 װ���˺���Ѫ��
	         0x1 ģ����߿�Ѫ��
	         0x2 �����ٿ�Ѫ��
	         0x3 ��ǹ��������Ѫ��
	         0x4 �����̹��ʿ�Ѫ��
	         0x5 װ��ײ����Ѫ
	*/
}ext_robot_hurt_t;

/* ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������ͣ����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
  uint8_t bullet_type;		//�ӵ�����: 1��17mm ���� 2��42mm ����
	uint8_t shooter_id;			//�������ID�� 1:1��17mm�������	2:2��17mm�������		3��42mm �������
  uint8_t bullet_freq;		//�ӵ���Ƶ ��λ Hz
  float bullet_speed;			//�ӵ����� ��λ m/s
}ext_shoot_data_t;

/* �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�10Hz ���ڷ��ͣ����л����ˣ�*/

typedef __packed struct 
{ 
	uint16_t bullet_remaining_num_17mm;
	/*
	17mm�ӵ�ʣ�෢����Ŀ 
	����������/Ӣ�ۻ�����:ȫ�Ӳ�����Ӣ��ʣ��ɷ���17mm��������(������)		
	���л�����/�ڱ�������:�û�����ʣ��ɷ���17mm��������(������/�Կ���)
	*/
	uint16_t bullet_remaining_num_42mm;	//42mm�ӵ�ʣ�෢����Ŀ
	uint16_t coin_remaining_num; 				//ʣ��������
} ext_bullet_remaining_t;

/* ������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct
{
  uint32_t rfid_status;
	/*
	bit 0����������� RFID ״̬��
  bit 1���ߵ������ RFID ״̬��
  bit 2���������ؼ���� RFID ״̬��
  bit 3����������� RFID ״̬��
  bit 4��ǰ�ڸ������ RFID ״̬��
  bit 5����Դ������� RFID ״̬��
  bit 6����Ѫ������� RFID ״̬��
  bit 7�����̻����˲�Ѫ�� RFID ״̬��
  bit 8-31������
  RFID ״̬����ȫ�����Ӧ������򴦷�״̬������з���ռ��ĸߵ�����㣬���ܻ�ȡ��Ӧ������Ч��.
	
	*/
}ext_rfid_status_t;

/* ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct//v1.61
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;

/* �����˼佻�����ݣ�0x0200~0x02ff�����ͷ�Χ����һ�����ˡ�*/
typedef __packed struct 
{ 
	uint8_t data[LEN_robot_interract+6];
} robot_interactive_receivedata_t;//�����˼佻������

int Judge_Read_Data(uint8_t *ReadFromUsart);
extern ext_shoot_data_t Shoot_Data;
extern ext_game_state_t Game_State;
extern ext_dart_client_cmd_t	Dart_Client;                //����բ��״̬
extern ext_dart_info_t          Dart_Info;

#endif //�汾����
