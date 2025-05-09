#include "sht30.h"
#include "bsp_usart.h"
#include "string.h"
sht30_t sht30;
unsigned char temp_buf[3][7];
uint8_t std_temp_bufhead[7] = "Temp = ";
//ά��sht30����
void Temperture_Analy(uint8_t* string,uint8_t num)
{
	/*
	�����۲죬���ͻ������¶����ݴ���Ϊ���и�ʽ
	<1>Temp = 
	<2>33.25   Humidity = 
	<3>87.34
	��ֻ�е���һ֡����Ϊ"Temp = "�ҵڶ�֡���ݵ�9λΪ'H'ʱ������Ч
	*/
	if((my_strncmp(temp_buf[num],std_temp_bufhead,7) == 0) && string[8] == 'H')
	{
		sht30.temp_fdb[num] = hex_to_dec(string[0]) * 10 + hex_to_dec(string[1])
		+ hex_to_dec(string[3])/10.0 + hex_to_dec(string[4])/100.0;
		memset(temp_buf[num],0,7);
	}
//	for(int i = 0;i < 7;i++)
//	{
//		temp_buf[num][i] = string[i];
//	}
	memcpy(temp_buf[num],string,7);//�洢��һ֡���ݣ���Ҫ�����ж�"Temp = "
}
//�������ƣ�2�ַ����ȽϺ���
//������string1���Ƚ��ַ���  string2��׼�ַ��� count�Ƚϳ���
//����ֵ��2�ַ�����ֵ
uint8_t my_strncmp(uint8_t *string1,uint8_t *string2,size_t count)
{
	uint8_t res = 0,k=1;
    while(*string1!='\0'&&*string2!='\0'&&k<=count)
    {
        k++;
		if(*string1==*string2)
		{
			string1++;
			string2++;     
		}
		else break;
	}
    res=*string1-*string2;
    return res;
}
//ʮ������ascllתdec����
uint8_t hex_to_dec(uint8_t hex)
{
	if(hex>=0x30&&hex<=0x39)
	{
		return (hex-48);
	}
	return 0;
}
//���¶����ݴ�ŵ��������Դ��ڷ��ͳ�ȥ
void move_from_fbd_to_buff(void)
{
	for(int i = 0;i<3;i++)
	{
		if(sht30.temp_fdb[i] - (int)sht30.temp_fdb[i]>=0.5)
		{
			sht30.buff[i] = (int)sht30.temp_fdb[i]+1;
		}
		else
			sht30.buff[i] = (int)sht30.temp_fdb[i];
	}
}
