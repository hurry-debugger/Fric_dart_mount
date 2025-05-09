#include "sht30.h"
#include "bsp_usart.h"
#include "string.h"
sht30_t sht30;
unsigned char temp_buf[3][7];
uint8_t std_temp_bufhead[7] = "Temp = ";
//维特sht30解算
void Temperture_Analy(uint8_t* string,uint8_t num)
{
	/*
	经过观察，发送回来的温度数据大致为下列格式
	<1>Temp = 
	<2>33.25   Humidity = 
	<3>87.34
	故只有当第一帧数据为"Temp = "且第二帧数据第9位为'H'时数据有效
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
	memcpy(temp_buf[num],string,7);//存储上一帧数据，主要用于判断"Temp = "
}
//函数名称：2字符串比较函数
//参数：string1待比较字符串  string2标准字符串 count比较长度
//返回值：2字符串差值
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
//十六进制ascll转dec整型
uint8_t hex_to_dec(uint8_t hex)
{
	if(hex>=0x30&&hex<=0x39)
	{
		return (hex-48);
	}
	return 0;
}
//将温度数据存放到数组中以串口发送出去
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
