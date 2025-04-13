/*******************************************************************************
 * @file    data_scope.c
 * @author  ZZJ
 * @date    2022.3.25
 * @brief   MiniBalance �� VOFA+ ������λ����������
 *          ʹ�÷�����
 *              0. Ĭ��ʹ��F4ϵ�У�������Ҫ�޸�DataWave()�еĴ��ڷ��ͼĴ���
 *              1. �� data_scope.h ѡ��ʹ�õ���λ����������Ҫ�޸�ʾ����ͨ����
 *                 ���У���ʹ�� MiniBalance�������ͨ����Ĭ��10
 *              2. ��д __weak void DataWavePkg(void) �����������м�������͵�����
 *                  void DataWavePkg(void)
 *                  {
 *                      the data needed to send:
 *                      DataScope_Get_Channel_Data(float_type_data1);
 *                      DataScope_Get_Channel_Data(float_type_data2);
 *                  }
 *              3. ����ִ�� DataWave(UART_HandleTypeDef* huart)
 *                 ��������һ���Խ�ע�����ݵ�ǰֵ˳�򷢳�
 * @note
 *          ��ʹ�� MiniBalance ��λ��������Ҫ���з�Ƶ���ͣ����⿨��
 *          ��ʹ�� VOFA+ ��λ������λ������Ҫʹ�� Justfloat Э��
 *******************************************************************************
 */
#ifndef __DATA_PRTOCOL_H__
#define __DATA_PRTOCOL_H__

#include "usart.h"

//#define MINIBALANCE       //��Ҫʹ��VOFA+������λ������ע�ʹ˺�
#define MAX_SEND_NUM 15     //MINIBALANCE���10��ͨ����VOFA+���Գ���10��ͨ��

void DataWavePkg(void);                         //��д��������
void DataScope_Get_Channel_Data(float Data);    //��˳��ע�����ӡ����
void DataWave(UART_HandleTypeDef* huart);       //һ���԰�˳��ע�����ݴ�ӡ����λ��

#endif
