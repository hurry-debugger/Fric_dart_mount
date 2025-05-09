#ifndef __FILTER_H
#define __FILTER_H
#include "stdint.h"
#include "math.h"
/*----------------------------------------------------------------------------------------------------------------------/
        *               
        *               һ�׵�ͨ�˲���
        *               
----------------------------------------------------------------------------------------------------------------------*/
/*һ�׵�ͨ�˲���ģʽ*/
#define LPFO_MODE_N 0
#define LPFO_MODE_D 1       //ʹ�ö�̬����


/*һ�׵�ͨ�˲����ṹ������*/
typedef struct
{
    uint8_t mode : 1;       //�Ƿ�ʹ�ö�̬����
    float K;                //�˲�ϵ��

    float last_data;        //��һ�λ�ȡ����
    float now_data;         //���λ�ȡ������

    float last_after_data;        //��һ�λ�ȡ����
    float now_after_data;         //���λ�ȡ������

    float Kw;               //�����ȶ�ʱϵ��
    uint8_t dir_last;   //���ݱ仯����
    uint8_t dir_now;
    uint8_t cnt;            //�ȶ����������٣�����ʱ��
    uint8_t Threshold_A;    //��ֵ1����һ�״����˲������仯�Ƕȴ��ڴ�ֵʱ����������
    uint8_t Threshold_T;    //��ֵ2����һ�״����˲���������ֵ���ڴ�ֵʱ�������������ǿ�˲�����
} LPFOfilter_t;


/*һ�׵�ͨ�˲�����ʼ���ṹ������*/
typedef struct {
    uint8_t mode : 1;
    float K;
    float Kw;
    uint8_t Threshold_A;
    uint8_t Threshold_T;
} LPFO_initstruct;


float LPFOfilter_cal(LPFOfilter_t* pft, float get);
void LPFOfilter_init(LPFOfilter_t* pft,LPFO_initstruct* pftinit);
/*----------------------------------------------------------------------------------------------------------------------/
        *               
        *               ƽ���˲���
        *               
----------------------------------------------------------------------------------------------------------------------*/
#ifndef __AVG_FILTER_H
#define __AVG_FILTER_H


#define AVG_FILTER_NUM 10


typedef struct{
    uint8_t num;
    float array[AVG_FILTER_NUM];
    uint8_t index;
    float after_data;
}AVGfilter_t;


void avg_filter_init(void);
float AVGfilter_cal(AVGfilter_t* avg, float get);
float GetMedianNum(float in, int iFilterLen);
#endif



/*----------------------------------------------------------------------------------------------------------------------/
        *		�����ƴ���Դ�ɿ� V1.1	�人�Ƽ���ѧ  By.YuYi
        *		        CSDN����: http://blog.csdn.net/u011992534
        *               �ſ�ID��NamelessCotrun����С��
        *               �����ƴ���Դ�ɿ�QQȺ��540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               �ٶ�����:�����ƴ���Դ�ɿ�
        *               Copyright(C) �人�Ƽ���ѧ�����ƴ��Ŷ� 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
//#ifndef _ADRC_H_
//#define _ADRC_H_
//typedef struct
//{
//    /*****���Ź��ȹ���*******/
//    float x1;   //����΢����״̬��
//    float x2;   //����΢����״̬��΢����
//    float r;    //ʱ��߶�
//    float h;    //ADRCϵͳ����ʱ��
//    uint16_t N0;//����΢��������ٶȳ���h0=N*h

//    float h0;
//    float fh;   //����΢�ּ��ٶȸ�����
//    /*****����״̬�۲���*******/
//    /******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
//    float z1;
//    float z2;
//    float z3;   //���ݿ��ƶ����������������ȡ���Ŷ���Ϣ
//    float e;    //ϵͳ״̬���
//    float y;    //ϵͳ�����
//    float fe;
//    float fe1;
//    float beta_01;
//    float beta_02;
//    float beta_03;

//    /**********ϵͳ״̬������*********/
//    float e0;   //״̬��������
//    float e1;   //״̬ƫ��
//    float e2;   //״̬��΢����
//    float u0;   //���������ϵͳ���
//    float u;    //���Ŷ�����������


//    /*********��һ�������ʽ*********/
//    float beta_0;   //����
//    float beta_1;   //��������ϲ���
//    float beta_2;   //u0=beta_1*e1+beta_2*e2+(beta_0*e0);
//    /*********�ڶ��������ʽ*********/
//    float alpha1;   //u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
//    float alpha2;   //0<alpha1<1<alpha2
//    float zeta;     //���Զε����䳤��
//    /*********�����������ʽ*********/
//    float h1;       //u0=-fhan(e1,e2,r,h1);
//    uint16_t N1;    //����΢��������ٶȳ���h0=N*h
//    /*********�����������ʽ*********/
//    float c;        //u0=-fhan(e1,c*e2*e2,r,h1);
//    float b0;       //�Ŷ�����
//} Fhan_Data;



//void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2);
//void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC);
//void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback);

//extern Fhan_Data ADRC_Pitch_Controller,ADRC_Roll_Controller;
//#endif


#endif



