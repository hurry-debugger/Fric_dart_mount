/**
  *************************************************************************
  * @brief  一阶低通滤波器
  * @author ZZJ
  * @date   2020/9/27
  * @note   使用时，在需要处理的数据变量文件中，包含头文件，
  *         编写初始化函数，主函数中完成初始化，在需要滤波的地方调用滤波器函数
  *         调参：一般模式下，只用调K； 动态模式下，需要调A，T，和Kw
  *************************************************************************
  */
#include "filter.h"


/**
  * @brief  一阶低通滤波器结构体初始化函数
  * @param  pft      滤波器结构体指针变量
  * @param  pftinit  滤波器结构体初始化结构体指针变量
  * @retval 无
  */
void LPFOfilter_init(LPFOfilter_t* pft,LPFO_initstruct* pftinit)
{
    pft->mode = pftinit->mode;
    pft->K    = pftinit->K;
    pft->last_data       = pft->now_data =
                               pft->last_after_data = pft->now_after_data = 0;
    if(pftinit->mode)  //如果启用滤波系数的动态调节
    {
        pft->Kw = pftinit->Kw;
        pft->Threshold_A = pftinit->Threshold_A;
        pft->Threshold_T = pftinit->Threshold_T;
        pft->cnt = 0;
        pft->dir_last = pft->dir_now = 0;
    }
}


/**
  * @brief  一阶低通滤波器功能函数
  * @param  pft  滤波器结构体指针变量
  * @retval 无
  */
float LPFOfilter_cal(LPFOfilter_t* pft, float get)
{
    pft->now_data = get;

    if(pft->mode)  //如果使用动态调整
    {
        /*根据连续真实数据判断数据状态*/
        if(pft->now_data > pft->last_data + pft->Threshold_A)  //判断变化方向
            pft->dir_now = 1;
        else if(pft->now_data < pft->last_data - pft->Threshold_A)
            pft->dir_now = 2;
        else
            pft->dir_now = 0;
        pft->last_data = pft->now_data;

        /*增量稳定性判断*/
        if(pft->dir_now == pft->dir_last)  //判断前后两次变化方向是否相等  // && pft->dir_last != 0
        {
            pft->cnt+=1;
            if(pft->cnt >= pft->Threshold_T)  //比较 单向变化持续时长
            {
                pft->K += 0.1f;  //提高灵敏度，增强跟随效果
                if(pft->K >= 1) pft->K = 1;
                pft->cnt = pft->Threshold_T;  //防止溢出
            }
        }
        else
        {
            //pft->cnt = 0;
            pft->K = pft->Kw;  //数据围绕某值波动时，降低灵敏度，增强滤波效果
        }
        pft->dir_last = pft->dir_now;
    }
    /*一阶低通滤波*/
    pft->now_after_data = pft->K*pft->now_data + (1-pft->K)*pft->last_after_data;

    pft->last_after_data = pft->now_after_data;

    return pft->now_after_data;
}

//LPFOfilter_t l_laser_filter;
//LPFOfilter_t fb_laser_filter;
//LPFOfilter_t y_vision_error_filter;
//LPFOfilter_t x_vision_error_filter;
//LPFOfilter_t chassis_set_filter[4];
//LPFOfilter_t chassis_fdb_filter[4];
/**
  * @brief  一阶滤波器全部初始化函数
  */
void MY_LPFO_init(void)
{
//    uint8_t i = 0;
//    LPFO_initstruct LPFO_initstr;

//    /*左侧距离*/
//    LPFO_initstr.mode        = LPFO_MODE_N;
//    LPFO_initstr.K           = 0.3f;
//    LPFOfilter_init(&l_laser_filter,&LPFO_initstr);

//    /*前后距离*/
//    LPFO_initstr.mode        = LPFO_MODE_N;
//    LPFO_initstr.K           = 0.2f;
//    LPFOfilter_init(&fb_laser_filter,&LPFO_initstr);

//    /*底盘速度反馈值*/
//    LPFO_initstr.mode        = LPFO_MODE_N;
//    LPFO_initstr.K           = 0.3f;
//    for(i=0; i<4; i++)
//        LPFOfilter_init(&chassis_fdb_filter[i],&LPFO_initstr);

//    /*视觉误差*/
//    LPFO_initstr.mode        = LPFO_MODE_N;
//    LPFO_initstr.Kw          = 0.6f;
//    LPFOfilter_init(&y_vision_error_filter,&LPFO_initstr);
//    LPFOfilter_init(&x_vision_error_filter,&LPFO_initstr);
}


/**
  *************************************************************************
  * @brief  平均滤波器
  * @author ZZJ
  * @date   2020/11/19
  * @note   
  *************************************************************************
  */
float AVGfilter_cal(AVGfilter_t* avg, float get)
{
    float res = 0;
    if(avg->index < avg->num)
    {
        avg->array[avg->index++] = get;
        res = get;
    }
    else
    {
        float sum = 0;
        avg->array[(avg->index++)%avg->num] = get;
        for(uint8_t i = 0; i<avg->num; i++)
            sum += avg->array[i];
        res = sum/avg->num;
    }
    return res;
}


AVGfilter_t avg_filter;
void avg_filter_init(void)
{
    avg_filter.num = 5;
    avg_filter.after_data = 0;
    avg_filter.index = 0;
    
    for(uint8_t i = 0; i<avg_filter.num; i++)
        avg_filter.array[i] = 0;
}


/*----------------------------------------------------------------------------------------------------------------------/
        *               本程序只供购买者学习使用，版权著作权属于无名科创团队，
        *               无名科创团队将飞控程序源码提供给购买者，
        *               购买者要为无名科创团队提供保护，
        *               未经作者许可，不得将源代码提供给他人
        *               不得将源代码放到网上供他人免费下载，
        *               更不能以此销售牟利，如发现上述行为，
        *               无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
        *               生命不息、奋斗不止；前人栽树，后人乘凉！！！
        *               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
-----------------------------------------------------------------------------------------------------------------------/
	*		无名科创开源飞控 V1.1	武汉科技大学  By.YuYi
	*		CSDN博客: http://blog.csdn.net/u011992534
	*               优酷ID：NamelessCotrun无名小哥
	*               无名科创开源飞控QQ群：540707961
        *               https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
        *               百度贴吧:无名科创开源飞控
        *               修改日期:2017/10/30
        *               版本：V1.1
        *               版权所有，盗版必究。
        *               Copyright(C) 武汉科技大学无名科创团队 2017-2019
        *               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
//Fhan_Data ADRC_Pitch_Controller;
//Fhan_Data ADRC_Roll_Controller;
//const float ADRC_Unit[3][15]=
//{
//    /*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
//    /*  r     h      N                  beta_01   beta_02    beta_03     b0       beta_0  beta_1  beta_2   N1     C    alpha1  alpha2*/
//    {300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50},
//    {300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50},
//    {50000  ,0.005 , 30,              100,      2000,      10000,    5    ,    0.002,   10,        0.001,    5,    5,    0.5,   1.05,   50},
//};


//float Constrain_Float(float amt, float low, float high) {
//    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
//}


//int16_t Sign_ADRC(float Input)
//{
//    int16_t output=0;
//    if(Input>1E-6f) output=1;
//    else if(Input<-1E-6f) output=-1;
//    else output=0;
//    return output;
//}


//int16_t Fsg_ADRC(float x,float d)
//{
//    int16_t output=0;
//    output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
//    return output;
//}


//void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2)
//{
//    fhan_Input1->r=ADRC_Unit[0][0];
//    fhan_Input1->h=ADRC_Unit[0][1];
//    fhan_Input1->N0=(uint16_t)(ADRC_Unit[0][2]);
//    fhan_Input1->beta_01=ADRC_Unit[0][3];
//    fhan_Input1->beta_02=ADRC_Unit[0][4];
//    fhan_Input1->beta_03=ADRC_Unit[0][5];
//    fhan_Input1->b0=ADRC_Unit[0][6];
//    fhan_Input1->beta_0=ADRC_Unit[0][7];
//    fhan_Input1->beta_1=ADRC_Unit[0][8];
//    fhan_Input1->beta_2=ADRC_Unit[0][9];
//    fhan_Input1->N1=(uint16_t)(ADRC_Unit[0][10]);
//    fhan_Input1->c=ADRC_Unit[0][11];

//    fhan_Input1->alpha1=ADRC_Unit[0][12];
//    fhan_Input1->alpha2=ADRC_Unit[0][13];
//    fhan_Input1->zeta=ADRC_Unit[0][14];

//    fhan_Input2->r=ADRC_Unit[1][0];
//    fhan_Input2->h=ADRC_Unit[1][1];
//    fhan_Input2->N0=(uint16_t)(ADRC_Unit[1][2]);
//    fhan_Input2->beta_01=ADRC_Unit[1][3];
//    fhan_Input2->beta_02=ADRC_Unit[1][4];
//    fhan_Input2->beta_03=ADRC_Unit[1][5];
//    fhan_Input2->b0=ADRC_Unit[1][6];
//    fhan_Input2->beta_0=ADRC_Unit[1][7];
//    fhan_Input2->beta_1=ADRC_Unit[1][8];
//    fhan_Input2->beta_2=ADRC_Unit[1][9];
//    fhan_Input2->N1=(uint16_t)(ADRC_Unit[1][10]);
//    fhan_Input2->c=ADRC_Unit[1][11];

//    fhan_Input2->alpha1=ADRC_Unit[1][12];
//    fhan_Input2->alpha2=ADRC_Unit[1][13];
//    fhan_Input2->zeta=ADRC_Unit[1][14];
//}



////ADRC最速跟踪微分器TD，改进的算法fhan
//void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
//{
//    float d=0,a0=0,y=0,a1=0,a2=0,a=0;
//    float x1_delta=0;//ADRC状态跟踪误差项
//    x1_delta=fhan_Input->x1-expect_ADRC;//用x1-v(k)替代x1得到离散更新公式
//    fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟踪微分器速度超调问题
//    d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
//    a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
//    y=x1_delta+a0;//y=x1+a0
//    a1=sqrt(d*(d+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
//    a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
//    a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
//    fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
//                   -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
//    fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
//    fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2
//}


////原点附近有连线性段的连续幂次函数
//float Fal_ADRC(float e,float alpha,float zeta)
//{
//    int16_t s=0;
//    float fal_output=0;
//    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
//    fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS(e),alpha)*Sign_ADRC(e)*(1-s);
//    return fal_output;
//}


///************扩张状态观测器********************/
////状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
//void ESO_ADRC(Fhan_Data *fhan_Input)
//{
//    fhan_Input->e=fhan_Input->z1-fhan_Input->y;//状态误差

//    fhan_Input->fe=Fal_ADRC(fhan_Input->e,0.5,fhan_Input->h);//非线性函数，提取跟踪状态与当前状态误差
//    fhan_Input->fe1=Fal_ADRC(fhan_Input->e,0.25,fhan_Input->h);

//    /*************扩展状态量更新**********/
//    fhan_Input->z1+=fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
//    fhan_Input->z2+=fhan_Input->h*(fhan_Input->z3
//                                   -fhan_Input->beta_02*fhan_Input->fe
//                                   +fhan_Input->b0*fhan_Input->u);
////ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
//    fhan_Input->z3+=fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
//}


///************非线性组合****************/
///*
//void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
//{
//      float d=0,a0=0,y=0,a1=0,a2=0,a=0;
//      float Sy=0,Sa=0;//ADRC状态跟踪误差项
//      fhan_Input->h1=fhan_Input->N1*fhan_Input->h;
//      d=fhan_Input->r*fhan_Input->h1*fhan_Input->h1;
//      a0=fhan_Input->h1*fhan_Input->c*fhan_Input->e2;
//      y=fhan_Input->e1+a0;
//      a1=sqrt(d*(d+8*ABS(y)));
//      a2=a0+Sign_ADRC(y)*(a1-d)/2;
//      Sy=Fsg_ADRC(y,d);
//      a=(a0+y-a2)*Sy+a2;
//      Sa=Fsg_ADRC(a,d);
//      fhan_Input->u0=-fhan_Input->r*((a/d)-Sign_ADRC(a))*Sa-fhan_Input->r*Sign_ADRC(a);
//      //a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
//      //fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
//      //                -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
//}
//*/


//void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
//{
//    float temp_e2=0;
//    temp_e2=Constrain_Float(fhan_Input->e2,-3000,3000);
//    fhan_Input->u0=fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
//                   +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);
//}


//void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback_ADRC)
//{
//    /*自抗扰控制器第1步*/
//    /********
//        **
//        **
//        **
//        **
//        **
//     ********/
//    /*****
//    安排过度过程，输入为期望给定，
//    由TD跟踪微分器得到：
//    过度期望信号x1，过度期望微分信号x2
//    ******/
//    Fhan_ADRC(fhan_Input,expect_ADRC);
//    /*自抗扰控制器第2步*/
//    /********
//            *
//            *
//       ****
//     *
//     *
//     ********/
//    /************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
//    fhan_Input->y=feedback_ADRC;
//    /*****
//    扩张状态观测器，得到反馈信号的扩张状态：
//    1、状态信号z1；
//    2、状态速度信号z2；
//    3、状态加速度信号z3。
//    其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
//    经过非线性函数映射，乘以beta系数后，
//    组合得到未加入状态加速度估计扰动补偿的原始控制量u
//    *********/
//    ESO_ADRC(fhan_Input);//低成本MEMS会产生漂移，扩展出来的z3此项会漂移，目前暂时未想到办法解决，未用到z3
//    /*自抗扰控制器第3步*/
//    /********
//           **
//         **
//       **
//         **
//           **
//     ********/
//    /********状态误差反馈率***/
//    fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;//状态积分项
//    fhan_Input->e1=fhan_Input->x1-fhan_Input->z1;//状态偏差项
//    fhan_Input->e2=fhan_Input->x2-fhan_Input->z2;//状态微分项
//    /********线性组合*******/
//    /*
//     fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
//                   +fhan_Input->beta_1*fhan_Input->e1
//                   +fhan_Input->beta_2*fhan_Input->e2;
//    */
//    Nolinear_Conbination_ADRC(fhan_Input);
//    /**********扰动补偿*******/
//    //fhan_Input->u=fhan_Input->u0
//    //             -fhan_Input->z3/fhan_Input->b0;
//    //由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，目前不加入扰动补偿控制量
//    fhan_Input->u=Constrain_Float(fhan_Input->u0,-200,200);
//}
