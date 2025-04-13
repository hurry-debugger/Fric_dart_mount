#ifndef __KALMAN_FILTERS_H__
#define __KALMAN_FILTERS_H__

#include "stdint.h"
#include "arm_math.h"

#define mat arm_matrix_instance_f32
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv arm_mat_inverse_f32

// һ�׿������˲��ṹ�嶨��,���Ǳ���
typedef struct
{
    float X_last; // ��һʱ�̵����Ž��  X(k-|k-1)
    float X_mid;  // ��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  // ��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  // ��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  // ��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; // ��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     // kalman����
    float A;      // ϵͳ����
    float B;      // ���������
    float Q;      // ״̬��������
    float R;      // �۲ⷽ������
    float H;
} Kalman1_param_t;

// ���׿�����������Ҫ�����м�������壬���ڼ��㣬���������ĵ�һ������
typedef struct
{
    float raw_value;
    float filtered_value[2]; // �������˲��ķ���ֵ��0��λ��1���ٶ�
    mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman2_filter_t;

// ���׿����������ʼ�������������ĵڶ�������
typedef struct
{
    float raw_value;
    float filtered_value[2]; // �������˲��ķ���ֵ��0��λ��1���ٶ�
    float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];
} Kalman2_param_t;

typedef struct // �Ӿ�Ŀ���ٶȲ���
{
    int delay_cnt; // ����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
    int freq;
    int last_time;         // �ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
    float last_position;   // �ϸ�Ŀ��Ƕ�
    float speed;           // �ٶ�
    float last_speed;      // �ϴ��ٶ�
    float processed_speed; // �ٶȼ�����
} speed_calc_data_t;

/* �������˲��������봴������ */
void Kalman1FilterCreate(Kalman1_param_t *p, float Q, float R);
void Kalman1FilterReinit(Kalman1_param_t *p, float Q, float R);
void Kalman1FilterDeinit(Kalman1_param_t *p);
float Kalman1FilterCalc(Kalman1_param_t *p, float dat);

void Kalman2FilterCreate(kalman2_filter_t *F, Kalman2_param_t *I);
float *Kalman2FilterCalc(kalman2_filter_t *F, float signal1, float signal2);

float Kalman2Filter_calc2(kalman2_filter_t *F, float signal1, float signal2);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);

#endif
