/** @auther
 * @date 2020.7.20
 * @version  V1.0
 * @note
 *һά����ά�������˲���
 *1.һά�������˲���ʹ��ʾ����
 *ʹ��ʱ�ȶ���һ��һάkalmanָ�룬Ȼ�����Kalman1FilterCreate()����һ���˲���
 *ÿ�ζ�ȡ�����������ݺ󼴿ɵ���KalmanFilter()�������ݽ����˲�
 *          Kalman1_param_t p;                  //����һ��һά�������˲����ṹ��
 *          float error_yaw_angle;             //��Ҫ�����˲�������
 *          KalmanCreate(&p,20,200);      //��ʼ�����˲�����Q=20 R=200����
 *          while(1)
 *          {
 *             error_yaw_angle = sersor();                     //��ȡ����
 *             error_yaw_angle = KalmanFilter(&p,SersorData);  //�����ݽ����˲�
 *          }
 *2.��ά�������˲���ʹ��ʾ����
 *ʹ��ʱ�ȶ���һ���ṹ��ָ���ʼ���������������ٶ���һ����άkalman�ṹ��ָ�룬���ڶ���
 *�������ʽ�����м������֮�����Kalman2FilterCreate()����һ���˲���
 *					 Kalman2_param_t F =
 *          {
 *					 		.P_data = {2, 0, 0, 2},
 *					 		.A_data = {1, 0.002, 0, 1},//����ʱ����
 *					 		.H_data = {1, 0, 0, 1},
 *					 		.Q_data = {1, 0, 0, 1},
 *					 		.R_data = {200, 0, 0, 400}//500 1000
 *					 };//��ʼ������kalman����
 *          kalman2_filter_t I;        //����һ����ά�������˲����ṹ��
 *          float Xx, vx;     	     //Xx��λ�ã�vx���ٶȣ���Ϊ�������������ź�������
 *          float *yaw_kf_result ;     //���ڴ洢�������˲������0���˲���λ�ã�1���˲����ٶ�
 *					 Kalman2FilterCreate(&F, &I);    //��ʼ����ά�������˲���
 *          yaw_kf_result = Kalman2FilterCalc(&F,Xx,vx);
 */

#include "KalmanFilter.h"

/**
 * @brief ����һ�׿������˲���
 * @param p���������ṹ�壻
 *        Q��״̬��������Э����
 *        R���۲ⷽ��Э����
 * @note  RԽ�󣬴���������Խ��˵��Խ�����δ���������Ϊ��ȫ�������ƣ���֮��Ϊ��ȫ������������
 */
void Kalman1FilterCreate(Kalman1_param_t *p, float Q, float R)
{
  memset(p, 0, sizeof(Kalman1_param_t));
  p->Q = Q;
  p->R = R;
  p->A = 1;
  p->H = 1;
}

void Kalman1FilterReinit(Kalman1_param_t *p, float Q, float R)
{
  p->Q = Q;
  p->R = R;
}

void Kalman1FilterDeinit(Kalman1_param_t *p)
{
  p->X_last = 0; // ��һʱ�̵����Ž��  X(k-|k-1)
  p->X_mid = 0;  // ��ǰʱ�̵�Ԥ����  X(k|k-1)
  p->X_now = 0;  // ��ǰʱ�̵����Ž��  X(k|k)
  p->P_mid = 0;  // ��ǰʱ��Ԥ������Э����  P(k|k-1)
  p->P_now = 0;  // ��ǰʱ�����Ž����Э����  P(k|k)
  p->P_last = 0; // ��һʱ�����Ž����Э����  P(k-1|k-1)
  p->kg = 0;     // kalman����
}

/**
 * @brief  һ�׿������˲�������
 * @param  p:�������ṹ��
 *         dat:���˲�����
 * @retval �˲��������
 * @attention �����в������������BUt=0������ʵ�ʾ���A=1���۲�����
 *            ״̬��Ϊͬһ������C=1��Q��RΪ�ֵ��������ֱ�Ϊ״̬���̺͹۲ⷽ����
 *            һ��H'��Ϊ������,����Ϊת�þ���
 */
float Kalman1FilterCalc(Kalman1_param_t *p, float dat)
{
  /*-------------------Ԥ��-------------------*/
  /*1.����һʱ�̾�ֵ��Ԥ�⣺X?k'=AX?k-1+BUk	*/
  p->X_mid = p->A * p->X_last; // �ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
  /*2.����һʱ�̷����Ԥ�⣺Pk'=APk-1A'+Q	*/
  p->P_mid = p->A * p->P_last + p->Q; // �ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  /*-------------------����-------------------*/
  /*3.���������棺Kk=Pk'C'/CPk'C'+R	*/
  p->kg = p->P_mid / (p->P_mid + p->R); // �ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  /*4.״̬�����£���Ȩ���㣩��X?k=X?k'+Kk��Yk-CX?k'��*/
  p->X_now = p->X_mid + p->kg * (dat - p->X_mid); // �ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  /*5.������£�Pk=��1-KkC��Pk'	*/
  p->P_now = (1 - p->kg) * p->P_mid; // �ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)

  p->P_last = p->P_now; // ״̬����
  p->X_last = p->X_now;
  return p->X_now; // ���Ԥ����x(k|k)
}

/**
  * @brief �������׿������˲���
  * @param F���������ṹ��,���ڼ�����󼰴����˲������
           I���������ṹ��,���������ʼ����
  * @note  ��mat�ĺ�������ͷ�ļ�"arm_math.h"�����涨����ǹ��ھ���ĸ�������
  */
void Kalman2FilterCreate(kalman2_filter_t *F, Kalman2_param_t *I)
{
  mat_init(&F->xhat, 2, 1, (float *)I->xhat_data); // �����ʼ��,�м���������Ϊ��������m*n
  mat_init(&F->xhatminus, 2, 1, (float *)I->xhatminus_data);
  mat_init(&F->z, 2, 1, (float *)I->z_data);
  mat_init(&F->A, 2, 2, (float *)I->A_data);
  mat_init(&F->H, 2, 2, (float *)I->H_data);
  mat_init(&F->Q, 2, 2, (float *)I->Q_data);
  mat_init(&F->R, 2, 2, (float *)I->R_data);
  mat_init(&F->P, 2, 2, (float *)I->P_data);
  mat_init(&F->Pminus, 2, 2, (float *)I->Pminus_data);
  mat_init(&F->K, 2, 2, (float *)I->K_data);
  mat_init(&F->AT, 2, 2, (float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT, 2, 2, (float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
  //  matrix_value2 = F->A.pData[1];
}

/**
 * @brief  ���׿������˲�������
 * @param  F���������ṹ��,���ڼ�����󼰴����˲������
 *         signal1:���˲�λ����
 *         signal2:���˲��ٶ���
 * @retval    �˲�������� F->filtered_value��0���˲���λ�ã�1���˲����ٶ�
 * @attention ʹ��ǰҪ�ȶ�����������ڴ���˲������ݣ������������ʹ�÷�����
 *            signal1�������һ��Ŀ��λ�ã�����ڶ�ά������,signal2 = 0��ΪĿ���ٶ�Ϊ0��
 *            ������һ�׿�����Ч������
 */
float *Kalman2FilterCalc(kalman2_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP, TEMP21;

  mat_init(&TEMP, 2, 2, (float *)TEMP_data);     //
  mat_init(&TEMP21, 2, 1, (float *)TEMP_data21); //

  F->z.pData[0] = signal1; // z(k)
  F->z.pData[1] = signal2; // z(k)

  // 1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus); //  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  // 2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);  //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP); //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);   //    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  // 3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K); //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);     //      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);       //        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);               //
  mat_mult(&F->Pminus, &F->HT, &TEMP); //
  mat_mult(&TEMP, &F->P, &F->K);       //

  // 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);   //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);         //            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);        //           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat); //    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  // 5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P); //            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);  //
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}

float Kalman2Filter_calc2(kalman2_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP, TEMP21;

  mat_init(&TEMP, 2, 2, (float *)TEMP_data);     //
  mat_init(&TEMP21, 2, 1, (float *)TEMP_data21); //

  F->z.pData[0] = signal1; // z(k)
  F->z.pData[1] = signal2; // z(k)

  // 1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus); //  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

  // 2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);  //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP); //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);   //    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  // 3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K); //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);     //      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);       //        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P); //
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K); //

  // 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);   //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);         //            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);        //           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat); //    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  // 5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P); //            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);  //
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  return F->filtered_value[0];
}

/**
 * @brief  Ŀ���ٶȽ���
 * @param  S���ٶȽ���ṹ��,��ż������������
 *         time:�����Ӿ�����ľ���ʱ��
 * @retval   Ŀ���ٶȽ��㣬���ڶ��׿������ڶ��ź�������
 */
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
  S->delay_cnt++;

  if (time != S->last_time)
  {
    S->speed = (position - S->last_position) / (time - S->last_time) * 2; // �����ٶ�
    S->processed_speed = S->speed;
    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }
  if (S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
  {
    S->processed_speed = 0; // ʱ���������Ϊ�ٶȲ���
  }
  return S->processed_speed; // ��������ٶ�
}

/* ���׿�������ʼ��ʾ�� */
// kalman2_filter_t kalman2_pit_filter;
// kalman2_filter_t kalman2_yaw_filter;
// float *pit_kf_result;
// float *yaw_kf_result;

// kalman2_filter_t kalman2_test1_filter;
// float *test1_kf_result;
// kalman2_filter_t kalman2_test2_filter;
// float *test2_kf_result;

// void Kalman_init(void)
//{
//     Kalman2_param_t kalman2_pit_param =
//     {
//         .P_data = {2, 0, 0, 2},
//         .A_data = {1, 0.002, 0, 1},//����ʱ����
//         .H_data = {1, 0, 0, 1},
//         .Q_data = {1, 0, 0, 1},
//         .R_data = {200, 0, 0, 400}//200 400//500 1000
//     };//��ʼ������kalman����;

//    Kalman2_param_t kalman2_yaw_param =
//    {
//        .P_data = {2, 0, 0, 2},
//        .A_data = {1, 0.002, 0, 1},//����ʱ����
//        .H_data = {1, 0, 0, 1},
//        .Q_data = {1, 0, 0, 1},
//        .R_data = {200, 0, 0, 400}//200 400//500 1000
//    };//��ʼ������kalman����;
//
//    Kalman2FilterCreate(&kalman2_pit_filter, &kalman2_pit_param);
//    Kalman2FilterCreate(&kalman2_yaw_filter, &kalman2_yaw_param);
//}
