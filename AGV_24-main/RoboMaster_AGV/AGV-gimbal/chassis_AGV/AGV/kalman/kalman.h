/** ******************************************************************************
  * @file    kalman.h                                                            *
  * @author  Liu heng                                                            *
  * @version V1.0.0                                                              *
  * @date    27-August-2013                                                      *
  * @brief   Hearder file for kalman filter                                      *
  *                                                                              *
  ********************************************************************************
  *          �˴�������⴫����ʹ�ã�����ע��������                              *
  ********************************************************************************/
#ifndef _KALMAN_H
#define _KALMAN_H

#include "stdlib.h"

typedef struct {
    float X_last; //��һʱ�̵����Ž��
    float X_mid;  //��ǰʱ�̵�Ԥ����
    float X_now;  //��ǰʱ�̵����Ž��
    float P_mid;  //��ǰʱ��Ԥ������Э����
    float P_now;  //��ǰʱ�����Ž����Э����
    float P_last; //��һʱ�����Ž����Э����
    float kg;     //kalman����
    float A;      //ϵͳ����
    float Q;
    float R;
    float H;
}kalman;



#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "arm_math.h"

/*************һ�׿�����**************/




/*************���׿�����**************/
#define mat arm_matrix_instance_f32    //float
#define mat_64 arm_matrix_instance_f64 //double
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32 //�������ת��
#define mat_inv arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

#define Angle_limit 200         //�Ƕ�С��50����Ԥ��
#define PredictAngle_limit 250 //Ԥ��ֵ�޷�

#define Kf_Angle 0
#define Kf_Speed 1

typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

typedef struct
{
  float Vision_Angle; //�Ӿ�--�Ƕ�
  float Vision_Speed; //�Ӿ�--�ٶ�
  float *Kf_result;   //���������ֵ
  uint16_t Kf_Delay;  //��������ʱ��ʱ

  struct
  {
    float Predicted_Factor;   //Ԥ���������
    float Predicted_SpeedMin; //Ԥ��ֵ��С�ٶ�
    float Predicted_SpeedMax; //Ԥ��ֵ����ٶ�
    float kf_delay_open;      //��������ʱ����ʱ��
  } Parameter;
} Kalman_Data_t;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
/*************���׿����� END**************/

#endif

void kalmanCreate(kalman *p,float T_Q,float T_R);
float KalmanFilter(kalman* p,float dat);


