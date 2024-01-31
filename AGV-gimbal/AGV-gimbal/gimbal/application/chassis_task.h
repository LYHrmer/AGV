/**
 * @file chassis_task.h
 * @author yuanluochen
 * @brief ˫�岽�����̿������񣬶�ȡң�������ݣ�ͨ��can���߷��͵�����
 * @version 0.1
 * @date 2023-09-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "referee.h"

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0
//��������ģʽͨ��
#define CHASSIS_RUN_MODE_CHANNEL 1
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 1.0f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 1.0f
//ҡ������
#define CHASSIS_RC_DEADLINE 30



//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

typedef enum
{
    CHASSIS_FOLLOW_GIMBAL_YAW = 10000,        // ���̸�����̨
    CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW = 20000, // �������̨
    CHASSIS_SPIN = 30000,                     // ����С����
    CHASSIS_NO_MOVE = 40000,                  // ���̱��ֲ���
    CHASSIS_ZERO_FORCE,               // ��������, ��û�ϵ�����
} chassis_behaviour_e;

typedef struct
{
    const RC_ctrl_t *chassis_RC;             // ����ʹ�õ�ң����ָ��
    const gimbal_motor_t *chassis_yaw_motor; // ����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
    chassis_behaviour_e chassis_behaviour;
    fp32 vx_set;                 // �����趨�ٶ� ǰ������ ǰΪ������λ m/s
    fp32 vy_set;                 // �����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
    fp32 chassis_relative_ecd; // ��������̨����ԽǶȣ���λ rad
} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

#endif
