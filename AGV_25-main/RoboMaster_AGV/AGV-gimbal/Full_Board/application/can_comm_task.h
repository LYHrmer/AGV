/**
 * @file can_comm_task.h
 * @author yuanluochen
 * @brief can�豸ͨ���������ö���ʵ��can���ݶ��з���
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H 
#include "can_comm.h"
#include "can.h"

//canͨ�������ʼ��ʱ�� ��λms
#define CAN_COMM_TASK_INIT_TIME 100
//canͨ����������ʱ���� ��λms
#define CAN_COMM_TASK_TIME 1
//��̨can�豸
#define GIMBAL_CAN hcan1
//˫��canͨ���豸
#define BOARD_CAN hcan1
//����canͨ���豸
#define SHOOT_CAN hcan2
//����ϵͳcanͨ��
#define SHOOT_FLAGS_CAN hcan1
//������̨pitch�����ԽǺ;��Խ�canͨ��
#define PitchAngle_CAN hcan1

typedef enum
{
	CAN_GIMBAL_AND_TRIGGER = 0x1FF,
	CAN_FRIC = 0x1FF,
	CAN_GIMBAL_CONTROL_CHASSIS_ID = 0x218,
	CAN_DM4310_TX_ID = 0x01
}can_give_frame;
typedef enum
{
		
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
	
    CAN_TRIGGER_MOTOR_ID = 0x207,

		CAN_SHOOT_FLAGS_ID = 0x219,

    CAN_3508_S1_ID = 0x205,
    CAN_3508_S2_ID = 0x206,

} can_get_frame;

//canͨ������ṹ��
typedef struct
{
    //canͨ�Ŷ��нṹ��
    can_comm_queue_t *can_comm_queue;
    
}can_comm_task_t;


extern void can_comm_task(void const* pvParameters);

extern void can_comm_board(int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour, int16_t cap_key);

//extern void can_comm_shoot_flags(uint16_t fric0,uint16_t fric1,uint16_t trigger,uint16_t bullet_round);

extern bool can_comm_task_init_finish(void);

#endif // !CAN_COMM_TASK_H
