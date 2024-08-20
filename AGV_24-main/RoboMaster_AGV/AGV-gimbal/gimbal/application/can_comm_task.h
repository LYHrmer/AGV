/**
 * @file can_comm_task.h
 * @author yuanluochen
 * @brief can设备通信任务，利用队列实现can数据队列发送
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

//can通信任务初始化时间 单位ms
#define CAN_COMM_TASK_INIT_TIME 100
//can通信任务运行时间间隔 单位ms
#define CAN_COMM_TASK_TIME 1
//云台can设备
#define GIMBAL_CAN hcan1
//双板can通信设备
#define BOARD_CAN hcan1
//发弹can通信设备
#define SHOOT_CAN hcan2
//裁判系统can通信
#define REFEREE_CAN hcan1
//发送云台pitch轴的相对角和绝对角can通信
#define PitchAngle_CAN hcan1

#define CAN_PITCHANGLE_ID  0x213
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x204,
    CAN_3508_M2_ID = 0x203,
    CAN_3508_M3_ID = 0x202,
    CAN_3508_M4_ID = 0x201,
    CAN_GIMBAL_CONTROL_CHASSIS_ID = 0x218,
		CAN_REFEREE_ID = 0x219,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    CAN_SHOOT_ALL_ID = 0x1FF,
    CAN_3508_S1_ID = 0x205,
    CAN_3508_S2_ID = 0x206,

    cap_voltage_ID = 0x221,
} can_msg_id_e;

//can通信任务结构体
typedef struct
{
    //can通信队列结构体
    can_comm_queue_t *can_comm_queue;
    
}can_comm_task_t;


/**
 * @brief  can通信任务
 * 
 */
void can_comm_task(void const* pvParameters);

/**
 * @brief 云台控制数据发送，发送值为电机电流值，发送到can_comm线程的通信队列中
 * 
 * @param yaw (0x205) 6020电机控制电流，范围[-30000, 30000]
 * @param pitch (0x206) 6020电机控制电流， 范围[-30000, 30000]
 */
void can_comm_gimbal(int16_t yaw, int16_t pitch);

/**
 * @brief 双板通信数据发送，云台控制底盘，将数据添加到can_comm线程通信队列中
 * 
 * @param relative_angle 云台相对角
 * @param chassis_vx 底盘x轴速度方向分量
 * @param chassis_vy 底盘y轴速度方向分量
 * @param chassis_behaviour 底盘运动模式
 */
void can_comm_board(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour);

/**
 * @brief 发弹电机通信数据发送，发送值为电机电流值，将发送数据添加到can_comm线程的通信队列中
 * 
 * @param fric1 摩擦轮电机电流值
 * @param fric2 摩擦轮电机电流值
 * @param trigger 拨弹盘电机电流值
 */
void can_comm_shoot(int16_t fric1, int16_t fric2, int16_t trigger);

void can_comm_referee(int16_t key_1,int32_t key_2, int32_t key_3, int16_t key_other );

//void can_comm_pitchangle(int16_t pitch_relative,int32_t pitch_absolute, int32_t key_3, int16_t key_other );

void CAN_CMD_cap(int16_t pitch_relative,int16_t pitch_absolute,int16_t kay_ctrl);

bool can_comm_task_init_finish(void);

#endif // !CAN_COMM_TASK_H
