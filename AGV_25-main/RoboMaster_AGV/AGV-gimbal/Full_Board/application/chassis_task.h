/**
 * @file chassis_task.h
 * @author yuanluochen
 * @brief 双板步兵底盘控制任务，读取遥控器数据，通过can总线发送到底盘
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


// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

// 前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
// 左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
// 选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
// 底盘运行模式通道
#define CHASSIS_RUN_MODE_CHANNEL 1
// 遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 1.0f
// 遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 1.0f
// 摇杆死区
#define CHASSIS_RC_DEADLINE 30
//开启超电时的放电大小参数
#define CAP_OUTPUT_to_CHASSIS 8000
#define CAP_OUTPUT_to_CHASSIS_FLY 7000
// 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
// 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

// 底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

typedef enum
{
  CHASSIS_FOLLOW_GIMBAL_YAW = 10000,        // 底盘跟随云台
  CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW = 20000, // 舵跟随云台
  CHASSIS_SPIN = 30000,                     // 底盘小陀螺
  CHASSIS_NO_MOVE = 40000,                  // 底盘保持不动
  CHASSIS_ZERO_FORCE,                       // 底盘无力, 跟没上电那样
} chassis_behaviour_e;

typedef enum
{
  RC_Control = 0,
  RUDDER_FOLLOW_GIMBAL,
  CHASSIS_FOLLOW_GIMBAL,
  CHASSIS_SPIIN,
} Chassis_mode_e;

typedef struct
{
  const RC_ctrl_t *chassis_RC;             // 底盘使用的遥控器指针
  const gimbal_motor_t *chassis_yaw_motor; // 底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
  chassis_behaviour_e chassis_behaviour;
  Chassis_mode_e Chasiss_mode;
  fp32 vx_set;               // 底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;               // 底盘设定速度 左右方向 左为正，单位 m/s
  int16_t yaw_relative_ecd; // 底盘与云台的相对角度，单位 rad
} chassis_move_t;

/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

#endif
