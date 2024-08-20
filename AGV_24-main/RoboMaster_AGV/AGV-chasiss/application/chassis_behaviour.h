/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "main.h"
#include "chassis_task.h"

#define CHASSIS_OPEN_RC_SCALE 10 //在 chassis_open 模型下，遥控器乘以该比例发送到can上

typedef enum
{
  CHASSIS_ZERO_FORCE,                  //底盘无力
  CHASSIS_NO_MOVE,                     //底盘保持不动
  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //正常步兵底盘跟随云台
  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, //工程底盘角度控制底盘
  CHASSIS_NO_FOLLOW_YAW,               //底盘不跟随角度
	CHASSIS_INFANTRY_SPIN,               //小陀螺
  CHASSIS_OPEN,                         //遥控器的值乘以比例直接发送到can总线上
		RUDDER_INFANTRY_FOLLOW_GIMBAL_YAW    //舵跟随云台模式 
} chassis_behaviour_e;

extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
