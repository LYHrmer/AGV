/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "can_comm_task.h"

#define CHASSIS_CAN hcan1

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f


typedef struct
{
	uint16_t shoot_17mm_heat_limit;
	uint16_t shoot_17mm_heat;
	uint16_t shoot_17mm_cooling_value;
	fp32 bullet_speed;
}robot_shoot_date;//裁判射击信息

typedef struct
{
	fp32 x;
	fp32 y;
	fp32 angle;
	uint8_t robot_level;
	uint8_t robot_id;
}robot_pos_date;//裁判位置信息


extern int32_t cur_output;
extern robot_shoot_date shoot_date;
extern robot_pos_date pos_date;

extern void can_cmd_fric(int16_t fric1, int16_t fric2);
extern void can_cmd_gimbal_and_shoot(int16_t yaw, int16_t trigger);
extern void can_cmd_dm(uint8_t *Tx_Date,uint16_t len);



#endif
