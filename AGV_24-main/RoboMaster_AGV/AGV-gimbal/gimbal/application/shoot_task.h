#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


// 射击电流控制
#define COOL_FIRST_SPEED_LIMIT_5_MAX_CONTROL_SPEED 2000
#define COOL_FIRST_SPEED_LIMIT_ELSE_MAX_CONTROL_SPEED 2000

//允许发弹角度误差 rad
// #define ALLOW_ATTACK_ERROR 0.04f//0.04f
//射速15
#define SHOOT_SPEED_15 2.0f
//射速18
#define SHOOT_SPEED_18 2.23f
//射速30
#define SHOOT_SPEED_30 3.25f

//冷却优先15弹速
#define COOL_FRIST_15_FRIC_SPEED SHOOT_SPEED_15
//冷却优先18弹速
#define COOL_FRIST_18_FRIC_SPEED SHOOT_SPEED_18

//1V1 18弹速
#define FRIT_1V1_18_FRIC_SPEED SHOOT_SPEED_18

//射速优先弹速30摩擦轮弹速
#define SPEED_FIRST_30_FRIC_SPEED SHOOT_SPEED_30;




//射击发射开关通道数据
#define Shoot_RC_Channel    1
//云台模式使用的开关通道
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  0.002

//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000

//鼠标长按判断
#define PRESS_LONG_TIME 400
//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E


//拨弹速度
#define MAX_SPEED 10.0f//15.0f //-12.0f
#define MID_SPEED 7.0f//12.0f //-12.0f
#define MIN_SPEED 5.0f//10.0f //-12.0f
#define Ready_Trigger_Speed 6.0f

//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 900///2450.0f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 100.0f

#define TRIGGER_READY_PID_MAX_OUT 8000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define Half_ecd_range 395  //395  7796
#define ecd_range 8191
//3508电机速度环PID
#define S3505_MOTOR_SPEED_PID_KP 20.f
#define S3505_MOTOR_SPEED_PID_KI  4.0f
#define S3505_MOTOR_SPEED_PID_KD  0.0f
#define S3505_MOTOR_SPEED_PID_MAX_OUT 16300.0f
#define S3505_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f

#define TRIGGER_SPEED 3.0f
#define SWITCH_TRIGGER_ON 0

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
	SHOOT_BULLET_ONE,
	SHOOT_DONE,
	SHOOT_AUTO,
} shoot_mode_e;

typedef struct
{
	const motor_measure_t *shoot_motor_measure;
	fp32 speed;
	fp32 speed_set;
	fp32 angle;
	int8_t ecd_count;
	fp32 set_angle;
  int16_t given_current;
	
	bool_t press_l;
	bool_t press_r;
	bool_t last_press_l;
	bool_t last_press_r;
	uint16_t press_l_time;
	uint16_t press_r_time;
	uint16_t rc_s_time;
	
	uint32_t run_time;
	uint32_t cmd_time;
	int16_t move_flag;
	int16_t move_flag_ONE;
	bool_t key;
	int16_t BulletShootCnt;
	int16_t last_butter_count;
  fp32 shoot_CAN_Set_Current;
    fp32  blocking_angle_set;
	fp32  blocking_angle_current;
	int8_t blocking_ecd_count;
} Shoot_Motor_t;

typedef struct
{
  const motor_measure_t *fric_motor_measure;
	fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	uint16_t rc_key_time;
} fric_Motor_t;

typedef struct
{
	const RC_ctrl_t *shoot_rc;                   //遥控器指针
  shoot_mode_e fric_mode;               //射击控制状态机
	shoot_mode_e last_fric_mode;          //射击上次控制状态机
	fric_Motor_t motor_fric[2];          //射击电机数据
	fp32 fric_CAN_Set_Current[2];
	PidTypeDef motor_speed_pid[4];             //射击电机速度pid
	
  first_order_filter_type_t fric1_cmd_slow_set_speed;  // 滤波数据
  first_order_filter_type_t fric2_cmd_slow_set_speed;  // 滤波数据
	fp32 angle[2];
	int16_t ecd_count[2];
  int16_t given_current[2];
  fp32 set_angle[2];
	fp32 speed[2];
	fp32 speed_set[2];	
	fp32 current_set[2];	
	bool_t move_flag;

	fp32 min_speed;
	fp32 max_speed;
	int flag[2];
	int laster_add;
} fric_move_t;

extern void shoot_init(void);
extern void shoot_control_loop(void);
extern void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance);
void shoot_task(void const *pvParameters);
#endif
