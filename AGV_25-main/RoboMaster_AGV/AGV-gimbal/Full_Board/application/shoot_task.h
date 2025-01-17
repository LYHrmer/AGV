#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "Dji_motor.h"

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

#define FRIC_SPEED 3.95f
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
#define LOW_SPEED 20.0f//15.0f //-12.0f
#define MID_SPEED 27.0f//12.0f //-12.0f
#define HIGH_SPEED 30.0f//10.0f //-12.0f
#define HIGHER_SPEED 32.0f

#define LOW_FRIC_SPEED 2.25f
#define MID_FRIC_SPEED 2.55f
#define HIGH_FRIC_SPEED 2.75f
#define HIGHER_FRIC_SPEED 2.95f
#define FRIC_MAX_SPEED 3.00f
#define FRIC_MIN_SPEED 2.25f
//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f

#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define FULL_COUNT 18
#define FULL_CNT 45

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 900///2450.0f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 100.0f

#define TRIGGER_READY_PID_MAX_OUT 9000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define Shoot_Encoder_Half 8192/2 //395  //395  7796
#define Shoot_Encoder 8192

//3508电机速度环PID
#define S3505_MOTOR_SPEED_PID_KP 20.f
#define S3505_MOTOR_SPEED_PID_KI  4.0f
#define S3505_MOTOR_SPEED_PID_KD  0.0f
#define S3505_MOTOR_SPEED_PID_MAX_OUT 16300.0f
#define S3505_MOTOR_SPEED_PID_MAX_IOUT 10000.0f

#define PI_Two 1.5707963
#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f

#define RATIO 5/2
#define TRIGGER_SPEED 3.0f
#define SWITCH_TRIGGER_ON 0

#define C620_Current_To_Out (16384.0f / 20.0f)// 电流控制值->输出的力矩电流转化系数
#define KT 0.01526f//N*m/A 0.3*187/3591力矩电流常数
#define EncoderToPI(ecd) ((ecd /8191 * 2 * PI) - PI)
#define BulletHeat17 10
typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,	//发弹准备
		SHOOT_AUTO,		//自动发弹
    SHOOT_BULLET,	//连发
	  SHOOT_TOTOTO,	//极速连发
		SHOOT_ONE,		//单发
		SHOOT_STUCK		//卡弹回拨
} shoot_mode_e;


typedef struct
{
	pid_type_def trigger_motor_pid;
	pid_type_def trigger_angle_pid;
  pid_type_def trigger_motor_a_pid;
	
  M2006_Motor_t trigger;
	fp32 give_rammer_angle;
	fp32 get_rammer_angle;//拨弹盘角度

	int16_t bullet_round;//弹丸发射量计数
	uint16_t sumHeat;//总计热量累计
} Trigger_Motor_t;

typedef struct
{
  M3508_Motor_t fric;
} Fric_Motor_t;


typedef struct
{
	bool_t press_l;
	bool_t press_r;
	bool_t last_press_l;
  bool_t last_press_r;
	
	bool_t G;
	bool_t last_G;
	bool_t R;
	bool_t last_R;
	bool_t rc_key_flag;
	
}Shoot_Rc;
typedef struct
{
	const RC_ctrl_t *shoot_rc;              //遥控器指针
	Shoot_Rc shoot_key;											//DT7+键盘
	vision_control_t vision_control;				//视觉数据						
  shoot_mode_e shoot_mode;               	//射击控制状态机
	shoot_mode_e last_shoot_mode;         	//射击上次控制状态机
	Fric_Motor_t  fric_motor[2];          		//摩擦轮电机数据
	Trigger_Motor_t trigger_motor;					//拨弹电机数据
  first_order_filter_type_t fric_cmd_slow_set_speed[2];  // 滤波数据
	fp32 min_speed;
	fp32 max_speed;
	
	fp32 HeatLimit17;
	fp32 HeatCool17;
	fp32 CurHeat17;
	fp32 lastHeat17;
	int move_flag;
	struct{
		bool_t shootAble;//根据热量判断是否可以发弹
		bool_t rc_flag;//遥控器or键盘标志
		bool_t ready_flag;
		bool_t fric_ON;
		bool_t trigger_ON;
		bool_t trigger_toto;
		bool_t cool_buff;//是否触发增益
	}shoot_flags;
	struct {
		bool_t HeatUpDate;
	}heat_flags;
} Ammo;
extern Ammo Ammo_booster;;
extern void shoot_task(void const *pvParameters);
#endif
