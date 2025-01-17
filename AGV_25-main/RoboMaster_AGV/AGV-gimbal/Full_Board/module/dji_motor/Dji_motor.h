#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H
#include "struct_typedef.h"
#include "pid.h"
#include "stm32.h"
#include "math.h"
#include "arm_math.h"

#define USE_BOARD 0//0：云台  1：底盘
#define Encoder 8191
#define Encoder_Half 8191/2
//3508新减速比
#define Reduction_ratio 18
#define KT 0.01526f//N*m/A 0.3*187/3591力矩电流常数
// 电流到输出的转化系数
#define M3508_Current_To_Out (20.0f/16384.0f)
#define GM6020_Current_To_Out (3.0f/16384.0f)
#define M2006_Current_To_Out (10.0f/10000.0f)
//转换式
#define RpmToOmega(rpm) (rpm*(float)PI/30.0f)
#define OmegaToRpm(omega) (omega *30.0f /(float)PI)
#define Torque_To_Icmd(torque) (torque / KT * M3508_Current_To_Out)
#define Icmd_To_Torque(icmd) (icmd * KT /M3508_Current_To_Out)
#define Encoder_To_PI(ecd) ((ecd/4095.5f) * PI - PI)
#define M3508_RPM_TO_VECTOR  0.000415809748903494517209f
#define M2006_RMP_TO_SPEED 0.00290888208665721596153948461415f

/*****大疆电机源数据*****/
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

/*****3508解析数据*****/
typedef struct
{
	uint8_t id;
	PidTypeDef chassis_pid;
	/*******系数*******/
	fp32 k0;
	fp32 k1;
	fp32 k2;
	fp32 k3;
	/*******编码值*******/
	int encoder_round;
	fp32 get_encoder;
	/*******给定值*******/
	fp32 give_speed_set;
	fp32 give_speed;
	fp32 give_angle_set;
	fp32 give_angle;
	int16_t give_cmd_current;
	fp32 give_torque;
	fp32 give_power;
	/*******当前值*******/
  fp32 get_speed;
	fp32 get_omega;
	fp32 get_angle;
	fp32 get_torque_current;//转矩电流
	fp32 get_torque;//转子扭矩
	fp32 get_power;
	fp32 get_wheel_angle;
	
} M3508_Motor_t;

/*****6020解析数据*****/
typedef struct
{
	uint8_t id;
	//Rudder_control rudder_control;
	/*******系数*******/
	fp32 k0;
	fp32 k1;
	fp32 k2;
	fp32 k3;
	/*******编码值*******/
	int16_t encoder_round;
	fp32 ecd;
	fp32 get_encoder;
	fp32 Encoder_Offset;   //舵电机归中值
	fp32 Encoder_set;
	fp32 Encoder_add;
	fp32 last_Encoder_add;
	fp32 Encoder_error;
	fp32 Speed_Dir;
	fp32 Speed_cosk;
	/*******给定值*******/
	fp32 give_speed_set;
	fp32 give_speed;
	fp32 give_angle;
	fp32 last_give_angle;
	fp32 give_cmd_current;
	fp32 give_torque;
	fp32 give_power;
	/*******当前值*******/
	fp32 get_speed;
	fp32 get_omega;
	fp32 get_angle;
	int16_t get_torque_current;
	fp32 get_torque;
	fp32 get_power;
	
} GM6020_Motor_t;

typedef struct
{
		/*
				 /|\
		0	  	|	   1
					|
	 <------|-------			
					|
		2     |    3
	*/
	uint8_t id;
	/*******系数*******/
	fp32 k0;
	fp32 k1;
	fp32 k2;
	fp32 k3;
	/*******编码值*******/
	int16_t encoder_round;
	fp32 ecd;
	fp32 get_encoder;
	fp32 Encoder_set;

	/*******给定值*******/
	fp32 give_speed_set;
	fp32 give_speed;
	fp32 give_angle_set;
	fp32 give_angle;
	fp32 last_give_angle;
	int16_t give_cmd_current;
	fp32 give_torque;
	fp32 give_power;
	/*******当前值*******/
	fp32 get_speed;
	fp32 get_omega;
	fp32 get_angle;
	fp32 get_torque_current;
	fp32 get_torque;
	fp32 get_power;
	
} M2006_Motor_t;

extern motor_measure_t M3508_Motor[4]; 
extern motor_measure_t GM6020_Motor[4];
extern motor_measure_t M2006_Motor[2];
extern void M3508_Init(M3508_Motor_t *motor,uint8_t id);
extern void M3508_Rx_Date(M3508_Motor_t *motor);
extern void GM6020_Init(GM6020_Motor_t *motor,uint8_t id,fp32 ecd_offect);
extern void GM6020_Rx_Date(GM6020_Motor_t *motor);
extern void M2006_Init(M2006_Motor_t *motor,uint8_t id);
extern void M2006_Rx_Date(M2006_Motor_t *motor);
#endif
