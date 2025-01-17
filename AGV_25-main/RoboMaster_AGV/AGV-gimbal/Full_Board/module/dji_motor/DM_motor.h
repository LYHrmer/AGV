#ifndef DM_MOTOR_H
#define DM_MOTOR_H
#include "struct_typedef.h"

//��������ʼ������
//#define Angle_Max 1.0f
#define Angle_Limit 12.5f
#define Omega_Limit 25.0f
#define Torque_Limit 10.0f
#define Current_Limit 10.0f

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200
typedef enum
{
	Disable = 0,
	Enable,
}DM_state;
typedef enum
{
	MIT,
	ANGLE_OMEGA,
	OMEGA,
}DM4310_Mode;
typedef struct
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}dm_measure_t;
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
	dm_measure_t dm_motor;
	uint8_t id;
	DM4310_Mode mode;

	/*******ϵ��*******/
	fp32 k0;
	fp32 k1;
	fp32 k2;
	fp32 k3;
	/*******����ֵ*******/
	int16_t encoder_round;
	fp32 Encoder_Offset;   //�������ֵ
	fp32 ecd;
	fp32 last_ecd;
	fp32 get_encoder;
	/*******����ֵ*******/
	fp32 give_kp;
	fp32 give_kd;
	fp32 give_vel;
	fp32 give_omega;
	fp32 give_pos;
	fp32 last_give_pos;
	fp32 give_torque_current;
	fp32 give_torque;
	fp32 give_power;
	/*******��ǰֵ*******/
	fp32 get_speed;
	fp32 get_omega;
	fp32 get_angle;
	fp32 get_torque_current;
	fp32 get_torque;
	fp32 get_power;
	/*******���MITģʽ��������*******/

} DM4310_Motor_t;


extern DM4310_Motor_t DM4310_Motor;


extern void DM4310_Enbale(void);
extern void DM4310_Disable(void);
extern void DM4310_Init(DM4310_Motor_t *motor,uint8_t id,DM4310_Mode mode,fp32 encoder_offset);
extern void DM4310_Rx_Date(DM4310_Motor_t *motor,uint8_t *rx_date);
extern void DM4310_Tx_Date(DM4310_Motor_t *motor);
#endif
