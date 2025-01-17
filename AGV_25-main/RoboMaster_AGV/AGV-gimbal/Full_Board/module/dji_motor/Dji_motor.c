#include "Dji_motor.h"

motor_measure_t M3508_Motor[4]; 
motor_measure_t GM6020_Motor[4];
motor_measure_t M2006_Motor[2];
/****************************************
*函 数 名: power_forecast
*功能说明: 功率模型计算
*形    参: K_0 系数k0
					 K_1 系数k1
					 K_2 系数k2
					 A 静态损耗
					 Current 转子转矩电流
					 Omega 转子角速度
*返 回 值: 模型计算所得功率
*****************************************/
static float power_forecast(fp32 K_0, fp32 K_1, fp32 K_2, fp32 A, fp32 Current, fp32 Omega)
{
	return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
}
/****************************************
*函 数 名: M3508_Init
*功能说明: 3508电机初始化
*形    参: id 0-4为电机分配一个id:0-4
*返 回 值: 无
*****************************************/
void M3508_Init(M3508_Motor_t *motor,uint8_t id)
{
	motor->id = id;
	motor->k0 = 0.2962f;//0.0758f;
	motor->k1 = 0.0000f;//0.0098f;
	motor->k2 = 0.1519f;//-0.0069f;
	motor->k3 = 1.3544;//1.4175f;
//	motor->k0 = 0.0758f;
//	motor->k1 = 0.0098f;
//	motor->k2 = -0.0069f;
//	motor->k3 = 1.4175f;
}
/****************************************
*函 数 名: M3508_Rx_Date
*功能说明: 3508电机源数据处理
*形    参: 无
*返 回 值: 无
*****************************************/
void M3508_Rx_Date(M3508_Motor_t *motor)
{
	int32_t delta_encoder;
	delta_encoder = M3508_Motor[motor->id].ecd - M3508_Motor[motor->id].last_ecd;
//	if(delta_encoder < -Encoder_Half)
//	{
//		motor->encoder_round++;
//	}
//	else if(delta_encoder > Encoder_Half)
//	{
//		motor->encoder_round--;
//	}
//	
	motor->get_encoder = motor->encoder_round * Encoder + M3508_Motor[motor->id].ecd;
	motor->get_angle = Encoder_To_PI(motor->get_encoder);
  motor->get_speed = M3508_RPM_TO_VECTOR * M3508_Motor[motor->id].speed_rpm;
	motor->get_omega = RpmToOmega(M3508_Motor[motor->id].speed_rpm);
	motor->get_torque_current = M3508_Motor[motor->id].given_current * M3508_Current_To_Out;
	motor->get_torque = Icmd_To_Torque(M3508_Motor[motor->id].given_current);
	motor->get_power = power_forecast(motor->k0,motor->k1,motor->k2,motor->k3,motor->get_torque_current,motor->get_omega);
}
/****************************************
*函 数 名: GM6020_Init
*功能说明: 6020电机初始化
*形    参: id 0-4为电机分配一个id
*返 回 值: 无
*****************************************/
void GM6020_Init(GM6020_Motor_t *motor,uint8_t id,fp32 ecd_offect)
{
	motor->id = id;
	motor->Encoder_Offset = ecd_offect;
	motor->k0 = 0.8130f;
	motor->k1 = -0.0005f;
	motor->k2 = 6.0021f;
	motor->k3 = 1.3715f;
}
/****************************************
*函 数 名: GM6020_Rx_Date
*功能说明: 6020电机源数据处理
*形    参: 无
*返 回 值: 无
*****************************************/
void GM6020_Rx_Date(GM6020_Motor_t *motor)
{
	int32_t delta_encoder;
	delta_encoder = GM6020_Motor[motor->id].ecd - GM6020_Motor[motor->id].last_ecd;
//	if(delta_encoder < -Encoder_Half) 
//	{
//		motor->encoder_round++;
//	}
//	else if(delta_encoder > Encoder_Half)
//	{
//		motor->encoder_round--;
//	}
	motor->ecd = GM6020_Motor[motor->id].ecd;

	motor->get_angle = Encoder_To_PI(motor->get_encoder);
	
	motor->get_omega = RpmToOmega(GM6020_Motor[motor->id].speed_rpm);
	motor->get_torque_current = GM6020_Motor[motor->id].given_current * GM6020_Current_To_Out;
	motor->get_torque = Icmd_To_Torque(GM6020_Motor[motor->id].given_current);
	motor->get_power = power_forecast(motor->k0,motor->k1,motor->k2,motor->k3,motor->get_torque_current,motor->get_omega);
}
/****************************************
*函 数 名: M2006_Init
*功能说明: 2006电机初始化
*形    参: id 0-为电机分配一个id
*返 回 值: 无
*****************************************/
void M2006_Init(M2006_Motor_t *motor,uint8_t id)
{
	motor->id = id;
	motor->k0 = 0.8130f;
	motor->k1 = -0.0005f;
	motor->k2 = 6.0021f;
	motor->k3 = 1.3715f;
}
/****************************************
*函 数 名: M2006_Rx_Date
*功能说明: 2006电机源数据处理
*形    参: 无
*返 回 值: 无
*****************************************/
void M2006_Rx_Date(M2006_Motor_t *motor)
{
	int16_t delta_encoder;
	delta_encoder = GM6020_Motor[motor->id].ecd - GM6020_Motor[motor->id].last_ecd;
	if(delta_encoder < -Encoder_Half) 
	{
		motor->encoder_round++;
	}
	else if(delta_encoder > Encoder_Half)
	{
		motor->encoder_round--;
	}
	motor->ecd = M2006_Motor[motor->id].ecd;
	//motor->get_encoder = motor->encoder_round * Encoder + M2006_Motor[motor->id].ecd;
	motor->get_angle = Encoder_To_PI(motor->get_encoder);
	motor->get_speed = M2006_RMP_TO_SPEED * M2006_Motor[motor->id].speed_rpm;
	motor->get_omega = RpmToOmega(M2006_Motor[motor->id].speed_rpm);
	motor->get_torque_current = M2006_Motor[motor->id].given_current * M2006_Current_To_Out;
	motor->get_torque = Icmd_To_Torque(M2006_Motor[motor->id].given_current);
	motor->get_power = power_forecast(motor->k0,motor->k1,motor->k2,motor->k3,motor->get_torque_current,motor->get_omega);
}

