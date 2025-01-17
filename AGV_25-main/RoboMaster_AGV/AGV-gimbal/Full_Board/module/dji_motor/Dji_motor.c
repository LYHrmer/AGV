#include "Dji_motor.h"

motor_measure_t M3508_Motor[4]; 
motor_measure_t GM6020_Motor[4];
motor_measure_t M2006_Motor[2];
/****************************************
*�� �� ��: power_forecast
*����˵��: ����ģ�ͼ���
*��    ��: K_0 ϵ��k0
					 K_1 ϵ��k1
					 K_2 ϵ��k2
					 A ��̬���
					 Current ת��ת�ص���
					 Omega ת�ӽ��ٶ�
*�� �� ֵ: ģ�ͼ������ù���
*****************************************/
static float power_forecast(fp32 K_0, fp32 K_1, fp32 K_2, fp32 A, fp32 Current, fp32 Omega)
{
	return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
}
/****************************************
*�� �� ��: M3508_Init
*����˵��: 3508�����ʼ��
*��    ��: id 0-4Ϊ�������һ��id:0-4
*�� �� ֵ: ��
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
*�� �� ��: M3508_Rx_Date
*����˵��: 3508���Դ���ݴ���
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: GM6020_Init
*����˵��: 6020�����ʼ��
*��    ��: id 0-4Ϊ�������һ��id
*�� �� ֵ: ��
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
*�� �� ��: GM6020_Rx_Date
*����˵��: 6020���Դ���ݴ���
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: M2006_Init
*����˵��: 2006�����ʼ��
*��    ��: id 0-Ϊ�������һ��id
*�� �� ֵ: ��
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
*�� �� ��: M2006_Rx_Date
*����˵��: 2006���Դ���ݴ���
*��    ��: ��
*�� �� ֵ: ��
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

