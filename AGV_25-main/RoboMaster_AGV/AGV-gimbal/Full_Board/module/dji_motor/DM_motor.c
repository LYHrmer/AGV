#include "cmsis_os.h"
#include "DM_motor.h"
#include "CAN_receive.h"
#include "can_comm_task.h"

DM4310_Motor_t DM4310_Motor;

uint8_t DM_Motor_CAN_Message_Enable[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};

uint8_t DM_Motor_CAN_Message_Disable[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};

	/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/****************************************
*函 数 名: DM4310_Enbale
*功能说明: 4310电机使能
*形    参: 无
*返 回 值: 无
*****************************************/
void DM4310_Enbale(void)
{
	can_cmd_dm(DM_Motor_CAN_Message_Enable,8);
}

/****************************************
*函 数 名: DM4310_Disable
*功能说明: 4310电机失能
*形    参: 无
*返 回 值: 无
*****************************************/
void DM4310_Disable(void)
{
	can_cmd_dm(DM_Motor_CAN_Message_Disable,8);
}

/****************************************
*函 数 名: DM4310_Init
*功能说明: 4310电机初始化
*形    参: id 0-2为电机分配一个id
*返 回 值: 无
*****************************************/
void DM4310_Init(DM4310_Motor_t *motor,uint8_t id,DM4310_Mode mode,fp32 encoder_offset)
{
	DM4310_Enbale();
	motor->id = id;
	motor->mode = mode;
	motor->Encoder_Offset = encoder_offset;
	motor->give_torque_current = 0.0f;
	motor->give_vel = 0.0f;
	motor->give_kp = 40.0f;
	motor->give_kd = 0.8f;
}
/****************************************
*函 数 名: DM4310_Rx_Date
*功能说明: 4310电机源数据处理
*形    参: 无
*返 回 值: 无
*****************************************/
void DM4310_Rx_Date(DM4310_Motor_t *motor,uint8_t *rx_date)
{
	motor->dm_motor.id = (rx_date[0])&0x0F;
	motor->dm_motor.state = (rx_date[0])>>4;
	motor->dm_motor.p_int=(rx_date[1]<<8)|rx_date[2];
	motor->dm_motor.v_int=(rx_date[3]<<4)|(rx_date[4]>>4);
	motor->dm_motor.t_int=((rx_date[4]&0xF)<<8)|rx_date[5];
	motor->dm_motor.pos = uint_to_float(motor->dm_motor.p_int, P_MIN, P_MAX, 16); // (-12.0,12.0)
	motor->dm_motor.vel = uint_to_float(motor->dm_motor.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	motor->dm_motor.tor = uint_to_float(motor->dm_motor.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	motor->dm_motor.Tmos = (float)(rx_date[6]);
	motor->dm_motor.Tcoil = (float)(rx_date[7]);
}

/****************************************
*函 数 名: DM4310_Tx_Date
*功能说明: 4310电机控制数据处理
*形    参: 无
*返 回 值: 无
*****************************************/
void DM4310_Tx_Date(DM4310_Motor_t *motor)
{
	uint8_t Tx_Date[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint8_t *pbuf,*vbuf;

	pos_tmp = float_to_uint(motor->give_pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(motor->give_vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(motor->give_kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(motor->give_kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(motor->give_torque, T_MIN,  T_MAX,  12);
	switch(motor->mode)
	{
		case MIT://MIT模式
		{
			Tx_Date[0] = (pos_tmp >> 8);
			Tx_Date[1] = pos_tmp;
			Tx_Date[2] = (vel_tmp >> 4);
			Tx_Date[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
			Tx_Date[4] = kp_tmp;
			Tx_Date[5] = (kd_tmp >> 4);
			Tx_Date[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
			Tx_Date[7] = tor_tmp;
	
			can_cmd_dm(Tx_Date,8);
			break;
		}
		case ANGLE_OMEGA://位置速度模式
		{
			pbuf = (uint8_t*)&motor->give_pos;
			vbuf = (uint8_t*)&motor->give_vel;
			
			Tx_Date[0] = *pbuf;
			Tx_Date[1] = *(pbuf+1);
			Tx_Date[2] = *(pbuf+2);
			Tx_Date[3] = *(pbuf+3);
			Tx_Date[4] = *vbuf;
			Tx_Date[5] = *(vbuf+1);
			Tx_Date[6] = *(vbuf+2);
			Tx_Date[7] = *(vbuf+3);
			can_cmd_dm(Tx_Date,8);
			break;
		}
		case OMEGA:
		{
			//vbuf =(uint8_t*)&motor->give_vel;
			break;
		}
	}
}
