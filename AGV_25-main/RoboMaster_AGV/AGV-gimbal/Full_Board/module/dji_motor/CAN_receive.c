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

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "shoot_task.h"
#include "Dji_motor.h"
#include "DM_motor.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

robot_shoot_date shoot_date;
robot_pos_date pos_date;
int32_t Cap_Voltage;
// motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
	
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
	
static CAN_TxHeaderTypeDef shoot_tx_message;
static uint8_t fric_can_send_data[8]={0};

static CAN_TxHeaderTypeDef pitch_tx_message;
static uint8_t pitch_tx_date[8];



void get_dm4310_mesure(uint8_t *rx_data);

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
uint8_t dm_rx_date[8];
uint8_t can1_rx_data[8];
uint8_t can2_rx_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  if (hcan == &hcan1)
  {
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, can1_rx_data);
    switch (rx_header.StdId)
    {

			case CAN_YAW_MOTOR_ID://yaw电机
			{
				get_motor_measure(&GM6020_Motor[0], can1_rx_data);
				//detect_hook(YAW_GIMBAL_MOTOR_TOE);
				break;
			}
			case CAN_TRIGGER_MOTOR_ID://拨弹电机
			{
				get_motor_measure(&M2006_Motor[0], can1_rx_data);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
//			case CAN_HEAT_ID://裁判系统信息
//			{
//				shoot_date.shoot_17mm_heat_limit = (uint16_t)(can1_rx_data[0] << 8|can1_rx_data[1]);
//				shoot_date.shoot_17mm_heat = (uint16_t)(can1_rx_data[2] << 8|can1_rx_data[1]);
//				shoot_date.shoot_17mm_cooling_value = (uint16_t)(can1_rx_data[4] << 8|can1_rx_data[5]);
//				shoot_date.bullet_speed = (fp32)(can1_rx_data[6] << 8|can1_rx_data[7]);
//				break;
//			}
//			case CAN_POS_ID://裁判系统信息
//			{
//				pos_date.x = (fp32)(can1_rx_data[0] << 8|can1_rx_data[1]);
//				pos_date.y = (fp32)(can1_rx_data[2] << 8|can1_rx_data[3]);
//				pos_date.angle = (fp32)(can1_rx_data[4] << 8|can1_rx_data[5]);
//				pos_date.robot_level = can1_rx_data[6];
//				pos_date.robot_id = can1_rx_data[7];
//				break;
//			}
			default:
			{
				break;
			}
   }
  }
  else if (hcan == &hcan2)
  {
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, can2_rx_data);
    switch (rx_header.StdId)
    {
			case 0x11://达妙电机
			{
				DM4310_Rx_Date(&DM4310_Motor,can2_rx_data);
			}	
			case CAN_3508_S1_ID://左摩擦轮电机
			{
				get_motor_measure(&M3508_Motor[0],can2_rx_data);
				detect_hook(FRIC_LEFT_MOTOR_TOE);
				break;
			}
			case CAN_3508_S2_ID://右摩擦轮电机
			{
				get_motor_measure(&M3508_Motor[1],can2_rx_data);
				detect_hook(FRIC_RIGHT_MOTOR_TOE);
				break;
			}
			default:
			{
				break;
			}
    }
  }
}

void can_cmd_gimbal_and_shoot(int16_t yaw, int16_t trigger)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_AND_TRIGGER;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = (trigger >> 8);
  gimbal_can_send_data[5] = trigger;
  gimbal_can_send_data[6] = (0 >> 8);
  gimbal_can_send_data[7] = 0;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void can_cmd_dm(uint8_t *Tx_Date,uint16_t len)
{
	uint32_t used_mailbox;
	pitch_tx_message.StdId = CAN_DM4310_TX_ID;
	pitch_tx_message.ExtId = 0;
	pitch_tx_message.IDE = 0;
	pitch_tx_message.RTR = 0;
	pitch_tx_message.DLC = len;
	
	HAL_CAN_AddTxMessage(&hcan2, &pitch_tx_message, Tx_Date, &used_mailbox);
}

void can_cmd_fric(int16_t fric1, int16_t fric2)
{
  uint32_t send_mail_box;
  shoot_tx_message.StdId = CAN_FRIC;
  shoot_tx_message.IDE = CAN_ID_STD;
  shoot_tx_message.RTR = CAN_RTR_DATA;
  shoot_tx_message.DLC = 0x08;
  fric_can_send_data[0] = (fric1 >> 8);
  fric_can_send_data[1] = fric1;
  fric_can_send_data[2] = (fric2 >> 8);
  fric_can_send_data[3] = fric2;
	fric_can_send_data[4] = 0;
	fric_can_send_data[5] = 0;
	fric_can_send_data[6] = 0;
	fric_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&hcan2, &shoot_tx_message, fric_can_send_data, &send_mail_box);
}


