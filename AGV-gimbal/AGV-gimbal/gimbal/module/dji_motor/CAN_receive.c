/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                 \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*

�������, CAN1 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 
		CAN2 0:Ħ���ֵ��1 3508���,   1:Ħ���ֵ��2 3508����� 3:�����ֵ�� 2006���*/
static motor_measure_t motor_chassis[7];
static motor_measure_t motor_shoot[2];
static motor_measure_t motor_tri;
		
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

static CAN_TxHeaderTypeDef  gimbal_callx_message;
static uint8_t              gimbal_can_call_data[8];
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        case CAN_YAW_MOTOR_ID:
        {

            get_motor_measure(&motor_chassis[4], rx_data);
            detect_hook(YAW_GIMBAL_MOTOR_TOE);
            break;
        }
        case CAN_PIT_MOTOR_ID:
        {
            get_motor_measure(&motor_chassis[5], rx_data);
            detect_hook(PITCH_GIMBAL_MOTOR_TOE);
            break;
        }
        default:
        {
            break;
        }
        }
    }
    else if (hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        case CAN_TRIGGER_MOTOR_ID:
        {
            get_motor_measure(&motor_tri, rx_data);
            detect_hook(TRIGGER_MOTOR_TOE);
            break;
        }
        case CAN_3508_S1_ID:
        {
            get_motor_measure(&motor_shoot[rx_header.StdId - CAN_3508_S1_ID], rx_data);
            detect_hook(FRIC_LEFT_MOTOR_TOE);
            break;
        }
        case CAN_3508_S2_ID:
        {
            get_motor_measure(&motor_shoot[rx_header.StdId - CAN_3508_S1_ID], rx_data);
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

//void CAN_cmd_shoot(int16_t fric1,int16_t fric2 ,int16_t shoot)
//{
//   uint32_t send_mail_box;
//   shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
//   shoot_tx_message.IDE = CAN_ID_STD;
//   shoot_tx_message.RTR = CAN_RTR_DATA;
//   shoot_tx_message.DLC = 0x08;
//   shoot_can_send_data[0] = (fric1 >> 8);
//   shoot_can_send_data[1] = fric1;
//   shoot_can_send_data[2] = (fric2 >> 8);
//   shoot_can_send_data[3] = fric2;
//   shoot_can_send_data[4] = (shoot >> 8);
//   shoot_can_send_data[5] = shoot;
//   shoot_can_send_data[6] = (0 >> 8);
//   shoot_can_send_data[7] = 0;
//   HAL_CAN_AddTxMessage(&hcan2, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
//}

void CAN_cmd_shoot(int16_t fric1,int16_t fric2 ,int16_t shoot, int16_t rev)
{
	  uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = (fric1 >> 8);
    shoot_can_send_data[1] = fric1;
    shoot_can_send_data[2] = (fric2 >> 8);
    shoot_can_send_data[3] = fric2;
    shoot_can_send_data[4] = (shoot >> 8);
    shoot_can_send_data[5] = shoot;
    shoot_can_send_data[6] = (rev >> 8);
    shoot_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage( &hcan2,  &shoot_tx_message, shoot_can_send_data, &send_mail_box);

}


/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (0 >> 8);
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = (0 >> 8);
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief ���͵��̿�������
 *
 * @param[in]      relative_angle: ��̨��Խ�
 * @param[in]      chassis_vx: ����x�᷽���ٶȷ���
 * @param[in]      chassis_vy: ����y�᷽���ٶȷ���
 * @param[in]      chassis_behaviour: �����˶�ģʽ
 * @retval         none
 */
void CAN_cmd_chassis(int16_t relative_angle, int32_t chassis_vx, int32_t chassis_vy, int16_t chassis_behaviour)
{
    uint32_t send_mail_box;
    gimbal_callx_message.StdId = CAN_GIMBAL_CONTROL_CHASSIS_ID;
    gimbal_callx_message.IDE = CAN_ID_STD;
    gimbal_callx_message.RTR = CAN_RTR_DATA;
    gimbal_callx_message.DLC = 0x08;
    gimbal_can_call_data[0] = (relative_angle >> 8);
    gimbal_can_call_data[1] = relative_angle;
    gimbal_can_call_data[2] = (chassis_vx >> 8);
    gimbal_can_call_data[3] = chassis_vx;
    gimbal_can_call_data[4] = (chassis_vy >> 8);
    gimbal_can_call_data[5] = chassis_vy;
    gimbal_can_call_data[6] = (chassis_behaviour >> 8);
    gimbal_can_call_data[7] = chassis_behaviour;
    HAL_CAN_AddTxMessage( &hcan1,  &gimbal_callx_message, gimbal_can_call_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

const motor_measure_t *get_shoot_motor_measure_point(uint8_t j)
{
    return &motor_shoot[(j & 0x01)];
}
/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_tri;
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
