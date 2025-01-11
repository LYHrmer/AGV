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
#include "can.h"
#include "motor_dm.h"
#include "gimbal_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

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
	
Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

// CANͨ�ŷ��ͻ�����
uint8_t CAN1_0x1fe_Tx_Data[8];
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2fe_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

uint8_t CAN2_0x1fe_Tx_Data[8];
uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2fe_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];
/*

�������, CAN1 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���;
    CAN2 0:Ħ���ֵ��1 3508���,   1:Ħ���ֵ��2 3508����� 3:�����ֵ�� 2006���*/
static motor_measure_t motor_chassis[7];
static motor_measure_t motor_shoot[2];
static motor_measure_t motor_tri;


static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static CAN_TxHeaderTypeDef shoot_tx_message;
static uint8_t shoot_can_send_data[8];

static CAN_TxHeaderTypeDef gimbal_callx_message;
static uint8_t gimbal_can_call_data[8];

/**
 * @brief ����CAN�Ĺ�����
 *
 * @param hcan CAN���
 * @param Object_Para ��� | FIFOx | ID���� | ֡����
 * @param ID ID
 * @param Mask_ID ����λ(0x3ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    //��⴫���Ƿ���ȷ
    assert_param(hcan != NULL);

    if ((Object_Para & 0x02))
    {
        //����֡
        //�����ID�ĸ�16bit
        can_filter_init_structure.FilterIdHigh = ID << 3 << 16;
        //�����ID�ĵ�16bit
        can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
        // ID����ֵ��16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
        // ID����ֵ��16bit
        can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
    }
    else
    {
        //����֡
        //�����ID�ĸ�16bit
        can_filter_init_structure.FilterIdHigh = ID << 5;
        //�����ID�ĵ�16bit
        can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
        // ID����ֵ��16bit
        can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
        // ID����ֵ��16bit
        can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
    }
    //�˲������, 0-27, ��28���˲���, ǰ14����CAN1, ��14����CAN2
    can_filter_init_structure.FilterBank = Object_Para >> 3;
    //�˲�����FIFO0
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
    //ʹ���˲���
    can_filter_init_structure.FilterActivation = ENABLE;
    //�˲���ģʽ������ID����ģʽ
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 32λ�˲�
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //�ӻ�ģʽѡ��ʼ��Ԫ
    can_filter_init_structure.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}
	
/**
 * @brief ��ʼ��CAN����
 *
 * @param hcan CAN���
 * @param Callback_Function ����ص�����
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    if (hcan->Instance == CAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == CAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
}

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


/**
 * @brief HAL��CAN����FIFO0�ж�
 *
 * @param hcan CAN���
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //ѡ��ص�����
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header, CAN1_Manage_Object.Rx_Buffer.Data);
        CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header, CAN2_Manage_Object.Rx_Buffer.Data);
        CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
    }
			CAN_RxHeaderTypeDef rx_header;

	}


void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
			
		  case (0x11):
        {
            Motor_DM_Normal_CAN_RxCpltCallback(&gimbal_control.DM_j4310.motor_j4310,Rx_Buffer->Data);
			break;
        }
		case CAN_YAW_MOTOR_ID:
			{

				get_motor_measure(&motor_chassis[4], Rx_Buffer->Data);
				break;
			}
      case cap_voltage_ID:
			{
				Cap_Voltage =(((int32_t)(Rx_Buffer->Data[0]) << 8) | (int32_t)(Rx_Buffer->Data[1]));   //���������ѹ
				cur_output = (((int32_t)(Rx_Buffer->Data[2]) << 8) | (int32_t)(Rx_Buffer->Data[3]));   //
			}
			default:
			{
				break;
			}
    }
}

void CAN2_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
			
    case CAN_TRIGGER_MOTOR_ID:
    {
      get_motor_measure(&motor_tri, Rx_Buffer->Data);
			detect_hook(TRIGGER_MOTOR_TOE);
      break;
    }
    case CAN_3508_S1_ID:
    {
      get_motor_measure(&motor_shoot[Rx_Buffer->Header.StdId - CAN_3508_S1_ID], Rx_Buffer->Data);
			detect_hook(FRIC_LEFT_MOTOR_TOE);
      break;
    }
    case CAN_3508_S2_ID:
    {
      get_motor_measure(&motor_shoot[Rx_Buffer->Header.StdId - CAN_3508_S1_ID], Rx_Buffer->Data);
			detect_hook(FRIC_RIGHT_MOTOR_TOE);
      break;
    }
    default:
    {
      break;
    }
    }
}
void CAN_cmd_shoot(int16_t fric1, int16_t fric2, int16_t shoot, int16_t rev)
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
  HAL_CAN_AddTxMessage(&hcan2, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
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

	
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
	CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;
	
	return(HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
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
