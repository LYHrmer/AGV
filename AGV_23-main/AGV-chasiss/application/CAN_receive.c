/**
 ****************************(C) COPYRIGHT 2019 DJI****************************
 * @file       can_receive.c/h
 * @brief      there is CAN interrupt function  to receive motor data,
 *             and CAN send function to send motor current to control motor.
 *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
 * @note
 * @history
 */

#include "CAN_receive.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx.h"
#include "chassis_task.h"
extern int linkState_1;
extern int linkState_2;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern chassis_move_t chassis_move;

#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

static motor_measure_t motor_chassis[4];                     // 0 1 2 3
static motor_measure_t Forward_L, Forward_R, Back_R, Back_L; // 0 1 2 3

cap_measure_t get_cap;
cap_measure_t get_POWER;

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

static CAN_TxHeaderTypeDef rudder_tx_message;
static uint8_t rudder_can_send_data[8];

static CAN_TxHeaderTypeDef capid_tx_message;
static uint8_t capid_can_send_data[8];

static CAN_TxHeaderTypeDef send_to_gimbal_tx_message;
static uint8_t send_to_gimbal_can_send_data[8];
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @author         LYH
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  if (hcan == &hcan1)
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {

    case CAN_3508_M1_ID:
    case CAN_3508_M2_ID:
    case CAN_3508_M3_ID:
    case CAN_3508_M4_ID:
    {
      static uint8_t i = 0;
      i = rx_header.StdId - CAN_3508_M1_ID;
      get_motor_measure(&motor_chassis[i], rx_data);
      break;
    }
    case CAN_GIMBAL_CALL_BACK_ID:
    {
      chassis_move.gimbal_data.relative_angle_receive = ((int16_t)(rx_data[0] << 8 | rx_data[1])); // 云台相对角度
      chassis_move.vx_set_CANsend = (int16_t)(rx_data[2] << 8 | rx_data[3]);
      chassis_move.vy_set_CANsend = (int16_t)(rx_data[4] << 8 | rx_data[5]);
      chassis_move.chassis_mode_CANsend = (int16_t)(rx_data[6] << 8 | rx_data[7]); // 底盘模式
      linkState_1 = 0;
      break;
    }
    case CAN_GIMBAL_CALL_BACK_KEY_ID:
    {
      chassis_move.chassis_power = ((fp32)(((int16_t)rx_data[0] << 8 )|(int16_t)(rx_data[1])))/100;        // 机器人当前功率
      chassis_move.chassis_power_buffer = ((int16_t)(rx_data[2] << 8 | rx_data[3]));       // 机器人功率buff
      chassis_move.key_C = ((int16_t)(rx_data[4] << 8 | rx_data[5])); // 机器人超电释放按键
      chassis_move.chassis_power_MAX = ((int16_t)(rx_data[6] << 8 | rx_data[7]));                // 机器人最大功率
      linkState_2 = 0;
      break;
    }
    case CAN_CAPID:
    {
      get_cap.invot = (fp32)(((int32_t)(rx_data[1]) << 8) | (int32_t)(rx_data[0])) / 100;   // 输入电压
      get_cap.capvot = (fp32)(((int32_t)(rx_data[3]) << 8) | (int32_t)(rx_data[2])) / 100;  // 电容电压
      get_cap.power = (fp32)get_cap.invot * get_cap.current;                                // 设定功率
      get_cap.current = (fp32)(((int32_t)(rx_data[5]) << 8) | (int32_t)(rx_data[4])) / 100; // 输入电流
      get_cap.power = (fp32)(((int32_t)(rx_data[7]) << 8) | (int32_t)(rx_data[6])) / 100;   // 当前功率
      break;
    }
    case CAN_POWERID: // 功率计
    {
      get_POWER.invot = (fp32)(((int32_t)(rx_data[1]) << 8) | (int32_t)(rx_data[0])) / 100;   // 输入电压
      get_POWER.current = (fp32)(((int32_t)(rx_data[3]) << 8) | (int32_t)(rx_data[2])) / 100; // 电容电压
      get_POWER.power = (fp32)get_POWER.invot * get_POWER.current;                            // 设定功率
      break;
    }
    default:
    {
      break;
    }
    }
  }
  if (hcan == &hcan2)
  {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
    case CAN_Forward_L_ID:
    {
      get_motor_measure(&Forward_L, rx_data);
      //            detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }
    case CAN_Forward_R_ID:
    {
      get_motor_measure(&Forward_R, rx_data);
      //            detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }
    case CAN_BACK_L_ID:
    {
      get_motor_measure(&Back_L, rx_data);
      //            detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }
    case CAN_BACK_R_ID:
    {
      get_motor_measure(&Back_R, rx_data);
      //            detect_hook(CHASSIS_MOTOR1_TOE + i);
      break;
    }
    default:
    {
      break;
    }
    }
  }
}

/**
 * @brief          发送舵电机控制电流(0x205,0x206,0x207,0x208)
 * @author         LYH
 * @param[in]      forward_L: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      forward_R: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      back_L: (0x207) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      back_R: (0x208) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
void CAN_cmd_rudder(int16_t forward_L, int16_t forward_R, int16_t back_L, int16_t back_R)
{
  uint32_t send_mail_box;
  rudder_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  rudder_tx_message.IDE = CAN_ID_STD;
  rudder_tx_message.RTR = CAN_RTR_DATA;
  rudder_tx_message.DLC = 0x08;
  rudder_can_send_data[0] = (forward_L >> 8);
  rudder_can_send_data[1] = forward_L;
  rudder_can_send_data[2] = (forward_R >> 8);
  rudder_can_send_data[3] = forward_R;
  rudder_can_send_data[4] = (back_L >> 8);
  rudder_can_send_data[5] = back_L;
  rudder_can_send_data[6] = (back_R >> 8);
  rudder_can_send_data[7] = back_R;
  HAL_CAN_AddTxMessage(&RUDDER_CAN, &rudder_tx_message, rudder_can_send_data, &send_mail_box);
}

/**
 * @brief          发送轮电机控制电流(0x201,0x202,0x203,0x204)
 * @author         LYH
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          发送超级电容充电功率
 * @author         XQL
 * @param[in]      temPower（0x210）：充电功率（45-100W）
 * @retval         none
 */
void CAN_CMD_cap(int16_t temPower)
{
  uint32_t send_mail_box;
  capid_tx_message.StdId = 0x210;
  capid_tx_message.IDE = CAN_ID_STD;
  capid_tx_message.RTR = CAN_RTR_DATA;
  capid_tx_message.DLC = 0x08;
  capid_can_send_data[0] = temPower >> 8;
  capid_can_send_data[1] = temPower;
  HAL_CAN_AddTxMessage(&CAPID_CAN, &capid_tx_message, capid_can_send_data, &send_mail_box);
}
/**
 * @brief          发送超级电容电压
 * @author         LYH
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t capvot, int16_t rev)
{
  uint32_t send_mail_box;
  send_to_gimbal_tx_message.StdId = 0x221;
  send_to_gimbal_tx_message.IDE = CAN_ID_STD;
  send_to_gimbal_tx_message.RTR = CAN_RTR_DATA;
  send_to_gimbal_tx_message.DLC = 0x08;
  send_to_gimbal_can_send_data[0] = (capvot >> 8);
  send_to_gimbal_can_send_data[1] = capvot;
  send_to_gimbal_can_send_data[2] = (rev >> 8);
  send_to_gimbal_can_send_data[3] = rev;
  send_to_gimbal_can_send_data[4] = (rev >> 8);
  send_to_gimbal_can_send_data[5] = rev;
  send_to_gimbal_can_send_data[6] = (rev >> 8);
  send_to_gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&CAPID_CAN, &send_to_gimbal_tx_message, send_to_gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          返回舵电机 6020电机数据指针
 * @author         LYH
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_Forward_L_motor_measure_point(void)
{
  return &Forward_L;
}
const motor_measure_t *get_Forward_R_motor_measure_point(void)
{
  return &Forward_R;
}
const motor_measure_t *get_Back_R_motor_measure_point(void)
{
  return &Back_R;
}
const motor_measure_t *get_Back_L_motor_measure_point(void)
{
  return &Back_L;
}

/**
 * @brief          返回超级电容数据指针
 * @author         LYH
 * @param[in]      none
 * @retval         超级电容数据指针
 */
cap_measure_t *get_cap_measure_point(void)
{
  return &get_cap;
}

/**
 * @brief          返回轮电机 3508电机数据指针
 * @author         LYH
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}
