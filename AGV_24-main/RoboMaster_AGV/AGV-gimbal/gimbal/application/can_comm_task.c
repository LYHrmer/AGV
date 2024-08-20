/**
 * @file can_comm_task.c
 * @author yuanluochen
 * @brief can设备通信任务，利用队列实现can数据顺序发送
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "can_comm_task.h"
#include "FreeRTOS.h"
#include "task.h"


static CAN_TxHeaderTypeDef capid_tx_message;
static uint8_t capid_can_send_data[8];

static CAN_TxHeaderTypeDef board_can_tx_message;
static uint8_t board_can_send_data[8];

/**
 * @brief can通信线程初始化, 主要任务为开辟线程队列
 * 
 * @param can_comm_init can通信线程初始化结构体
 */
static void can_comm_task_init(can_comm_task_t *can_comm_init);

/**
 * @brief can通信任务发送函数，通信数据队列发送 
 * 
 * @param can_comm_transmit can通信任务控制结构体
 */
static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit);

/**
 * @brief can通信队列添加函数 
 * 
 * @param add_comm_queue can通信任务函数
 * @param comm_data can通信数据
 */
static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data);

/**
 * @brief can通信队列数据更新
 * 
 * @param feedback_update can通信队列结构体
 */
static void can_comm_feedback_update(can_comm_task_t *feedback_update);

//云台can通信数据
static can_comm_data_t gimbal_can_comm_data = {
    .can_handle = &GIMBAL_CAN, // 初始化云台通信设备can
    .can_comm_target = CAN_COMM_GIMBAL,
};

//双板can通信数据
static can_comm_data_t board_can_comm_data = {
    .can_handle = &BOARD_CAN, // 初始化双板通信设备can
    .can_comm_target = CAN_COMM_CHASSIS,
};

//发弹can通信数据
static can_comm_data_t shoot_can_comm_data = {
    .can_handle = &SHOOT_CAN, // 初始化发弹通信设备can
    .can_comm_target = CAN_COMM_SHOOT,
};

//裁判系统通信数据
static can_comm_data_t referee_can_comm_data = {
    .can_handle = &REFEREE_CAN, // 初始化裁判系统通信设备can
    .can_comm_target = CAN_COMM_REFEREE,
};
//发送云台pitch轴的相对角和绝对角
static can_comm_data_t PitchAngle_can_comm_data = {
    .can_handle = &PitchAngle_CAN, // 初始化裁判系统通信设备can
    .can_comm_target = CAN_COMM_PITCHANGLE,
};

bool init_finish = false;

//实例化can通信线程结构体,全局变量，保证数据一直存在
can_comm_task_t can_comm = { 0 };


void can_comm_task(void const* pvParameters)
{ 
    //要在云台和底盘任务开始之前完成该任务的初始化
    vTaskDelay(CAN_COMM_TASK_INIT_TIME);
    //can通信任务初始化
    can_comm_task_init(&can_comm);
    init_finish = true;
    while(1)
    {
        //can通信参数更新
        can_comm_feedback_update(&can_comm);
        //can通信数据发送
        can_comm_task_transmit(&can_comm);
        //can数据数据发送 
        vTaskDelay(CAN_COMM_TASK_TIME);
    }
}

static void can_comm_feedback_update(can_comm_task_t *feedback_update)
{
    feedback_update->can_comm_queue->size = can_comm_queue_size(feedback_update->can_comm_queue);
}

static void can_comm_task_init(can_comm_task_t *can_comm_init)
{
    if (can_comm_init == NULL)
        return; 
    //创建并初始化can通信队列
    can_comm_init->can_comm_queue = can_comm_queue_init(); 
}

static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit)
{
    if (can_comm_transmit == NULL)
        return;
    //队列非空发送
    if (!can_comm_queue_is_empty(can_comm_transmit->can_comm_queue))
    {
        can_comm_data_t *data = can_comm_queue_pop(can_comm_transmit->can_comm_queue);
        if(data)
        {
            can_transmit(data);
        }
    }
}


static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data)
{
    if (add_comm_queue == NULL || comm_data == NULL)
        return;
    //添加数据到发送队列中
    can_comm_queue_push(add_comm_queue->can_comm_queue, comm_data);
}


void can_comm_gimbal(int16_t yaw, int16_t pitch)
{
    //配置can发送数据
    gimbal_can_comm_data.transmit_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    gimbal_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    gimbal_can_comm_data.transmit_message.DLC = 0x08;
    gimbal_can_comm_data.data[0] = (yaw >> 8);
    gimbal_can_comm_data.data[1] = yaw;
    gimbal_can_comm_data.data[2] = (pitch >> 8);
    gimbal_can_comm_data.data[3] = pitch;
    gimbal_can_comm_data.data[4] = 0;
    gimbal_can_comm_data.data[5] = 0;
    gimbal_can_comm_data.data[6] = 0;
    gimbal_can_comm_data.data[7] = 0;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &gimbal_can_comm_data);
}

//void can_comm_board(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour)
//{
//  uint32_t send_mail_box;
//  board_can_tx_message.StdId = CAN_GIMBAL_CONTROL_CHASSIS_ID;
//  board_can_tx_message.IDE = CAN_ID_STD;
//  board_can_tx_message.RTR = CAN_RTR_DATA;
//  board_can_tx_message.DLC = 0x08;
//  board_can_send_data[0] = relative_angle >> 8;
//  board_can_send_data[1] = relative_angle;
//  board_can_send_data[2] = chassis_vx >> 8;
//  board_can_send_data[3] = chassis_vx;
//  board_can_send_data[4] = chassis_vy >> 8;
//  board_can_send_data[5] = chassis_vy;
//  board_can_send_data[6] = chassis_behaviour >> 8;
//  board_can_send_data[7] = chassis_behaviour;
//  HAL_CAN_AddTxMessage(&hcan1, &board_can_tx_message, board_can_send_data, &send_mail_box);
//}


void can_comm_board(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour)
{
    //配置can发送数据
    board_can_comm_data.transmit_message.StdId = CAN_GIMBAL_CONTROL_CHASSIS_ID;
    board_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    board_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    board_can_comm_data.transmit_message.DLC = 0x08;
    board_can_comm_data.data[0] = (relative_angle >> 8);
    board_can_comm_data.data[1] = relative_angle;
    board_can_comm_data.data[2] = (chassis_vx >> 8);
    board_can_comm_data.data[3] = chassis_vx;
    board_can_comm_data.data[4] = (chassis_vy >> 8);
    board_can_comm_data.data[5] = chassis_vy;
    board_can_comm_data.data[6] = (chassis_behaviour >> 8);
    board_can_comm_data.data[7] = chassis_behaviour;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &board_can_comm_data);
}

//void can_comm_pitchangle(int16_t pitch_relative,int32_t pitch_absolute, int32_t key_3, int16_t key_other )
//{
//    PitchAngle_can_comm_data.transmit_message.StdId = CAN_PITCHANGLE_ID;
//    PitchAngle_can_comm_data.transmit_message.IDE = CAN_ID_STD;
//    PitchAngle_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
//    PitchAngle_can_comm_data.transmit_message.DLC = 0x08;
//    PitchAngle_can_comm_data.data[0] = (pitch_relative >> 8);
//    PitchAngle_can_comm_data.data[1] = pitch_relative;
//    PitchAngle_can_comm_data.data[2] = (pitch_absolute >> 8);
//    PitchAngle_can_comm_data.data[3] = pitch_absolute;
//    PitchAngle_can_comm_data.data[4] = (key_3 >> 8);
//    PitchAngle_can_comm_data.data[5] = key_3;
//    PitchAngle_can_comm_data.data[6] = (key_other >> 8);
//    PitchAngle_can_comm_data.data[7] = key_other;
//    //添加数据到通信队列
//    add_can_comm_queue(&can_comm, &PitchAngle_can_comm_data); 
//}

void CAN_CMD_cap(int16_t pitch_relative,int16_t pitch_absolute,int16_t kay_ctrl)
{
  uint32_t send_mail_box;
  capid_tx_message.StdId = 0x213;
  capid_tx_message.IDE = CAN_ID_STD;
  capid_tx_message.RTR = CAN_RTR_DATA;
  capid_tx_message.DLC = 0x08;
  capid_can_send_data[0] = pitch_relative >> 8;
  capid_can_send_data[1] = pitch_relative;
  capid_can_send_data[2] = pitch_absolute >> 8;
  capid_can_send_data[3] = pitch_absolute;
  capid_can_send_data[2] = kay_ctrl >> 8;
  capid_can_send_data[3] = kay_ctrl;
  HAL_CAN_AddTxMessage(&hcan1, &capid_tx_message, capid_can_send_data, &send_mail_box);
}

void can_comm_shoot(int16_t fric1, int16_t fric2, int16_t trigger)
{
    //配置can发送数据
    shoot_can_comm_data.transmit_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    shoot_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    shoot_can_comm_data.transmit_message.DLC = 0x08;
    shoot_can_comm_data.data[0] = (fric1 >> 8);
    shoot_can_comm_data.data[1] = fric1;
    shoot_can_comm_data.data[2] = (fric2 >> 8);
    shoot_can_comm_data.data[3] = fric2;
    shoot_can_comm_data.data[4] = (trigger >> 8);
    shoot_can_comm_data.data[5] = trigger;
    shoot_can_comm_data.data[6] = 0;
    shoot_can_comm_data.data[7] = 0;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &shoot_can_comm_data); 
}

void can_comm_referee(int16_t key_1,int32_t key_2, int32_t key_3, int16_t key_other )
{
	  uint32_t send_mail_box;
    referee_can_comm_data.transmit_message.StdId = CAN_REFEREE_ID;
    referee_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    referee_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    referee_can_comm_data.transmit_message.DLC = 0x08;
    referee_can_comm_data.data[0] = (key_1 >> 8);
    referee_can_comm_data.data[1] = key_1;
    referee_can_comm_data.data[2] = (key_2 >> 8);
    referee_can_comm_data.data[3] = key_2;
    referee_can_comm_data.data[4] = (key_3 >> 8);
    referee_can_comm_data.data[5] = key_3;
    referee_can_comm_data.data[6] = (key_other >> 8);
    referee_can_comm_data.data[7] = key_other;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &referee_can_comm_data); 
}


bool can_comm_task_init_finish(void)
{
    return init_finish;
}

