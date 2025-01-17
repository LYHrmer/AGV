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


//双板can通信数据
static can_comm_data_t board_can_comm_data = {
    .can_handle = &BOARD_CAN, // 初始化双板通信设备can
    .can_comm_target = CAN_COMM_CHASSIS,
};


//裁判系统通信数据
//static can_comm_data_t referee_can_comm_data = {
//    .can_handle = &SHOOT_FLAGS_CAN, // 初始化裁判系统通信设备can
//    .can_comm_target = CAN_COMM_SHOOT_FLAGS,
//};
//发送云台pitch轴的相对角和绝对角


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


void can_comm_board(int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour, int16_t cap_key)
{
    //配置can发送数据
    board_can_comm_data.transmit_message.StdId = CAN_GIMBAL_CONTROL_CHASSIS_ID;
    board_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    board_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    board_can_comm_data.transmit_message.DLC = 0x08;
    board_can_comm_data.data[0] = (chassis_vx >> 8);
    board_can_comm_data.data[1] = chassis_vx;
    board_can_comm_data.data[2] = (chassis_vy >> 8);
    board_can_comm_data.data[3] = chassis_vy;
    board_can_comm_data.data[4] = (chassis_behaviour >> 8);
    board_can_comm_data.data[5] = chassis_behaviour;
    board_can_comm_data.data[6] = (cap_key >> 8);
    board_can_comm_data.data[7] = cap_key;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &board_can_comm_data);
}




//void can_comm_shoot_flags(uint16_t fric0,uint16_t fric1,uint16_t trigger,uint16_t bullet_round)
//{
//	  uint32_t send_mail_box;
//    referee_can_comm_data.transmit_message.StdId = CAN_SHOOT_FLAGS_ID;
//    referee_can_comm_data.transmit_message.IDE = CAN_ID_STD;
//    referee_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
//    referee_can_comm_data.transmit_message.DLC = 0x08;
//    referee_can_comm_data.data[0] = (fric0 >> 8);
//    referee_can_comm_data.data[1] = fric0;
//    referee_can_comm_data.data[2] = (fric1 >> 8);
//    referee_can_comm_data.data[3] = fric1;
//    referee_can_comm_data.data[4] = (trigger >> 8);
//    referee_can_comm_data.data[5] = trigger;
//    referee_can_comm_data.data[6] = (bullet_round >> 8);
//    referee_can_comm_data.data[7] = bullet_round;
//    //添加数据到通信队列
//    add_can_comm_queue(&can_comm, &referee_can_comm_data); 
//}



bool can_comm_task_init_finish(void)
{
    return init_finish;
}

