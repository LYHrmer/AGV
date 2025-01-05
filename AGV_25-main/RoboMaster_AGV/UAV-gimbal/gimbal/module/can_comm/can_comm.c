/**
 * @file can_comm.c
 * @author yuanluochen
 * @brief 多设备通信模块，主要用于控制板之间的通信，使用can总线实现
 * @version 0.1
 * @date 2023-09-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "can_comm.h"
#include "main.h"
#include "CRC8_CRC16.h"
#include <string.h>
#include <stdlib.h>


// /**
//  * @brief can设备通信函数 
//  * 
//  * @param can_comm_transmit can设备通信结构体 
//  * @param data 发送数据指针
//  * @param data_len 数据长度
//  */
// void can_comm_transmit(can_comm_t *can_comm_transmit, uint8_t* data, uint8_t data_len)
// {
//     //判断发送数据是否符合要求
//     if ((data_len > CAN_COMM_MAX_BUFSIZE) || (can_comm_transmit == NULL) || (data == NULL))
//     {
//         //不符合要求,直接退出
//         return;
//     }
//     //发送数据长度
//     uint8_t transmit_len = 0;
//     //赋值发送数据参数
//     can_comm_transmit->transmit_buf_len = data_len;
//     //赋值头帧
//     can_comm_transmit->transmit_buf[0] = CAN_COMM_HEADER;
//     //赋值长度
//     can_comm_transmit->transmit_buf[1] = can_comm_transmit->transmit_buf_len;
//     //赋值数据段
//     memcpy(can_comm_transmit->transmit_buf + 2, data, can_comm_transmit->transmit_buf_len);
//     //赋值帧尾
//     can_comm_transmit->transmit_buf[2 + can_comm_transmit->transmit_buf_len] = CAN_COMM_TAIL;
//     //在发送数据末尾添加crc8检验
//     append_CRC8_check_sum(can_comm_transmit->transmit_buf, can_comm_transmit->transmit_buf_len);

//     //数据发送，根据数据大小确定发送次数
//     for (int i = 0; i < can_comm_transmit->transmit_buf_len; i += 8)
//     {
//         //计算该次发送数据长度
//         transmit_len = can_comm_transmit->transmit_buf_len - i >= 8 ? 8 : can_comm_transmit->transmit_buf_len - i;
//         //设置该次发送数据长度

//         //拷贝发送数据
//         // memcpy(can_comm_transmit->)
        
//     }
    
// }

void can_transmit(can_comm_data_t* can_comm_data)
{
    if (can_comm_data == NULL)
        return;
    if (can_comm_data->can_handle == NULL)
        return;

    uint32_t send_mail_box;
    HAL_CAN_AddTxMessage(can_comm_data->can_handle, &can_comm_data->transmit_message, can_comm_data->data, &send_mail_box);
}

can_comm_queue_t *can_comm_queue_init()
{
    can_comm_queue_t *comm_queue = malloc(sizeof(can_comm_queue_t));
    //队列置空
    memset(comm_queue, 0, sizeof(can_comm_queue_t));
    //赋值队列容量
    comm_queue->capacity = CAN_COMM_QUEUE_CAPACITY;
    //初始化队头尾指针
    comm_queue->head = comm_queue->tail = 0;
    return comm_queue;
}

bool can_comm_queue_push(can_comm_queue_t *comm_queue, const can_comm_data_t *can_comm_data)
{
    //判断队列是否已满
    if ((comm_queue->tail + 1) % comm_queue->capacity == comm_queue->head)
    {
        //添加失败不添加
        return false;     
    }
    comm_queue->tail = (++comm_queue->tail) % comm_queue->capacity;
    //尾指针后移添加数据
    memcpy(comm_queue->can_comm_data + comm_queue->tail, can_comm_data, sizeof(can_comm_data_t));
    //返回添加数据
    return true;

}

can_comm_data_t *can_comm_queue_pop(can_comm_queue_t *comm_queue)
{
    //判断是否队空
    if (comm_queue->head == comm_queue->tail)
    {
        //队空返回空指针
        return NULL;
    }
    //返回头指针，并且头指针后移
    comm_queue->head = (++comm_queue->head) % comm_queue->capacity;
    return comm_queue->can_comm_data + comm_queue->head;
}

int can_comm_queue_size(can_comm_queue_t *comm_queue)
{
    return (comm_queue->tail - comm_queue->head + comm_queue->capacity) % comm_queue->capacity;
}


bool can_comm_queue_is_empty(can_comm_queue_t *comm_queue)
{
    return (can_comm_queue_size(comm_queue) == NULL);
}