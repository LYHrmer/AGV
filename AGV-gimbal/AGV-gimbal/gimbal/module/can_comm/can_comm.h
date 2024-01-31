/**
 * @file can_comm.h
 * @author yuanluochen
 * @brief 多设备通信模块，主要用于控制板之间的通信，使用can总线实现，基于数组实现
 * @version 0.1
 * @date 2023-09-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CAN_PACKEET_CONNECTION_H
#define CAN_PACKEET_CONNECTION_H

#include "struct_typedef.h"
#include "main.h"

//最大传输数据量 unit byte
#define CAN_COMM_MAX_BUFSIZE 60
//校验数据量 unit byte, 保存帧头帧尾 + + 数据长度位 + crc校验和
#define CAN_COMM_OFFSET_BUFSIZE 4

//帧头
#define CAN_COMM_HEADER 0X73
//帧尾
#define CAN_COMM_TAIL 0x65

//can设备单次发送最大值
#define CAN_COMM_SINGLE_TRANSMIT_MAX_SIZE 8
//can通信队列容量
#define CAN_COMM_QUEUE_CAPACITY 11

//can通信目标
typedef enum
{
    CAN_COMM_NONE,
    CAN_COMM_GIMBAL,
    CAN_COMM_CHASSIS,
    CAN_COMM_SHOOT,
    CAN_COMM_REFEREE,
} can_comm_target_e;

//can通信数据结构体
typedef struct 
{
    //can设备
    CAN_HandleTypeDef *can_handle;
    //can通信目标
    can_comm_target_e can_comm_target;
    //can发送数据句柄
    CAN_TxHeaderTypeDef transmit_message;
    //通信数据
    uint8_t data[CAN_COMM_SINGLE_TRANSMIT_MAX_SIZE];
}can_comm_data_t;

//can设备通信队列
typedef struct 
{
    //通信数据队列存储缓存
    can_comm_data_t can_comm_data[CAN_COMM_QUEUE_CAPACITY]; 
    //队列容量
    int capacity;
    //队列数据量
    int size;
    //头指针
    int head;
    //尾指针
    int tail;
}can_comm_queue_t;

// /**
//  * @brief can设备通信函数 
//  * 
//  * @param can_comm_transmit can设备通信结构体 
//  * @param data 发送数据指针
//  * @param data_len 数据长度
//  */
// void can_comm_transmit(can_comm_t *can_comm_transmit, uint8_t* data, uint8_t data_len);

/**
 * @brief  can设备通信函数,发送can通信数据结构体内数据
 * 
 * @param can_commit_data can通信数据
 */
void can_transmit(can_comm_data_t* can_commit_data);
/**
 * @brief can通信队列开辟函数
 * 
 * @param queue_capacity 队列空间大小
 * @return can_comm_queue_t* 返回开辟队列指针
 */
can_comm_queue_t *can_comm_queue_init();

/**
 * @brief can通信队列添加, 数据添加使用memcpy进行内存拷贝，无需担心添加数据会变
 * 
 * @param comm_queue can通信队列结构体
 * @param can_comm_data can通信队列数据
 * @return bool_t 添加成功返回true，添加失败返回false
 */
bool can_comm_queue_push(can_comm_queue_t *comm_queue, const can_comm_data_t *can_comm_data);

/**
 * @brief can通信队列出队
 * 
 * @param comm_queue 
 * @return can_comm_data_t* 返回出队元素指针，如果队空则返回空指针
 */
can_comm_data_t *can_comm_queue_pop(can_comm_queue_t *comm_queue);

/**
 * @brief 返回队列大小 
 * 
 * @param comm_queue 队列结构体
 * @return 返回队列大小，类型为整形 
 */
int can_comm_queue_size(can_comm_queue_t *comm_queue);

/**
 * @brief 判断是否发生队空 
 * 
 * @param comm_queue can通信队列
 * @return true 是队空
 * @return false 不是队空
 */
bool can_comm_queue_is_empty(can_comm_queue_t *comm_queue);

#endif // !CAN_PACKEET_CONNECTION_H
