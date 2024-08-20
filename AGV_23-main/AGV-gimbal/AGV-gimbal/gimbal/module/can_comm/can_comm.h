/**
 * @file can_comm.h
 * @author yuanluochen
 * @brief ���豸ͨ��ģ�飬��Ҫ���ڿ��ư�֮���ͨ�ţ�ʹ��can����ʵ�֣���������ʵ��
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

//����������� unit byte
#define CAN_COMM_MAX_BUFSIZE 60
//У�������� unit byte, ����֡ͷ֡β + + ���ݳ���λ + crcУ���
#define CAN_COMM_OFFSET_BUFSIZE 4

//֡ͷ
#define CAN_COMM_HEADER 0X73
//֡β
#define CAN_COMM_TAIL 0x65

//can�豸���η������ֵ
#define CAN_COMM_SINGLE_TRANSMIT_MAX_SIZE 8
//canͨ�Ŷ�������
#define CAN_COMM_QUEUE_CAPACITY 11

//canͨ��Ŀ��
typedef enum
{
    CAN_COMM_NONE,
    CAN_COMM_GIMBAL,
    CAN_COMM_CHASSIS,
    CAN_COMM_SHOOT,
    CAN_COMM_REFEREE,
} can_comm_target_e;

//canͨ�����ݽṹ��
typedef struct 
{
    //can�豸
    CAN_HandleTypeDef *can_handle;
    //canͨ��Ŀ��
    can_comm_target_e can_comm_target;
    //can�������ݾ��
    CAN_TxHeaderTypeDef transmit_message;
    //ͨ������
    uint8_t data[CAN_COMM_SINGLE_TRANSMIT_MAX_SIZE];
}can_comm_data_t;

//can�豸ͨ�Ŷ���
typedef struct 
{
    //ͨ�����ݶ��д洢����
    can_comm_data_t can_comm_data[CAN_COMM_QUEUE_CAPACITY]; 
    //��������
    int capacity;
    //����������
    int size;
    //ͷָ��
    int head;
    //βָ��
    int tail;
}can_comm_queue_t;

// /**
//  * @brief can�豸ͨ�ź��� 
//  * 
//  * @param can_comm_transmit can�豸ͨ�Žṹ�� 
//  * @param data ��������ָ��
//  * @param data_len ���ݳ���
//  */
// void can_comm_transmit(can_comm_t *can_comm_transmit, uint8_t* data, uint8_t data_len);

/**
 * @brief  can�豸ͨ�ź���,����canͨ�����ݽṹ��������
 * 
 * @param can_commit_data canͨ������
 */
void can_transmit(can_comm_data_t* can_commit_data);
/**
 * @brief canͨ�Ŷ��п��ٺ���
 * 
 * @param queue_capacity ���пռ��С
 * @return can_comm_queue_t* ���ؿ��ٶ���ָ��
 */
can_comm_queue_t *can_comm_queue_init();

/**
 * @brief canͨ�Ŷ������, �������ʹ��memcpy�����ڴ濽�������赣��������ݻ��
 * 
 * @param comm_queue canͨ�Ŷ��нṹ��
 * @param can_comm_data canͨ�Ŷ�������
 * @return bool_t ��ӳɹ�����true�����ʧ�ܷ���false
 */
bool can_comm_queue_push(can_comm_queue_t *comm_queue, const can_comm_data_t *can_comm_data);

/**
 * @brief canͨ�Ŷ��г���
 * 
 * @param comm_queue 
 * @return can_comm_data_t* ���س���Ԫ��ָ�룬����ӿ��򷵻ؿ�ָ��
 */
can_comm_data_t *can_comm_queue_pop(can_comm_queue_t *comm_queue);

/**
 * @brief ���ض��д�С 
 * 
 * @param comm_queue ���нṹ��
 * @return ���ض��д�С������Ϊ���� 
 */
int can_comm_queue_size(can_comm_queue_t *comm_queue);

/**
 * @brief �ж��Ƿ����ӿ� 
 * 
 * @param comm_queue canͨ�Ŷ���
 * @return true �Ƕӿ�
 * @return false ���Ƕӿ�
 */
bool can_comm_queue_is_empty(can_comm_queue_t *comm_queue);

#endif // !CAN_PACKEET_CONNECTION_H
