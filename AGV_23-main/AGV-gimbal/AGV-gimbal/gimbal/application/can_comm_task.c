/**
 * @file can_comm_task.c
 * @author yuanluochen
 * @brief can�豸ͨ���������ö���ʵ��can����˳����
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "can_comm_task.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief canͨ���̳߳�ʼ��, ��Ҫ����Ϊ�����̶߳���
 * 
 * @param can_comm_init canͨ���̳߳�ʼ���ṹ��
 */
static void can_comm_task_init(can_comm_task_t *can_comm_init);

/**
 * @brief canͨ�������ͺ�����ͨ�����ݶ��з��� 
 * 
 * @param can_comm_transmit canͨ��������ƽṹ��
 */
static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit);

/**
 * @brief canͨ�Ŷ�����Ӻ��� 
 * 
 * @param add_comm_queue canͨ��������
 * @param comm_data canͨ������
 */
static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data);

/**
 * @brief canͨ�Ŷ������ݸ���
 * 
 * @param feedback_update canͨ�Ŷ��нṹ��
 */
static void can_comm_feedback_update(can_comm_task_t *feedback_update);

//��̨canͨ������
static can_comm_data_t gimbal_can_comm_data = {
    .can_handle = &GIMBAL_CAN, // ��ʼ����̨ͨ���豸can
    .can_comm_target = CAN_COMM_GIMBAL,
};

//˫��canͨ������
static can_comm_data_t board_can_comm_data = {
    .can_handle = &BOARD_CAN, // ��ʼ��˫��ͨ���豸can
    .can_comm_target = CAN_COMM_CHASSIS,
};

//����canͨ������
static can_comm_data_t shoot_can_comm_data = {
    .can_handle = &SHOOT_CAN, // ��ʼ������ͨ���豸can
    .can_comm_target = CAN_COMM_SHOOT,
};

//����ϵͳͨ������
static can_comm_data_t referee_can_comm_data = {
    .can_handle = &REFEREE_CAN, // ��ʼ������ϵͳͨ���豸can
    .can_comm_target = CAN_COMM_REFEREE,
};

bool init_finish = false;

//ʵ����canͨ���߳̽ṹ��,ȫ�ֱ�������֤����һֱ����
can_comm_task_t can_comm = { 0 };


void can_comm_task(void const* pvParameters)
{ 
    //Ҫ����̨�͵�������ʼ֮ǰ��ɸ�����ĳ�ʼ��
    vTaskDelay(CAN_COMM_TASK_INIT_TIME);
    //canͨ�������ʼ��
    can_comm_task_init(&can_comm);
    init_finish = true;
    while(1)
    {
        //canͨ�Ų�������
        can_comm_feedback_update(&can_comm);
        //canͨ�����ݷ���
        can_comm_task_transmit(&can_comm);
        //can�������ݷ��� 
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
    //��������ʼ��canͨ�Ŷ���
    can_comm_init->can_comm_queue = can_comm_queue_init(); 
}

static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit)
{
    if (can_comm_transmit == NULL)
        return;
    //���зǿշ���
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
    //������ݵ����Ͷ�����
    can_comm_queue_push(add_comm_queue->can_comm_queue, comm_data);
}


void can_comm_gimbal(int16_t yaw, int16_t pitch)
{
    //����can��������
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
    //������ݵ�ͨ�Ŷ���
    add_can_comm_queue(&can_comm, &gimbal_can_comm_data);
}

void can_comm_board(int16_t relative_angle, int16_t chassis_vx, int16_t chassis_vy, int16_t chassis_behaviour)
{
    //����can��������
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
    //������ݵ�ͨ�Ŷ���
    add_can_comm_queue(&can_comm, &board_can_comm_data);
}

void can_comm_shoot(int16_t fric1, int16_t fric2, int16_t trigger)
{
    //����can��������
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
    //������ݵ�ͨ�Ŷ���
    add_can_comm_queue(&can_comm, &shoot_can_comm_data); 
}

void can_can_comm_referee(int16_t key_1,int32_t key_2, int32_t key_3, int16_t key_other )
{
	  uint32_t send_mail_box;
    referee_can_comm_data.transmit_message.StdId = 0x219;
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
    //������ݵ�ͨ�Ŷ���
    add_can_comm_queue(&can_comm, &referee_can_comm_data); 
}

bool can_comm_task_init_finish(void)
{
    return init_finish;
}

