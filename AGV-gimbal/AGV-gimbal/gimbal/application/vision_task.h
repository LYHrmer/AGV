/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief �����Ӿ����ݰ��������Ӿ��۲����ݣ�Ԥ��װ�װ�λ�ã��Լ����㵯���켣�����е�������
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "usart.h"
#include "dma.h"
#include "INS_task.h"
#include "arm_math.h"
#include "referee.h"
#include "remote_control.h"

//�������Ƕ���� rad
#define ALLOW_ATTACK_ERROR 0.04f
//���������� m 
#define ALLOW_ATTACK_DISTANCE 10.0f
//����������
#define ALLOE_ATTACK_P 3.0f


//��ʱ�ȴ�
#define VISION_SEND_TASK_INIT_TIME 401
//ϵͳ��ʱʱ��
#define VISION_SEND_CONTROL_TIME_MS 1

//��ʱ�ȴ�
#define VISION_TASK_INIT_TIME 450
//ϵͳ��ʱʱ��
#define VISION_CONTROL_TIME_MS 1

//������ת�Ƕ���
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//�����˺���id�ֽ�ֵ�����ڸ�ֵ�����������Ϊ��ɫ��С�����ֵ����������Ϊ��ɫ
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100


//��С�趨����
#define MIN_SET_BULLET_SPEED 20.0f
//����趨����
#define MAX_SET_BULLET_SPEED 30.0f
//��ʼ�趨����
#define BEGIN_SET_BULLET_SPEED 25.0f

//��������ϵ��
#define AIR_K1 0.01f
//��ʼ�ӵ����е�����ֵ
#define T_0 0.0f
//��������
#define PRECISION 0.000001f
//��С������ֵ
#define MIN_DELTAT 0.001f
//����������
#define MAX_ITERATE_COUNT 20
//�Ӿ�����ʱ��
#define VISION_CALC_TIME 0.003f

//��������������ϵ��
#define ITERATE_SCALE_FACTOR 0.3f
//�������ٶ�
#define GRAVITY 9.7988f

//����ʱ��ƫ�Ƽ���λ������ʱ�䵥λms
#define TIME_BIAS 6
//�������������ʱ��ƫ��
#define ROBOT_TIMR_BIAS 100//����С100//С����250//ƽ��100//20
//ƫ��ʱ����д�С
#define TIME_BIAS_QUEUE_CAPACITY 10 

//msתs
#ifndef TIME_MS_TO_S
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)
 
#endif // !TIME_MS_TO_S(x)

//ȫԲ����
#define ALL_CIRCLE (2 * PI)

// ����з�������0.1
//imu��ǹ�ڵ���ֱ����
#define Z_STATIC 0.03955f//0.1f
//ǹ��ǰ�ƾ���
#define DISTANCE_STATIC 0.15733f//0.21085f
//��ʼ����ʱ��
#define INIT_FILIGHT_TIME 0.5f


//���δ�������ݵ�ʱ�� s
#define MAX_NOT_RECEIVE_DATA_TIME 0.05f

//�췽�����Ƕ����
#define RED_AND_BLUE_ANGLE_ERROR 180

//����ת�ٶȲ���P 
#define DISTANCE_TO_SPEED_P 0.7f
//����Զ��ƶ��ٶ�
#define MAX_AUTO_MOVE_SPEED 5.0f
//��С�Զ��ƶ��ٶ�
#define MIN_AUTO_MOVE_SPEED 0.2f


//�ӵ�����
typedef enum
{
    BULLET_17 = 1,
    BULLET_42 = 2,
}bullet_type_e;

//�����������
typedef enum
{
    // ���漺��Ӣ��
    FOLLOW_PERSON_HERO_KEYBOARD = 'Q',
    // ���漺������
    FOLLOW_PERSON_ENGINEER_KEYBOARD = 'W',
    // ���漺������3��
    FOLLOW_PERSON_INFANTRY_3_KEYBOARD = 'E',
    // ���漺������4��
    FOLLOW_PERSON_INFANTRY_4_KEYBOARD = 'R',
    // ���漺������5��
    FOLLOW_PERSON_INFANTRY_5_KEYBOARD = 'T',

    // Ϯ���з�������
    ATTACK_ENEMY_ROBOT_KEYBOARD = 'A',
    // ����Է�ǰ��վ
    ATTACK_ENEMY_OUTPOST_KEYBOARD = 'S',

    // �����ƶ�Ŀ���
    AUTO_MOVE_TARGET_POINT_KEYBOARD = 'D',

} robot_command_keyboard_e;

//������ģʽ
typedef enum
{
    // ���漺������
    FOLLOW_PERSON_ENGINEER,
    // ���漺��Ӣ��
    FOLLOW_PERSON_HERO,
    // ���漺������3��
    FOLLOW_PERSON_INFANTRY_3,
    // ���漺������4��
    FOLLOW_PERSON_INFANTRY_4,
    // ���漺������5��
    FOLLOW_PERSON_INFANTRY_5,

    // Ϯ���з�������
    ATTACK_ENEMY_ROBOT,
    // ����Է�ǰ��վ
    ATTACK_ENEMY_OUTPOST,

    //�����ƶ�Ŀ���
    // AUTO_MOVE_TARGET_POINT,
}robot_mode_e;


// ����ǹ��id
typedef enum
{
    SHOOTER_17_1 = 1,
    SHOOTER_17_2 = 2,
    SHOOTER_42 = 3,
} shooter_id_e;

//��������״̬
typedef enum
{
    //δ��ȡ
    UNLOADED,
    //�Ѷ�ȡ
    LOADED,
}receive_state_e;


//������ʼ֡����
typedef enum
{
    //��λ�����͵���λ��
    LOWER_TO_HIGH_HEAD = 0x5A,
    //��λ�����͵���λ��
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//װ�װ���ɫ
typedef enum
{
    RED = 0,
    BLUE = 1,
}robot_armor_color_e;


typedef enum
{
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7,  
    //ȫ��������
    ARMOR_ALL_ROBOT = 8,
}armor_id_e;

//�ڱ���������
typedef enum
{
    SHOOT_ATTACK,       // Ϯ��
    SHOOT_READY_ATTACK, // ׼��Ϯ��
    SHOOT_STOP_ATTACK,  // ֹͣϮ��
} shoot_command_e;

// �Ӿ�Ŀ��״̬
typedef enum
{
    TARGET_UNAPPEAR, // δʶ��Ŀ��
    TARGET_APPEAR,   // ʶ��Ŀ��
} vision_target_appear_state_e;

//�����˶�ģʽ
typedef enum
{
    FOLLOW_TARGET,
    UNFOLLOW_TARGET,
}vision_control_chassis_mode_e;

//ŷ���ǽṹ��
typedef struct
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
} eular_angle_t;

//����
typedef struct 
{
    // ��������ָ��
    fp32* date;
    // ����
    int16_t capacity;
    // ��ǰ�洢��������
    int16_t cur_size;

}queue_t;

//�����ṹ�� 
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
} vector_t;

// �������ݰ�(����ģʽ�µĽṹ�壬��ֹ�����ݶ������������ݴ�λ)
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t detect_color : 1; // 0-red 1-blue
    bool_t reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;
} send_packet_t;

// �������ݰ�(����ģʽ�µĽṹ�壬��ֹ�����ݶ������������ݴ�λ)
typedef struct __attribute__((packed))
{
    uint8_t header;
    bool_t tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // װ�װ����� 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    float p;   //״̬Э�������ļ�
    uint16_t checksum;
} receive_packet_t;

//����ת��
typedef receive_packet_t target_data_t;

// �Ӿ����սṹ��
typedef struct
{
    // ����״̬λ
    uint8_t receive_state : 1;
    // �ϴν������ݵ�ʱ��
    fp32 last_receive_time;
    // ��ǰ��������ʱ��
    fp32 current_receive_time;

    // ��ǰʱ�� -- ���ڼ����Ƿ�ʱ��δ����
    fp32 current_time;
    //  ���ʱ��
    fp32 interval_time;
    // �������ݰ�
    receive_packet_t receive_packet;
} vision_receive_t;

// �ڱ���̨����˶�����,���˲���������ֵ
typedef struct
{
    // ������̨yaw����ֵ
    fp32 gimbal_yaw;
    // ������̨pitch����ֵ
    fp32 gimbal_pitch;
    //��̨ģʽ
    robot_mode_e robot_mode;
} gimbal_vision_control_t;

// �ڱ��������˶���������
typedef struct
{
    // �Զ���������
    shoot_command_e shoot_command;
} shoot_vision_control_t;

//�ڱ����̿�������
typedef struct 
{
    //�����˵���ģʽ
    vision_control_chassis_mode_e vision_control_chassis_mode;
    //�ҷ������˾���з������˵ľ���
    fp32 distance;
}chassis_vision_control_t;

// Ŀ��λ�ýṹ��
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} target_position_t;


//�Զ��ƶ��ṹ��
typedef struct
{
    //�����˵�ǰλ��
    ext_game_robot_pos_t* game_robot_pos;
    //Ŀ��λ��
    target_position_t target_pos;
    //��ǰλ��
    target_position_t cur_pos;

    //yaw���ʼ�Ƕ� -- ��ǰ�Ƕ�
    fp32 begin_yaw; 
    //yaw������Ƕ� -- ���Խ�
    fp32 command_yaw;
    //pitch������Ƕ� -- ���Խ�
    fp32 command_pitch;
    //�����ƶ��ٶ����� -- �����ƶ��ٶ�
    fp32 command_chassis_vx;
} auto_move_t;


//��������ṹ��
typedef struct
{
    // ��ǰ����
    fp32 current_bullet_speed;
    // ��ǰpitch
    fp32 current_pitch;
    // ��ǰyaw
    fp32 current_yaw;
    // ����ϵ��
    fp32 k1;
    //�ӵ�����ʱ��
    fp32 flight_time;
    // ���м��ʱ��
    fp32 time_bias;
    //Ԥ��ʱ��
    fp32 predict_time;
    

    // Ŀ��yaw
    fp32 target_yaw;

    // װ�װ�����
    uint8_t armor_num;

    //IMU��yaw��������ֱ����
    fp32 z_static;
    //ǹ��ǰ�ƾ���
    fp32 distance_static;

    //����װ�װ�λ��ָ��
    target_position_t* all_target_position_point;

} solve_trajectory_t;



// �Ӿ�����ṹ��
typedef struct
{
    // ���Խ�ָ��
    const INS_t* vision_angle_point;
    // ��ǰ����
    fp32 bullet_speed;
    // ƫ��ʱ��
    queue_t* time_bias;
    //������ģʽ
    robot_mode_e robot_mode;
    // ���װ�װ����ɫ(�з�װ�װ����ɫ)
    uint8_t detect_armor_color;

    //Ŀ������
    target_data_t target_data;
    //�ϴ�Ŀ������
    target_data_t last_target_data;
    
    //��������
    solve_trajectory_t solve_trajectory;
    //Ŀ��λ��
    target_position_t target_position;

    // ����imu���Խ�
    eular_angle_t imu_absolution_angle;

    //������״ָ̬��
    const ext_game_robot_state_t* robot_state_point;
    //�����������ָ��
    const ext_shoot_data_t* shoot_data_point;
    //��������������ָ��
    const ext_robot_command_t* robot_command_point;
    //�����¼�����ָ��
    const ext_event_data_t* field_event_point;
    //��ȡ����������Ѫ��ָ��
    const ext_game_robot_HP_t* game_robot_HP_point;

    // ��������̨��׼λ������
    vector_t robot_gimbal_aim_vector;

    //���յ����ݰ�ָ��
    vision_receive_t* vision_receive_point; 
    //�������ݰ�
    send_packet_t send_packet;


    // �Ӿ�Ŀ��״̬
    vision_target_appear_state_e vision_target_appear_state;
    // Ŀ��װ�װ���
    armor_id_e target_armor_id;
    // ��̨����˶�����
    gimbal_vision_control_t gimbal_vision_control;
    // ���������������
    shoot_vision_control_t shoot_vision_control;
    //�����˶�����
    chassis_vision_control_t chassis_vision_control;

    //�����������ƶ��ṹ��
    auto_move_t auto_move;

} vision_control_t;

// �Ӿ����ݴ�������
void vision_task(void const *pvParameters);

// ��ȡ��λ����̨����
const gimbal_vision_control_t *get_vision_gimbal_point(void);

// ��ȡ��λ����������
const shoot_vision_control_t *get_vision_shoot_point(void);

// ��ȡ��λ�����̿�������
const chassis_vision_control_t* get_vision_chassis_point(void);

//��ȡ�Զ���������ָ��
const auto_move_t *get_auto_move_point(void);

/**
 * @brief �ж��Ӿ��Ƿ�ʶ��Ŀ��
 * 
 * @return bool_t ����1 ʶ��Ŀ�� ����0 δʶ��Ŀ��
 */
bool_t judge_vision_appear_target(void);
/**
 * @brief �жϵ�ǰ������ģʽ�Ƿ�Ϊ�Զ��ƶ�ģʽ
 * 
 * @return bool_t ����1 ��ǰģʽΪ�Զ�����ģʽ ����0 ��ǰģʽ�����Զ�����ģʽ
 */
bool_t judge_cur_mode_is_auto_move_mode(void);


/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з��䣬�ж�yaw��pitch�ĽǶȣ������һ����Χ�ڣ������ֵ���ӣ����ӵ�һ����ֵ���жϷ��䣬���yaw��pitch��Ƕȴ��ڸ÷�Χ�����������
 * 
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 * @param target_distance Ŀ�����
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance);

/**
 * @brief �������ݽ���
 * 
 * @param buf ���յ�������
 * @param len ���յ������ݳ���
 */
void receive_decode(uint8_t* buf, uint32_t len);

/**
 * @brief �����ݰ�ͨ��usb���͵�nuc
 * 
 * @param send �������ݰ�
 */
void send_packet(vision_control_t* send);


/**
 * @brief �������ٿռ� ����ָ��
 * 
 * @param capacity �����ռ������
 * @return queue_t* 
 */
queue_t* queue_create(int16_t capacity);
/**
 * @brief ��������������
 * 
 * @param queue ���нṹ��
 * @param append_data ��ӵ�����
 */
void queue_append_data(queue_t* queue, fp32 append_data);
/**
 * @brief �ͷŶ��д洢���ݵĿռ䣬���ͷŶ��еĿռ�
 * 
 * @param queue ���нṹ��
 */
void queue_delete(queue_t* queue);
/**
 * @brief ����������ݵ�ƽ��ֵ
 * 
 * @param queue queue �ṹ��
 * @return fp32 ���ش洢���ݵ�ƽ��ֵ�� ��������������򷵻�-1
 */
fp32 queue_data_calc_average(queue_t* queue);


#endif // !VISION_TASK_H
