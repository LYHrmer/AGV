/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
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

//允许发弹角度误差 rad
#define ALLOW_ATTACK_ERROR 0.042f//0.04f
//允许发弹距离 m 
#define ALLOW_ATTACK_DISTANCE 20.0f
//允许发弹概率
#define ALLOE_ATTACK_P 10.0f


//延时等待
#define VISION_SEND_TASK_INIT_TIME 401
//系统延时时间
#define VISION_SEND_CONTROL_TIME_MS 1

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 1

//弧度制转角度制
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//机器人红蓝id分界值，大于该值则机器人自身为蓝色，小于这个值机器人自身为红色
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100


//最小设定弹速
#define MIN_SET_BULLET_SPEED 20.0f
//最大设定弹速
#define MAX_SET_BULLET_SPEED 30.0f
//初始设定弹速
#define BEGIN_SET_BULLET_SPEED 25.0f

//空气阻力系数
#define AIR_K1 0.005f
//初始子弹飞行迭代数值
#define T_0 0.0f
//迭代精度
#define PRECISION 0.000001f
//最小迭代差值
#define MIN_DELTAT 0.001f
//最大迭代次数
#define MAX_ITERATE_COUNT 20
//视觉计算时间
#define VISION_CALC_TIME 0.003f

//比例补偿器比例系数
#define ITERATE_SCALE_FACTOR 0.3f
//重力加速度
#define GRAVITY 9.79849f//9.8035f//9.7988f

//固有时间偏移即上位机计算时间单位ms
#define TIME_BIAS 40
//机器人自身固有时间偏差
#define ROBOT_TIMR_BIAS 75


//偏差时间队列大小
#define TIME_BIAS_QUEUE_CAPACITY 10

//ms转s
#ifndef TIME_MS_TO_S
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)
 
#endif // !TIME_MS_TO_S(x)

//全圆弧度0
#define ALL_CIRCLE (2 * PI)

// 击打敌方机器人0.1
//imu到枪口的竖直距离
#define Z_STATIC -0.0281f//
//枪口前推距离
#define DISTANCE_STATIC 0.06868f//
//初始飞行时间
#define INIT_FILIGHT_TIME 0.5f


//最大未接受数据的时间 s
#define MAX_NOT_RECEIVE_DATA_TIME 0.05f

//红方蓝方角度误差
#define RED_AND_BLUE_ANGLE_ERROR 180

//距离转速度参数P 
#define DISTANCE_TO_SPEED_P 0.7f
//最大自动移动速度
#define MAX_AUTO_MOVE_SPEED 5.0f
//最小自动移动速度
#define MIN_AUTO_MOVE_SPEED 0.2f


//子弹类型
typedef enum
{
    BULLET_17 = 1,
    BULLET_42 = 2,
}bullet_type_e;

//机器人命令按键
typedef enum
{
    // 跟随己方英雄
    FOLLOW_PERSON_HERO_KEYBOARD = 'Q',
    // 跟随己方工程
    FOLLOW_PERSON_ENGINEER_KEYBOARD = 'W',
    // 跟随己方步兵3号
    FOLLOW_PERSON_INFANTRY_3_KEYBOARD = 'E',
    // 跟随己方步兵4号
    FOLLOW_PERSON_INFANTRY_4_KEYBOARD = 'R',
    // 跟随己方步兵5号
    FOLLOW_PERSON_INFANTRY_5_KEYBOARD = 'T',

    // 袭击敌方机器人
    ATTACK_ENEMY_ROBOT_KEYBOARD = 'A',
    // 击打对方前哨站
    ATTACK_ENEMY_OUTPOST_KEYBOARD = 'S',

    // 自主移动目标点
    AUTO_MOVE_TARGET_POINT_KEYBOARD = 'D',

} robot_command_keyboard_e;

//机器人模式
typedef enum
{
    // 跟随己方工程
    FOLLOW_PERSON_ENGINEER,
    // 跟随己方英雄
    FOLLOW_PERSON_HERO,
    // 跟随己方步兵3号
    FOLLOW_PERSON_INFANTRY_3,
    // 跟随己方步兵4号
    FOLLOW_PERSON_INFANTRY_4,
    // 跟随己方步兵5号
    FOLLOW_PERSON_INFANTRY_5,

    // 袭击敌方机器人
    ATTACK_ENEMY_ROBOT,
    // 击打对方前哨站
    ATTACK_ENEMY_OUTPOST,

    //自主移动目标点
    // AUTO_MOVE_TARGET_POINT,
}robot_mode_e;


// 发射枪管id
typedef enum
{
    SHOOTER_17_1 = 1,
    SHOOTER_17_2 = 2,
    SHOOTER_42 = 3,
} shooter_id_e;

//接收数据状态
typedef enum
{
    //未读取
    UNLOADED,
    //已读取
    LOADED,
}receive_state_e;


//数据起始帧类型
typedef enum
{
    //下位机发送到上位机
    LOWER_TO_HIGH_HEAD = 0x5A,
    //上位机发送到下位机
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//装甲板颜色
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
    //全部机器人
    ARMOR_ALL_ROBOT = 8,
}armor_id_e;

//哨兵发射命令
typedef enum
{
    SHOOT_ATTACK,       // 袭击
    SHOOT_READY_ATTACK, // 准备袭击
    SHOOT_STOP_ATTACK,  // 停止袭击
} shoot_command_e;

// 视觉目标状态
typedef enum
{
    TARGET_UNAPPEAR, // 未识别到目标
    TARGET_APPEAR,   // 识别到目标
} vision_target_appear_state_e;

//底盘运动模式
typedef enum
{
    FOLLOW_TARGET,
    UNFOLLOW_TARGET,
}vision_control_chassis_mode_e;

//欧拉角结构体
typedef struct
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
} eular_angle_t;

//队列
typedef struct 
{
    // 队列数据指针
    fp32* date;
    // 容量
    int16_t capacity;
    // 当前存储的数据量
    int16_t cur_size;

}queue_t;

//向量结构体 
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
} vector_t;

// 发送数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
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

// 接收数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
typedef struct __attribute__((packed))
{
    uint8_t header;
    bool_t tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // 装甲板数量 2-balance 3-outpost 4-normal
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
    float p;   //状态协方差矩阵的迹
    uint16_t checksum;
} receive_packet_t;

//类型转化
typedef receive_packet_t target_data_t;

// 视觉接收结构体
typedef struct
{
    // 接收状态位
    uint8_t receive_state : 1;
    // 上次接收数据的时间
    fp32 last_receive_time;
    // 当前接受数据时间
    fp32 current_receive_time;

    // 当前时间 -- 用于计算是否长时间未接受
    fp32 current_time;
    //  间隔时间
    fp32 interval_time;
    // 接收数据包
    receive_packet_t receive_packet;
} vision_receive_t;

// 哨兵云台电机运动命令,经滤波处理后的数值
typedef struct
{
    // 本次云台yaw轴数值
    fp32 gimbal_yaw;
    // 本次云台pitch轴数值
    fp32 gimbal_pitch;
    //云台模式
    robot_mode_e robot_mode;
} gimbal_vision_control_t;

// 哨兵发射电机运动控制命令
typedef struct
{
    // 自动发射命令
    shoot_command_e shoot_command;
} shoot_vision_control_t;

//哨兵底盘控制命令
typedef struct 
{
    //机器人底盘模式
    vision_control_chassis_mode_e vision_control_chassis_mode;
    //我方机器人距离敌方机器人的距离
    fp32 distance;
}chassis_vision_control_t;

// 目标位置结构体
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} target_position_t;


//自动移动结构体
typedef struct
{
    //机器人当前位置
    ext_game_robot_pos_t* game_robot_pos;
    //目标位置
    target_position_t target_pos;
    //当前位置
    target_position_t cur_pos;

    //yaw轴初始角度 -- 正前角度
    fp32 begin_yaw; 
    //yaw轴命令角度 -- 绝对角
    fp32 command_yaw;
    //pitch轴命令角度 -- 绝对角
    fp32 command_pitch;
    //底盘移动速度命令 -- 底盘移动速度
    fp32 command_chassis_vx;
} auto_move_t;


//弹道计算结构体
typedef struct
{
    // 当前弹速
    fp32 current_bullet_speed;
    // 当前pitch
    fp32 current_pitch;
    // 当前yaw
    fp32 current_yaw;
    // 弹道系数
    fp32 k1;
    //子弹飞行时间
    fp32 flight_time;
    // 固有间隔时间
    fp32 time_bias;
    //预测时间
    fp32 predict_time;
    

    // 目标yaw
    fp32 target_yaw;

    // 装甲板数量
    uint8_t armor_num;

    //IMU到yaw轴电机的竖直距离
    fp32 z_static;
    //枪口前推距离
    fp32 distance_static;

    //所有装甲板位置指针
    target_position_t* all_target_position_point;

} solve_trajectory_t;



// 视觉任务结构体
typedef struct
{
    // 绝对角指针
    const INS_t* vision_angle_point;
    // 当前弹速
    fp32 bullet_speed;
    // 偏差时间
    queue_t* time_bias;
    //机器人模式
    robot_mode_e robot_mode;
    // 检测装甲板的颜色(敌方装甲板的颜色)
    uint8_t detect_armor_color;

    //目标数据
    target_data_t target_data;
    //上次目标数据
    target_data_t last_target_data;
    
    //弹道解算
    solve_trajectory_t solve_trajectory;
    //目标位置
    target_position_t target_position;

    // 自身imu绝对角
    eular_angle_t imu_absolution_angle;

    //机器人状态指针
    const ext_game_robot_state_t* robot_state_point;
    //发射机构弹速指针
    const ext_shoot_data_t* shoot_data_point;
    //机器人命令数据指针
    const ext_robot_command_t* robot_command_point;
    //场地事件数据指针
    const ext_event_data_t* field_event_point;
    //获取比赛机器人血量指针
    const ext_game_robot_HP_t* game_robot_HP_point;

    // 机器人云台瞄准位置向量
    vector_t robot_gimbal_aim_vector;

    //接收的数据包指针
    vision_receive_t* vision_receive_point; 
    //发送数据包
    send_packet_t send_packet;


    // 视觉目标状态
    vision_target_appear_state_e vision_target_appear_state;
    // 目标装甲板编号
    armor_id_e target_armor_id;
    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;
    // 发射机构发射命令
    shoot_vision_control_t shoot_vision_control;
    //底盘运动命令
    chassis_vision_control_t chassis_vision_control;

    //机器人自主移动结构体
    auto_move_t auto_move;

} vision_control_t;

// 视觉数据处理任务
void vision_task(void const *pvParameters);

// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void);

// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void);

// 获取上位机底盘控制命令
const chassis_vision_control_t* get_vision_chassis_point(void);

//获取自动跟随命令指针
const auto_move_t *get_auto_move_point(void);

/**
 * @brief 判断视觉是否识别到目标
 * 
 * @return bool_t 返回1 识别到目标 返回0 未识别到目标
 */
bool_t judge_vision_appear_target(void);
/**
 * @brief 判断当前机器人模式是否为自动移动模式
 * 
 * @return bool_t 返回1 当前模式为自动跟随模式 返回0 当前模式不是自动跟随模式
 */
bool_t judge_cur_mode_is_auto_move_mode(void);


/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射，判断yaw轴pitch的角度，如果在一定范围内，则计算值增加，增加到一定数值则判断发射，如果yaw轴pitch轴角度大于该范围，则计数归零
 * 
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 * @param target_distance 目标距离
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance);

/**
 * @brief 接收数据解码
 * 
 * @param buf 接收到的数据
 * @param len 接收到的数据长度
 */
void receive_decode(uint8_t* buf, uint32_t len);

/**
 * @brief 将数据包通过usb发送到nuc
 * 
 * @param send 发送数据包
 */
void send_packet(vision_control_t* send);


/**
 * @brief 创建开辟空间 返回指针
 * 
 * @param capacity 创建空间的容量
 * @return queue_t* 
 */
queue_t* queue_create(int16_t capacity);
/**
 * @brief 向队列中添加数据
 * 
 * @param queue 队列结构体
 * @param append_data 添加的数据
 */
void queue_append_data(queue_t* queue, fp32 append_data);
/**
 * @brief 释放队列存储数据的空间，并释放队列的空间
 * 
 * @param queue 队列结构体
 */
void queue_delete(queue_t* queue);
/**
 * @brief 计算队列数据的平均值
 * 
 * @param queue queue 结构体
 * @return fp32 返回存储数据的平均值， 如果计算有问题则返回-1
 */
fp32 queue_data_calc_average(queue_t* queue);


#endif // !VISION_TASK_H
