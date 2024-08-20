/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "vision_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"


// 视觉任务初始化
static void vision_task_init(vision_control_t* init);
// 视觉任务数据更新
static void vision_task_feedback_update(vision_control_t* update);
// 设置机器人模式
static void vision_set_robot_mode(vision_control_t* set_robot_mode);
// 设置目标装甲板颜色
static void vision_set_target_armor_color(vision_control_t* set_detect_color);
// 设置目标装甲板数数字
static void vision_set_target_armor_num(vision_control_t* set_target_armor_num);
// 判断是否识别到目标
static void vision_judge_appear_target(vision_control_t* judge_appear_target);
// 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
static void vision_data_process(vision_control_t* vision_data);
// 配置发送数据包
static void set_vision_send_packet(vision_control_t* set_send_packet);
// 实时计算弹速
static void calc_current_bullet_speed(vision_control_t* calc_cur_bullet_speed, bullet_type_e bullet_type, shooter_id_e shooter_id);

// 计算自动移动
// static void calc_auto_move_data(vision_control_t* robot_auto_move);
//获取当前机器人位置
static void get_robot_cur_pos(vision_control_t* robot_pos);
//获取目标机器人位置
// static void get_robot_target_pos(vision_control_t* robot_pos);


// 初始化弹道解算的参数
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static);
// 赋值弹道解算的一些可变参数
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed, fp32 time_bias);
// 选择最优击打目标
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position);
// 赋值云台瞄准位置
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time);
// 计算弹道落点 -- 完全空气阻力模型
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch);
// 二维平面弹道模型，计算pitch轴的高度
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z);

// 获取接收数据包指针
static vision_receive_t* get_vision_receive_point(void);

// 视觉任务结构体
vision_control_t vision_control = { 0 };
// 视觉接收结构体
vision_receive_t vision_receive = { 0 };

void vision_task(void const* pvParameters)
{
    // 延时等待，等待上位机发送数据成功
    vTaskDelay(VISION_TASK_INIT_TIME);
    // 视觉任务初始化
    vision_task_init(&vision_control);
//    // 等待云台射击初始化完成
//    while(shoot_control_vision_task() && gimbal_control_vision_task())
//    {
//        //系统延时
//        vTaskDelay(VISION_CONTROL_TIME_MS);
//    }

    while(1)
    {
        // 更新数据
        vision_task_feedback_update(&vision_control);
        //设置机器人模式
        vision_set_robot_mode(&vision_control);
        //判断机器人是否处于自主移动到目标点这一模式

        // 设置目标装甲板颜色
        vision_set_target_armor_color(&vision_control);
        // 设置识别目标装甲板数字
        vision_set_target_armor_num(&vision_control);
        // 判断是否识别到目标
        vision_judge_appear_target(&vision_control);
        // 处理上位机数据,计算弹道的空间落点，并反解空间绝对角,并设置控制命令
        vision_data_process(&vision_control);

        // 配置发送数据包
        set_vision_send_packet(&vision_control);
        // 发送数据包
        send_packet(&vision_control);

        // 系统延时
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

static void vision_task_init(vision_control_t* init)
{
    // 获取陀螺仪绝对角指针                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_point();
    // 获取接收数据包指针
    init->vision_receive_point = get_vision_receive_point();
    // 获取机器人状态指针
    init->robot_state_point = get_game_robot_status_point();
    // 获取发射机构弹速指针
    init->shoot_data_point = get_shoot_data_point();
    // 获取机器人命令指针
    init->robot_command_point = get_robot_command_point();
    // 获取场地状态指针
    init->field_event_point = get_field_event_point();
    // 获取比赛机器人血量指针
    init->game_robot_HP_point = get_game_robot_HP_point();
    //获取当前位置指针
    init->auto_move.game_robot_pos = get_game_robot_pos();

    //初始化发射模式为停止袭击
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    //初始化一些基本的弹道参数
    solve_trajectory_param_init(&init->solve_trajectory, AIR_K1, INIT_FILIGHT_TIME, TIME_MS_TO_S(TIME_BIAS), Z_STATIC, DISTANCE_STATIC);
    //创建偏差时间队列
    init->time_bias = queue_create(TIME_BIAS_QUEUE_CAPACITY);
    queue_append_data(init->time_bias, TIME_BIAS);
    //初始化弹速
    init->bullet_speed = BEGIN_SET_BULLET_SPEED;
    //初始化视觉目标状态为未识别到目标
    init->vision_target_appear_state = TARGET_UNAPPEAR;
    //更新数据
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // 获取云台位姿数据
    update->imu_absolution_angle.yaw = update->vision_angle_point->Yaw;
    update->imu_absolution_angle.pitch = update->vision_angle_point->Pitch;
    update->imu_absolution_angle.roll = update->vision_angle_point->Roll;
    // 修正当前弹速
    calc_current_bullet_speed(update, BULLET_17, SHOOTER_17_1);
    // 存放偏差时间
    queue_append_data(update->time_bias, update->vision_receive_point->interval_time);
    // 更新弹道计算的可变参数
    assign_solve_trajectory_param(&update->solve_trajectory, update->imu_absolution_angle.pitch, update->imu_absolution_angle.yaw, update->bullet_speed, queue_data_calc_average(update->time_bias) + TIME_MS_TO_S(ROBOT_TIMR_BIAS));
    // 获取地图正方向
    update->auto_move.begin_yaw = get_yaw_positive_direction();

    //获取目标数据
    if (update->vision_receive_point->receive_state == UNLOADED)
    {
        //拷贝数据
        memcpy(&update->target_data, &update->vision_receive_point->receive_packet, sizeof(target_data_t));
        //接收数值状态置为已读取
        update->vision_receive_point->receive_state = LOADED;
    }
}

static void vision_set_robot_mode(vision_control_t* set_robot_mode)
{
    
    //根据云台手命令判断机器人模式
    switch (set_robot_mode->robot_command_point->commd_keyboard)
    {
    case FOLLOW_PERSON_ENGINEER_KEYBOARD:
        set_robot_mode->robot_mode = FOLLOW_PERSON_ENGINEER;
        break;
        
    case FOLLOW_PERSON_HERO_KEYBOARD:
        set_robot_mode->robot_mode = FOLLOW_PERSON_HERO;
        break;

    case FOLLOW_PERSON_INFANTRY_3_KEYBOARD:
        set_robot_mode->robot_mode = FOLLOW_PERSON_INFANTRY_3;
        break;

    case FOLLOW_PERSON_INFANTRY_4_KEYBOARD:
        set_robot_mode->robot_mode = FOLLOW_PERSON_INFANTRY_4;
        break;

    case FOLLOW_PERSON_INFANTRY_5_KEYBOARD:
        set_robot_mode->robot_mode = FOLLOW_PERSON_INFANTRY_5;
        break;

    case ATTACK_ENEMY_OUTPOST_KEYBOARD:
        set_robot_mode->robot_mode = ATTACK_ENEMY_OUTPOST;
        break;

    case ATTACK_ENEMY_ROBOT_KEYBOARD:
        set_robot_mode->robot_mode = ATTACK_ENEMY_ROBOT;
        break;
    
    case AUTO_MOVE_TARGET_POINT_KEYBOARD:
        // set_robot_mode->robot_mode = AUTO_MOVE_TARGET_POINT;
        break;

    default:
        set_robot_mode->robot_mode = ATTACK_ENEMY_ROBOT;
        break;
    } 
}

static void vision_set_target_armor_color(vision_control_t* set_detect_color) 
{

    //己方机器人颜色
    robot_armor_color_e person_armor_color;
    //敌方机器人颜色
    robot_armor_color_e enemy_armor_color;
    //目标颜色
    robot_armor_color_e target_armor_color;
    //判断机器人id是否大于ROBOT_RED_AND_BULE_DIVIDE_VALUE这个值
    if (set_detect_color->robot_state_point->robot_id > ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        //自己为蓝色
        person_armor_color = BLUE;
        enemy_armor_color = RED;
    }
    else
    {
        person_armor_color = RED;
        enemy_armor_color = BLUE;
    } 

    //根据机器人模式判断识别目标装甲板的颜色
    switch (set_detect_color->robot_mode)
    {
    case FOLLOW_PERSON_ENGINEER:
    case FOLLOW_PERSON_HERO: 
    case FOLLOW_PERSON_INFANTRY_3:
    case FOLLOW_PERSON_INFANTRY_4:
    case FOLLOW_PERSON_INFANTRY_5:
        //跟随己方机器人时识别己方机器人颜色
        target_armor_color = person_armor_color;
        break;

    case ATTACK_ENEMY_OUTPOST:
    case ATTACK_ENEMY_ROBOT:
        //袭击对方机器人时识别对方机器人颜色
        target_armor_color = enemy_armor_color; 
        break;

    default:
        //其他, 识别对方机器人颜色
        target_armor_color = enemy_armor_color;
        break;
    } 
    //设置目标颜色为目标颜色
    set_detect_color->detect_armor_color = target_armor_color;
}

static void vision_set_target_armor_num(vision_control_t* set_target_armor_num)
{
    //根据机器人模式判断目标装甲板数字
    switch (set_target_armor_num->robot_mode)
    {
        //跟随己方工程
    case FOLLOW_PERSON_ENGINEER:
        set_target_armor_num->target_armor_id = ARMOR_ENGINEER;
        break;
        //跟随己方英雄
    case FOLLOW_PERSON_HERO:
        set_target_armor_num->target_armor_id = ARMOR_HERO;
        break;
        //跟随己方步兵3号
    case FOLLOW_PERSON_INFANTRY_3:
        set_target_armor_num->target_armor_id = ARMOR_INFANTRY3;
        break;
        //跟随己方步兵4号
    case FOLLOW_PERSON_INFANTRY_4:
        set_target_armor_num->target_armor_id = ARMOR_INFANTRY4;
        break;
        //跟随己方步兵5号
    case FOLLOW_PERSON_INFANTRY_5:
        set_target_armor_num->target_armor_id = ARMOR_INFANTRY5;
        break;
        //袭击敌方前哨站
    case ATTACK_ENEMY_OUTPOST:
        set_target_armor_num->target_armor_id = ARMOR_OUTPOST;
        break;
        //袭击敌方机器人
    case ATTACK_ENEMY_ROBOT:
        set_target_armor_num->target_armor_id = ARMOR_ALL_ROBOT;
        break;
    }    
}

static void vision_judge_appear_target(vision_control_t* judge_appear_target)
{
    //根据接收数据判断是否为识别到目标
    if (judge_appear_target->vision_receive_point->receive_packet.x == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.y == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.z == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.yaw == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vx == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vy == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vz == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.v_yaw == 0
        )
    {
        //未识别到目标
        judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
    }
    else
    {
        //识别到目标

        //更新当前时间
        judge_appear_target->vision_receive_point->current_time = TIME_MS_TO_S(HAL_GetTick());
        //判断当前时间是否距离上次接收的时间过长
        if (fabs(judge_appear_target->vision_receive_point->current_time - judge_appear_target->vision_receive_point->current_receive_time) > MAX_NOT_RECEIVE_DATA_TIME)
        {
            //判断为未识别目标
            judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
        }
        else
        {
            //判断识别目标装甲板类型
            if (judge_appear_target->target_armor_id == ARMOR_ALL_ROBOT) //识别全部机器人
            {
                //筛选掉基地和前哨站
                if (judge_appear_target->target_data.id == ARMOR_BASE || judge_appear_target->target_data.id == ARMOR_OUTPOST)
                {
                    //设置不识别目标
                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                }
                else
                {

                    // 设置为识别到目标
                    judge_appear_target->vision_target_appear_state = TARGET_APPEAR;

                    // 判断当前模式为袭击对方机器人
                    if (judge_appear_target->robot_mode == ATTACK_ENEMY_ROBOT)
                    {
                        // 判断目标是否为哨兵 -- 分析是否要进行击打
                        if (judge_appear_target->target_data.id == ARMOR_GUARD)
                        {
                            // 若敌方前哨站存活，则去掉哨兵这一目标
                            // 根据己方装甲板颜色，判断敌方前哨站是否摧毁
                            if (judge_appear_target->detect_armor_color == BLUE)
                            {
                                // 蓝色
                                if (judge_appear_target->game_robot_HP_point->blue_outpost_HP > 1)
                                {
                                    // 敌方前哨站存活 -- 不击打哨兵
                                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                                }
                            }
                            else if (judge_appear_target->detect_armor_color == RED)
                            {
                                // 红色
                                if (judge_appear_target->game_robot_HP_point->red_outpost_HP > 1)
                                {
                                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                                }
                            }
                        }
                    }
               }
            }
            else //识别特定数字
            {
                //判断识别到目标装甲板的数字是否符合目标数字
                if (judge_appear_target->target_data.id == judge_appear_target->target_armor_id)
                {
                    //设置为识别到目标
                    judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
                }
                else
                {
                    //未识别到目标
                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                }


                if (judge_appear_target->robot_mode == ATTACK_ENEMY_OUTPOST)
                {
                    //击打前哨战模式设置为识别所有数字
                    //设置为识别所有数字
                    judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
                }
            }
        }
    }
}


static void vision_data_process(vision_control_t* vision_data)
{
    //判断是否识别到目标
    if (vision_data->vision_target_appear_state == TARGET_APPEAR)
    {
        // 识别到目标
        //  选择最优装甲板
        select_optimal_target(&vision_data->solve_trajectory, &vision_data->target_data, &vision_data->target_position);
        // 计算机器人瞄准位置
        calc_robot_gimbal_aim_vector(&vision_data->robot_gimbal_aim_vector, &vision_data->target_position, vision_data->target_data.vx, vision_data->target_data.vy, vision_data->target_data.vz, vision_data->solve_trajectory.predict_time);
        // 计算机器人pitch轴与yaw轴角度
        vision_data->gimbal_vision_control.gimbal_pitch = calc_target_position_pitch_angle(&vision_data->solve_trajectory, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)) - vision_data->solve_trajectory.distance_static, vision_data->robot_gimbal_aim_vector.z + vision_data->solve_trajectory.z_static);
        vision_data->gimbal_vision_control.gimbal_yaw = atan2(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);

        // 根据机器人模式赋值发送以及运动跟随命令
        switch (vision_data->robot_mode)
        {
        case FOLLOW_PERSON_ENGINEER:
        case FOLLOW_PERSON_HERO:
        case FOLLOW_PERSON_INFANTRY_3:
        case FOLLOW_PERSON_INFANTRY_4:
        case FOLLOW_PERSON_INFANTRY_5:
            {
                // 设置不发弹但是跟随
                vision_data->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
                // 设置底盘模式为跟随模式
                vision_data->chassis_vision_control.vision_control_chassis_mode = FOLLOW_TARGET;
                // 赋值底盘控制命令 -- 我方机器人与目标的距离
                vision_data->chassis_vision_control.distance = sqrt(pow(vision_data->target_data.x, 2) + pow(vision_data->target_data.y, 2));
            }
            break;
        case ATTACK_ENEMY_OUTPOST:
        case ATTACK_ENEMY_ROBOT:
            {
                //设置判断发弹但不跟随
                // 判断发射
                vision_shoot_judge(vision_data, vision_data->gimbal_vision_control.gimbal_yaw - vision_data->imu_absolution_angle.yaw, vision_data->gimbal_vision_control.gimbal_pitch - vision_data->imu_absolution_angle.pitch, sqrt(pow(vision_data->target_data.x, 2) + pow(vision_data->target_data.y, 2)));
                // 设置底盘模式为不跟随
                vision_data->chassis_vision_control.vision_control_chassis_mode = UNFOLLOW_TARGET;
                // 赋值底盘控制命令 -- 0
                vision_data->chassis_vision_control.distance = 0;
            }
            break;
        }

    }
    else
    {
        //未识别到目标 -- 控制值清零
        vision_data->gimbal_vision_control.gimbal_yaw = 0;
        vision_data->gimbal_vision_control.gimbal_pitch = 0;
        vision_data->chassis_vision_control.distance = 0;
        vision_data->chassis_vision_control.vision_control_chassis_mode = UNFOLLOW_TARGET;
        //设置停止发射
        vision_data->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

    vision_data->gimbal_vision_control.robot_mode = vision_data->robot_mode;
}



/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射，判断yaw轴pitch的角度，如果在一定范围内，则计算值增加，增加到一定数值则判断发射，如果yaw轴pitch轴角度大于该范围，则计数归零
 * 
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 * @param target_distance 目标距离
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance)
{ 
    //判断目标距离
    if (target_distance <= ALLOW_ATTACK_DISTANCE)
    {
        // 判断迹是否小于允许值
        if (shoot_judge->vision_receive_point->receive_packet.p < ALLOE_ATTACK_P)
        {
            // 小于一角度开始击打
            if (fabs(vision_begin_add_pitch_angle) <= ALLOW_ATTACK_ERROR && fabs(vision_begin_add_yaw_angle) <= ALLOW_ATTACK_ERROR)
            {
                shoot_judge->shoot_vision_control.shoot_command = SHOOT_ATTACK;
            }
            else
            {
                shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
            }
        }
    }
    else
    {
        //远距离不击打
        shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

}

static void set_vision_send_packet(vision_control_t* set_send_packet)
{
    set_send_packet->send_packet.header = LOWER_TO_HIGH_HEAD;
    set_send_packet->send_packet.detect_color = set_send_packet->detect_armor_color;
    set_send_packet->send_packet.roll = set_send_packet->imu_absolution_angle.roll;
    set_send_packet->send_packet.pitch = set_send_packet->imu_absolution_angle.pitch;
    set_send_packet->send_packet.yaw = set_send_packet->imu_absolution_angle.yaw;
    set_send_packet->send_packet.aim_x = set_send_packet->robot_gimbal_aim_vector.x;
    set_send_packet->send_packet.aim_y = set_send_packet->robot_gimbal_aim_vector.y;
    set_send_packet->send_packet.aim_z = set_send_packet->robot_gimbal_aim_vector.z;
}


/**
 * @brief 由于受天气温度影响，弹速可能会在直流信号上产生较大的变化，所以实时计算弹速，修正精度到整数
 * 
 * @param calc_cur_bullet_speed 视觉控制结构体
 * @param bullet_type 子弹类型
 * @param shooter_id 枪管id
 */
static void calc_current_bullet_speed(vision_control_t* calc_cur_bullet_speed, bullet_type_e bullet_type, shooter_id_e shooter_id)
{
    //判断子弹类型
    if (calc_cur_bullet_speed->shoot_data_point->bullet_type == bullet_type)
    {
        if (calc_cur_bullet_speed->shoot_data_point->shooter_id == shooter_id)
        {
            //筛选不合理的数据
            if (calc_cur_bullet_speed->shoot_data_point->bullet_speed >= MIN_SET_BULLET_SPEED && calc_cur_bullet_speed->shoot_data_point->bullet_speed <= MAX_SET_BULLET_SPEED)
            {
                //修正弹速 -- 修正精度到整数
                calc_cur_bullet_speed->bullet_speed = (int16_t)calc_cur_bullet_speed->shoot_data_point->bullet_speed;
            }
        }
    }
}


// static void calc_auto_move_data(vision_control_t* robot_auto_move)
// {
//     fp32 relative_yaw_angle = 0;
//     // 获取当前位置
//     get_robot_cur_pos(robot_auto_move);
//     //获取机器人目标位置
//     get_robot_target_pos(robot_auto_move);
//     //计算相对yaw轴角度
//     relative_yaw_angle = atan2(robot_auto_move->auto_move.target_pos.y - robot_auto_move->auto_move.cur_pos.y, robot_auto_move->auto_move.target_pos.x - robot_auto_move->auto_move.cur_pos.x);

//     robot_auto_move->auto_move.command_yaw = rad_format(relative_yaw_angle + robot_auto_move->auto_move.begin_yaw);

//     //赋值运动速度 = 距离 * 系数
//     robot_auto_move->auto_move.command_chassis_vx = sqrt(pow((robot_auto_move->auto_move.target_pos.x - robot_auto_move->auto_move.cur_pos.x), 2) + pow((robot_auto_move->auto_move.target_pos.y - robot_auto_move->auto_move.cur_pos.y), 2)) * DISTANCE_TO_SPEED_P;
//     if (fabs(robot_auto_move->auto_move.command_chassis_vx) > MAX_AUTO_MOVE_SPEED)
//     {
//         robot_auto_move->auto_move.command_chassis_vx = MAX_AUTO_MOVE_SPEED;
//     }
//     else if (fabs(robot_auto_move->auto_move.command_chassis_vx) < MIN_AUTO_MOVE_SPEED)
//     {
//         robot_auto_move->auto_move.command_chassis_vx = 0;
//     }

// }


static void get_robot_cur_pos(vision_control_t* robot_pos)
{
    robot_pos->auto_move.cur_pos.x = robot_pos->auto_move.game_robot_pos->x;
    robot_pos->auto_move.cur_pos.y = robot_pos->auto_move.game_robot_pos->y;
    robot_pos->auto_move.cur_pos.z = robot_pos->auto_move.game_robot_pos->z;
    robot_pos->auto_move.cur_pos.yaw = robot_pos->auto_move.game_robot_pos->yaw;
}


// static void get_robot_target_pos(vision_control_t* robot_pos)
// {
//     //判断是否处于自动移动模式
//     if (robot_pos->robot_mode == AUTO_MOVE_TARGET_POINT)
//     {
//         robot_pos->auto_move.target_pos.x = robot_pos->robot_command_point->target_position_x;
//         robot_pos->auto_move.target_pos.y = robot_pos->robot_command_point->target_position_y;
//         robot_pos->auto_move.target_pos.z = robot_pos->robot_command_point->target_position_z;
//     }
// }

void send_packet(vision_control_t* send)
{
    if (send == NULL)
    {
        return;
    }
    //添加CRC16到结尾
    append_CRC16_check_sum((uint8_t*)&send->send_packet, sizeof(send->send_packet));
    //发送数据
    CDC_Transmit_FS((uint8_t*)&send->send_packet, sizeof(send->send_packet));
}

void receive_decode(uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len < 2)
    {
        return;
    }
    //CRC校验
    if (verify_CRC16_check_sum(buf, len))
    {
        receive_packet_t temp_packet = {0};
        // 拷贝接收到的数据到临时内存中
        memcpy(&temp_packet, buf, sizeof(receive_packet_t));
        if (temp_packet.header == HIGH_TO_LOWER_HEAD)
        {
            // 数据正确，将临时数据拷贝到接收数据包中
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            // 接收数据数据状态标志为未读取
            vision_receive.receive_state = UNLOADED;

            // 保存时间
            vision_receive.last_receive_time = vision_receive.current_receive_time;
            // 记录当前接收数据的时间
            vision_receive.current_receive_time = TIME_MS_TO_S(HAL_GetTick());
            //计算时间间隔
            vision_receive.interval_time = vision_receive.current_receive_time - vision_receive.last_receive_time;
        }
    }
}

/**
 * @brief 初始化弹道计算的参数
 * 
 * @param solve_trajectory 弹道计算结构体
 * @param k1 弹道参数
 * @param init_flight_time 初始飞行时间估计值
 * @param time_bias 固有间隔时间
 * @param z_static yaw轴电机到枪口水平面的垂直距离
 * @param distance_static 枪口前推距离 
 */
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static)\
{
    solve_trajectory->k1 = k1;
    solve_trajectory->flight_time = init_flight_time;
    solve_trajectory->time_bias = time_bias;
    solve_trajectory->z_static = z_static;
    solve_trajectory->distance_static = distance_static;
    solve_trajectory->all_target_position_point = NULL;
    solve_trajectory->current_bullet_speed = MIN_SET_BULLET_SPEED;
}

/**
 * @brief 赋值弹道解算的一些可变参数
 *
 * @param solve_trajectory 弹道计算结构体
 * @param current_pitch 当前云台的pitch
 * @param current_yaw 当前云台的yaw
 * @param current_bullet_speed 当前弹速
 * @param time_bias 当前时间偏差值
 */
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed, fp32 time_bias)
{
    solve_trajectory->current_yaw = current_yaw;
    solve_trajectory->current_pitch = current_pitch;
    solve_trajectory->current_bullet_speed = current_bullet_speed;
    solve_trajectory->time_bias = time_bias;
}

/**
 * @brief 选择最优击打目标
 * 
 * @param solve_trajectory 弹道计算结构体 
 * @param vision_data 接收视觉数据
 * @param optimal_target_position 最优目标位置 
 */
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position)
{
    //计算预测时间 = 上一次的子弹飞行时间 + 固有偏移时间, 时间可能不正确，但可以接受
    solve_trajectory->predict_time = solve_trajectory->flight_time + solve_trajectory->time_bias;
    //计算子弹到达目标时的yaw角度
    solve_trajectory->target_yaw = vision_data->yaw + vision_data->v_yaw * solve_trajectory->predict_time;

    //赋值装甲板数量
    solve_trajectory->armor_num = vision_data->armors_num;
    
    //开辟装甲板数量的位置变量的空间
    if (solve_trajectory->all_target_position_point == NULL)
    {
       solve_trajectory->all_target_position_point = malloc(solve_trajectory->armor_num * sizeof(target_position_t));
    }

    //选择目标的数组编号
    uint8_t select_targrt_num = 0;
    
    //计算所有装甲板的位置
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        //由于装甲板距离机器人中心距离不同，但是一般两两对称，所以进行计算装甲板位置时，第0 2块用当前半径，第1 3块用上一次半径
        fp32 r = (i % 2 == 0) ? vision_data->r1 : vision_data->r2;
        solve_trajectory->all_target_position_point[i].yaw = solve_trajectory->target_yaw + i * (ALL_CIRCLE / solve_trajectory->armor_num);
        solve_trajectory->all_target_position_point[i].x = vision_data->x - r * cos(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].y = vision_data->y - r * sin(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].z = (i % 2 == 0) ? vision_data->z : vision_data->z + vision_data->dz;
    }

    // 选择与机器人自身yaw差值最小的目标,排序选择最小目标
    fp32 yaw_error_min = fabsf(solve_trajectory->current_yaw - solve_trajectory->all_target_position_point[0].yaw);
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        fp32 yaw_error_temp = fabsf(solve_trajectory->current_yaw - solve_trajectory->all_target_position_point[i].yaw);
        if (yaw_error_temp <= yaw_error_min)
        {
            yaw_error_min = yaw_error_temp;
            select_targrt_num = i;
        }
    }
    // 将选择的装甲板数据，拷贝打最优目标中去
    memcpy(optimal_target_position, &solve_trajectory->all_target_position_point[select_targrt_num], sizeof(target_position_t));
    //释放开辟的内存
    free(solve_trajectory->all_target_position_point);
    //指针置空
    solve_trajectory->all_target_position_point = NULL;
}

/**
 * @brief 计算装甲板瞄准位置
 * 
 * @param robot_gimbal_aim_vector 机器人云台瞄准向量
 * @param target_position 目标位置
 * @param vx 机器人中心速度
 * @param vy 机器人中心速度
 * @param vz 机器人中心速度
 * @param predict_time 预测时间
 */
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time)
{
    //由于目标与观测中心处于同一系，速度相同
    robot_gimbal_aim_vector->x = target_position->x + vx * predict_time;
    robot_gimbal_aim_vector->y = target_position->y + vy * predict_time;
    robot_gimbal_aim_vector->z = target_position->z + vz * predict_time;
}



/**
 * @brief 计算弹道落点 -- 完全空气阻力模型 该模型适用于大仰角击打的击打
 * 
 * @param solve_trajectory 弹道解算结构体
 * @param x 距离
 * @param bullet_speed 弹速
 * @param pitch 仰角
 * @return 弹道落点
 */
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch)
{
    //子弹落点高度
    fp32 bullet_drop_z = 0;
    //计算总飞行时间
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(pitch)));
    
    if (pitch > 0) 
    {
        //补偿空气阻力系数 对竖直方向
        //上升过程中 子弹速度方向向量的角度逐渐趋近于0，竖直空气阻力 hat(f_z) = f_z * sin(pitch) 会趋近于零 ，水平空气阻力 hat(f_x) = f_x * cos(pitch) 会趋近于 f_x ，所以要对竖直空气阻力系数进行补偿
        fp32 k_z = solve_trajectory->k1 * (1 / sin(pitch));
        // 上升段
        // 初始竖直飞行速度
        fp32 v_z_0 = bullet_speed * sin(pitch);
        // 计算上升段最大飞行时间
        fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
        // 判断总飞行时间是否小于上升最大飞行时间
        if (solve_trajectory->flight_time <= max_flight_up_time)
        {
            // 子弹存在上升段
            bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
        }
        else
        {
            // 超过最大上升飞行时间 -- 存在下降段
            // 计算最大高度
            fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // 计算下降
            bullet_drop_z = z_max - 0.5f * GRAVITY * pow((solve_trajectory->flight_time - max_flight_up_time), 2);
        }
    }
    else
    {
        bullet_drop_z = (float)(bullet_speed * sin(pitch) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    }


    return bullet_drop_z;
}

/**
 * @brief 二维平面弹道模型，计算pitch轴的高度
 * 
 * @param solve_tragectory 弹道计算结构体
 * @param x 水平距离
 * @param z 竖直距离
 * @param bullet_speed 弹速
 * @return 返回pitch轴数值
 */
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z)
{
    // 计算落点高度
    float bullet_drop_z = 0;
    // 瞄准高度
    float aim_z = z;

    // 仰角
    float pitch = 0;
    // 计算值与真实值之间的误差
    float calc_and_actual_error = 0;
    // 比例迭代法
    for (int i = 0; i < MAX_ITERATE_COUNT; i++)
    {
        // 计算仰角
        pitch = atan2(aim_z, x);
        // 计算子弹落点高度
        bullet_drop_z = calc_bullet_drop_in_complete_air(solve_trajectory, x, solve_trajectory->current_bullet_speed, pitch);
        // 计算误差
        calc_and_actual_error = z - bullet_drop_z;
        // 对瞄准高度进行补偿
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // 判断误差是否符合精度要求
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
    //由于为右手系，pitch为向下为正，所以置负
    return -pitch;
}

static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}

//获取当前视觉是否识别到目标
bool_t judge_vision_appear_target(void)
{
    return vision_control.vision_target_appear_state == TARGET_APPEAR;
}

//判断当前机器人模式是否为自动移动模式
bool_t judge_cur_mode_is_auto_move_mode(void)
{
    // return vision_control.robot_mode == AUTO_MOVE_TARGET_POINT;
    return 0;
}

// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}

//获取底盘控制命令
const chassis_vision_control_t* get_vision_chassis_point(void)
{
    return &vision_control.chassis_vision_control;
}

//获取自动跟随命令指针
const auto_move_t* get_auto_move_point(void)
{
    return &vision_control.auto_move;
}


queue_t* queue_create(int16_t capacity)
{
    //创建空间
    queue_t* queue = malloc(sizeof(queue_t));
    if (queue == NULL)
    {
        return NULL;
    }
    //数值清空
    memset(queue, 0, sizeof(queue_t));
    //赋值容量
    queue->capacity = capacity;
    //开辟空间
    queue->date = malloc(queue->capacity * sizeof(fp32));
    //赋值当前存储值
    queue->cur_size = 0; 
    return queue;
}


void queue_append_data(queue_t* queue, fp32 append_data)
{
    
    if (queue->date != NULL && queue->capacity != 0)
    {
        //判断是否已满
        if (queue->cur_size < queue->capacity)
        {
            //未满 -- 添加数据

            //数据后移
            for (int i = (queue->cur_size - 1); i >= 0; i--)
            {
                queue->date[i + 1] = queue->date[i];   
            }

            //添加最新数据
            queue->date[0] = append_data;

            //存储量加1
            queue->cur_size += 1;
        }
        else
        {
            // 已满 -- 清空最后一位数据

            //数据后移动
            for (int i = queue->cur_size - 2; i >= 0; i--)
            {
                queue->date[i + 1] = queue->date[i];
            }

            //添加最新数据
            queue->date[0] = append_data;

            //存储量不变
        }
    }
    else
    {
        return;
    }
}



fp32 queue_data_calc_average(queue_t* queue)
{
    if (queue == NULL)
    {
        //存在问题，返回-1
        return -1;
    }
    fp32 sum = 0;
    if (queue->cur_size > 0)
    {
        for (int i = 0; i < queue->cur_size; i++)
        {
            sum += queue->date[i];
        }
    }
    else
    {
        //无数据返回0
        return 0;
    }

    return sum / queue->cur_size;
}


void queue_delete(queue_t* queue)
{
    if (queue == NULL)
    {
        return;
    }
    //释放存储空间
    free(queue->date);
    queue->date = NULL;
    //释放自身空间
    free(queue);
    queue = NULL;
}

