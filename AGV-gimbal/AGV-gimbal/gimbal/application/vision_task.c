/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief �����Ӿ����ݰ��������Ӿ��۲����ݣ�Ԥ��װ�װ�λ�ã��Լ����㵯���켣�����е�������
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


// �Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
// �Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
// ���û�����ģʽ
static void vision_set_robot_mode(vision_control_t* set_robot_mode);
// ����Ŀ��װ�װ���ɫ
static void vision_set_target_armor_color(vision_control_t* set_detect_color);
// ����Ŀ��װ�װ�������
static void vision_set_target_armor_num(vision_control_t* set_target_armor_num);
// �ж��Ƿ�ʶ��Ŀ��
static void vision_judge_appear_target(vision_control_t* judge_appear_target);
// ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�
static void vision_data_process(vision_control_t* vision_data);
// ���÷������ݰ�
static void set_vision_send_packet(vision_control_t* set_send_packet);
// ʵʱ���㵯��
static void calc_current_bullet_speed(vision_control_t* calc_cur_bullet_speed, bullet_type_e bullet_type, shooter_id_e shooter_id);

// �����Զ��ƶ�
// static void calc_auto_move_data(vision_control_t* robot_auto_move);
//��ȡ��ǰ������λ��
static void get_robot_cur_pos(vision_control_t* robot_pos);
//��ȡĿ�������λ��
// static void get_robot_target_pos(vision_control_t* robot_pos);


// ��ʼ����������Ĳ���
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 time_bias, fp32 z_static, fp32 distance_static);
// ��ֵ���������һЩ�ɱ����
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed, fp32 time_bias);
// ѡ�����Ż���Ŀ��
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position);
// ��ֵ��̨��׼λ��
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time);
// ���㵯����� -- ��ȫ��������ģ��
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch);
// ��άƽ�浯��ģ�ͣ�����pitch��ĸ߶�
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z);

// ��ȡ�������ݰ�ָ��
static vision_receive_t* get_vision_receive_point(void);

// �Ӿ�����ṹ��
vision_control_t vision_control = { 0 };
// �Ӿ����սṹ��
vision_receive_t vision_receive = { 0 };

void vision_task(void const* pvParameters)
{
    // ��ʱ�ȴ����ȴ���λ���������ݳɹ�
    vTaskDelay(VISION_TASK_INIT_TIME);
    // �Ӿ������ʼ��
    vision_task_init(&vision_control);
//    // �ȴ���̨�����ʼ�����
//    while(shoot_control_vision_task() && gimbal_control_vision_task())
//    {
//        //ϵͳ��ʱ
//        vTaskDelay(VISION_CONTROL_TIME_MS);
//    }

    while(1)
    {
        // ��������
        vision_task_feedback_update(&vision_control);
        //���û�����ģʽ
        vision_set_robot_mode(&vision_control);
        //�жϻ������Ƿ��������ƶ���Ŀ�����һģʽ

        // ����Ŀ��װ�װ���ɫ
        vision_set_target_armor_color(&vision_control);
        // ����ʶ��Ŀ��װ�װ�����
        vision_set_target_armor_num(&vision_control);
        // �ж��Ƿ�ʶ��Ŀ��
        vision_judge_appear_target(&vision_control);
        // ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�,�����ÿ�������
        vision_data_process(&vision_control);

        // ���÷������ݰ�
        set_vision_send_packet(&vision_control);
        // �������ݰ�
        send_packet(&vision_control);

        // ϵͳ��ʱ
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

static void vision_task_init(vision_control_t* init)
{
    // ��ȡ�����Ǿ��Խ�ָ��                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_point();
    // ��ȡ�������ݰ�ָ��
    init->vision_receive_point = get_vision_receive_point();
    // ��ȡ������״ָ̬��
    init->robot_state_point = get_game_robot_status_point();
    // ��ȡ�����������ָ��
    init->shoot_data_point = get_shoot_data_point();
    // ��ȡ����������ָ��
    init->robot_command_point = get_robot_command_point();
    // ��ȡ����״ָ̬��
    init->field_event_point = get_field_event_point();
    // ��ȡ����������Ѫ��ָ��
    init->game_robot_HP_point = get_game_robot_HP_point();
    //��ȡ��ǰλ��ָ��
    init->auto_move.game_robot_pos = get_game_robot_pos();

    //��ʼ������ģʽΪֹͣϮ��
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    //��ʼ��һЩ�����ĵ�������
    solve_trajectory_param_init(&init->solve_trajectory, AIR_K1, INIT_FILIGHT_TIME, TIME_MS_TO_S(TIME_BIAS), Z_STATIC, DISTANCE_STATIC);
    //����ƫ��ʱ�����
    init->time_bias = queue_create(TIME_BIAS_QUEUE_CAPACITY);
    queue_append_data(init->time_bias, TIME_BIAS);
    //��ʼ������
    init->bullet_speed = BEGIN_SET_BULLET_SPEED;
    //��ʼ���Ӿ�Ŀ��״̬Ϊδʶ��Ŀ��
    init->vision_target_appear_state = TARGET_UNAPPEAR;
    //��������
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // ��ȡ��̨λ������
    update->imu_absolution_angle.yaw = update->vision_angle_point->Yaw;
    update->imu_absolution_angle.pitch = update->vision_angle_point->Pitch;
    update->imu_absolution_angle.roll = update->vision_angle_point->Roll;
    // ������ǰ����
    calc_current_bullet_speed(update, BULLET_17, SHOOTER_17_1);
    // ���ƫ��ʱ��
    queue_append_data(update->time_bias, update->vision_receive_point->interval_time);
    // ���µ�������Ŀɱ����
    assign_solve_trajectory_param(&update->solve_trajectory, update->imu_absolution_angle.pitch, update->imu_absolution_angle.yaw, update->bullet_speed, queue_data_calc_average(update->time_bias) + TIME_MS_TO_S(ROBOT_TIMR_BIAS));
    // ��ȡ��ͼ������
    update->auto_move.begin_yaw = get_yaw_positive_direction();

    //��ȡĿ������
    if (update->vision_receive_point->receive_state == UNLOADED)
    {
        //��������
        memcpy(&update->target_data, &update->vision_receive_point->receive_packet, sizeof(target_data_t));
        //������ֵ״̬��Ϊ�Ѷ�ȡ
        update->vision_receive_point->receive_state = LOADED;
    }
}

static void vision_set_robot_mode(vision_control_t* set_robot_mode)
{
    
    //������̨�������жϻ�����ģʽ
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

    //������������ɫ
    robot_armor_color_e person_armor_color;
    //�з���������ɫ
    robot_armor_color_e enemy_armor_color;
    //Ŀ����ɫ
    robot_armor_color_e target_armor_color;
    //�жϻ�����id�Ƿ����ROBOT_RED_AND_BULE_DIVIDE_VALUE���ֵ
    if (set_detect_color->robot_state_point->robot_id > ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        //�Լ�Ϊ��ɫ
        person_armor_color = BLUE;
        enemy_armor_color = RED;
    }
    else
    {
        person_armor_color = RED;
        enemy_armor_color = BLUE;
    } 

    //���ݻ�����ģʽ�ж�ʶ��Ŀ��װ�װ����ɫ
    switch (set_detect_color->robot_mode)
    {
    case FOLLOW_PERSON_ENGINEER:
    case FOLLOW_PERSON_HERO: 
    case FOLLOW_PERSON_INFANTRY_3:
    case FOLLOW_PERSON_INFANTRY_4:
    case FOLLOW_PERSON_INFANTRY_5:
        //���漺��������ʱʶ�𼺷���������ɫ
        target_armor_color = person_armor_color;
        break;

    case ATTACK_ENEMY_OUTPOST:
    case ATTACK_ENEMY_ROBOT:
        //Ϯ���Է�������ʱʶ��Է���������ɫ
        target_armor_color = enemy_armor_color; 
        break;

    default:
        //����, ʶ��Է���������ɫ
        target_armor_color = enemy_armor_color;
        break;
    } 
    //����Ŀ����ɫΪĿ����ɫ
    set_detect_color->detect_armor_color = target_armor_color;
}

static void vision_set_target_armor_num(vision_control_t* set_target_armor_num)
{
    //���ݻ�����ģʽ�ж�Ŀ��װ�װ�����
    switch (set_target_armor_num->robot_mode)
    {
        //���漺������
    case FOLLOW_PERSON_ENGINEER:
        set_target_armor_num->target_armor_id = ARMOR_ENGINEER;
        break;
        //���漺��Ӣ��
    case FOLLOW_PERSON_HERO:
        set_target_armor_num->target_armor_id = ARMOR_HERO;
        break;
        //���漺������3��
    case FOLLOW_PERSON_INFANTRY_3:
        set_target_armor_num->target_armor_id = ARMOR_INFANTRY3;
        break;
        //���漺������4��
    case FOLLOW_PERSON_INFANTRY_4:
        set_target_armor_num->target_armor_id = ARMOR_INFANTRY4;
        break;
        //���漺������5��
    case FOLLOW_PERSON_INFANTRY_5:
        set_target_armor_num->target_armor_id = ARMOR_INFANTRY5;
        break;
        //Ϯ���з�ǰ��վ
    case ATTACK_ENEMY_OUTPOST:
        set_target_armor_num->target_armor_id = ARMOR_OUTPOST;
        break;
        //Ϯ���з�������
    case ATTACK_ENEMY_ROBOT:
        set_target_armor_num->target_armor_id = ARMOR_ALL_ROBOT;
        break;
    }    
}

static void vision_judge_appear_target(vision_control_t* judge_appear_target)
{
    //���ݽ��������ж��Ƿ�Ϊʶ��Ŀ��
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
        //δʶ��Ŀ��
        judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
    }
    else
    {
        //ʶ��Ŀ��

        //���µ�ǰʱ��
        judge_appear_target->vision_receive_point->current_time = TIME_MS_TO_S(HAL_GetTick());
        //�жϵ�ǰʱ���Ƿ�����ϴν��յ�ʱ�����
        if (fabs(judge_appear_target->vision_receive_point->current_time - judge_appear_target->vision_receive_point->current_receive_time) > MAX_NOT_RECEIVE_DATA_TIME)
        {
            //�ж�Ϊδʶ��Ŀ��
            judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
        }
        else
        {
            //�ж�ʶ��Ŀ��װ�װ�����
            if (judge_appear_target->target_armor_id == ARMOR_ALL_ROBOT) //ʶ��ȫ��������
            {
                //ɸѡ�����غ�ǰ��վ
                if (judge_appear_target->target_data.id == ARMOR_BASE || judge_appear_target->target_data.id == ARMOR_OUTPOST)
                {
                    //���ò�ʶ��Ŀ��
                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                }
                else
                {

                    // ����Ϊʶ��Ŀ��
                    judge_appear_target->vision_target_appear_state = TARGET_APPEAR;

                    // �жϵ�ǰģʽΪϮ���Է�������
                    if (judge_appear_target->robot_mode == ATTACK_ENEMY_ROBOT)
                    {
                        // �ж�Ŀ���Ƿ�Ϊ�ڱ� -- �����Ƿ�Ҫ���л���
                        if (judge_appear_target->target_data.id == ARMOR_GUARD)
                        {
                            // ���з�ǰ��վ����ȥ���ڱ���һĿ��
                            // ���ݼ���װ�װ���ɫ���жϵз�ǰ��վ�Ƿ�ݻ�
                            if (judge_appear_target->detect_armor_color == BLUE)
                            {
                                // ��ɫ
                                if (judge_appear_target->game_robot_HP_point->blue_outpost_HP > 1)
                                {
                                    // �з�ǰ��վ��� -- �������ڱ�
                                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                                }
                            }
                            else if (judge_appear_target->detect_armor_color == RED)
                            {
                                // ��ɫ
                                if (judge_appear_target->game_robot_HP_point->red_outpost_HP > 1)
                                {
                                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                                }
                            }
                        }
                    }
               }
            }
            else //ʶ���ض�����
            {
                //�ж�ʶ��Ŀ��װ�װ�������Ƿ����Ŀ������
                if (judge_appear_target->target_data.id == judge_appear_target->target_armor_id)
                {
                    //����Ϊʶ��Ŀ��
                    judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
                }
                else
                {
                    //δʶ��Ŀ��
                    judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
                }


                if (judge_appear_target->robot_mode == ATTACK_ENEMY_OUTPOST)
                {
                    //����ǰ��սģʽ����Ϊʶ����������
                    //����Ϊʶ����������
                    judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
                }
            }
        }
    }
}


static void vision_data_process(vision_control_t* vision_data)
{
    //�ж��Ƿ�ʶ��Ŀ��
    if (vision_data->vision_target_appear_state == TARGET_APPEAR)
    {
        // ʶ��Ŀ��
        //  ѡ������װ�װ�
        select_optimal_target(&vision_data->solve_trajectory, &vision_data->target_data, &vision_data->target_position);
        // �����������׼λ��
        calc_robot_gimbal_aim_vector(&vision_data->robot_gimbal_aim_vector, &vision_data->target_position, vision_data->target_data.vx, vision_data->target_data.vy, vision_data->target_data.vz, vision_data->solve_trajectory.predict_time);
        // ���������pitch����yaw��Ƕ�
        vision_data->gimbal_vision_control.gimbal_pitch = calc_target_position_pitch_angle(&vision_data->solve_trajectory, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)) - vision_data->solve_trajectory.distance_static, vision_data->robot_gimbal_aim_vector.z + vision_data->solve_trajectory.z_static);
        vision_data->gimbal_vision_control.gimbal_yaw = atan2(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);

        // ���ݻ�����ģʽ��ֵ�����Լ��˶���������
        switch (vision_data->robot_mode)
        {
        case FOLLOW_PERSON_ENGINEER:
        case FOLLOW_PERSON_HERO:
        case FOLLOW_PERSON_INFANTRY_3:
        case FOLLOW_PERSON_INFANTRY_4:
        case FOLLOW_PERSON_INFANTRY_5:
            {
                // ���ò��������Ǹ���
                vision_data->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
                // ���õ���ģʽΪ����ģʽ
                vision_data->chassis_vision_control.vision_control_chassis_mode = FOLLOW_TARGET;
                // ��ֵ���̿������� -- �ҷ���������Ŀ��ľ���
                vision_data->chassis_vision_control.distance = sqrt(pow(vision_data->target_data.x, 2) + pow(vision_data->target_data.y, 2));
            }
            break;
        case ATTACK_ENEMY_OUTPOST:
        case ATTACK_ENEMY_ROBOT:
            {
                //�����жϷ�����������
                // �жϷ���
                vision_shoot_judge(vision_data, vision_data->gimbal_vision_control.gimbal_yaw - vision_data->imu_absolution_angle.yaw, vision_data->gimbal_vision_control.gimbal_pitch - vision_data->imu_absolution_angle.pitch, sqrt(pow(vision_data->target_data.x, 2) + pow(vision_data->target_data.y, 2)));
                // ���õ���ģʽΪ������
                vision_data->chassis_vision_control.vision_control_chassis_mode = UNFOLLOW_TARGET;
                // ��ֵ���̿������� -- 0
                vision_data->chassis_vision_control.distance = 0;
            }
            break;
        }

    }
    else
    {
        //δʶ��Ŀ�� -- ����ֵ����
        vision_data->gimbal_vision_control.gimbal_yaw = 0;
        vision_data->gimbal_vision_control.gimbal_pitch = 0;
        vision_data->chassis_vision_control.distance = 0;
        vision_data->chassis_vision_control.vision_control_chassis_mode = UNFOLLOW_TARGET;
        //����ֹͣ����
        vision_data->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

    vision_data->gimbal_vision_control.robot_mode = vision_data->robot_mode;
}



/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з��䣬�ж�yaw��pitch�ĽǶȣ������һ����Χ�ڣ������ֵ���ӣ����ӵ�һ����ֵ���жϷ��䣬���yaw��pitch��Ƕȴ��ڸ÷�Χ�����������
 * 
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 * @param target_distance Ŀ�����
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance)
{ 
    //�ж�Ŀ�����
    if (target_distance <= ALLOW_ATTACK_DISTANCE)
    {
        // �жϼ��Ƿ�С������ֵ
        if (shoot_judge->vision_receive_point->receive_packet.p < ALLOE_ATTACK_P)
        {
            // С��һ�Ƕȿ�ʼ����
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
        //Զ���벻����
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
 * @brief �����������¶�Ӱ�죬���ٿ��ܻ���ֱ���ź��ϲ����ϴ�ı仯������ʵʱ���㵯�٣��������ȵ�����
 * 
 * @param calc_cur_bullet_speed �Ӿ����ƽṹ��
 * @param bullet_type �ӵ�����
 * @param shooter_id ǹ��id
 */
static void calc_current_bullet_speed(vision_control_t* calc_cur_bullet_speed, bullet_type_e bullet_type, shooter_id_e shooter_id)
{
    //�ж��ӵ�����
    if (calc_cur_bullet_speed->shoot_data_point->bullet_type == bullet_type)
    {
        if (calc_cur_bullet_speed->shoot_data_point->shooter_id == shooter_id)
        {
            //ɸѡ�����������
            if (calc_cur_bullet_speed->shoot_data_point->bullet_speed >= MIN_SET_BULLET_SPEED && calc_cur_bullet_speed->shoot_data_point->bullet_speed <= MAX_SET_BULLET_SPEED)
            {
                //�������� -- �������ȵ�����
                calc_cur_bullet_speed->bullet_speed = (int16_t)calc_cur_bullet_speed->shoot_data_point->bullet_speed;
            }
        }
    }
}


// static void calc_auto_move_data(vision_control_t* robot_auto_move)
// {
//     fp32 relative_yaw_angle = 0;
//     // ��ȡ��ǰλ��
//     get_robot_cur_pos(robot_auto_move);
//     //��ȡ������Ŀ��λ��
//     get_robot_target_pos(robot_auto_move);
//     //�������yaw��Ƕ�
//     relative_yaw_angle = atan2(robot_auto_move->auto_move.target_pos.y - robot_auto_move->auto_move.cur_pos.y, robot_auto_move->auto_move.target_pos.x - robot_auto_move->auto_move.cur_pos.x);

//     robot_auto_move->auto_move.command_yaw = rad_format(relative_yaw_angle + robot_auto_move->auto_move.begin_yaw);

//     //��ֵ�˶��ٶ� = ���� * ϵ��
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
//     //�ж��Ƿ����Զ��ƶ�ģʽ
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
    //���CRC16����β
    append_CRC16_check_sum((uint8_t*)&send->send_packet, sizeof(send->send_packet));
    //��������
    CDC_Transmit_FS((uint8_t*)&send->send_packet, sizeof(send->send_packet));
}

void receive_decode(uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len < 2)
    {
        return;
    }
    //CRCУ��
    if (verify_CRC16_check_sum(buf, len))
    {
        receive_packet_t temp_packet = {0};
        // �������յ������ݵ���ʱ�ڴ���
        memcpy(&temp_packet, buf, sizeof(receive_packet_t));
        if (temp_packet.header == HIGH_TO_LOWER_HEAD)
        {
            // ������ȷ������ʱ���ݿ������������ݰ���
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            // ������������״̬��־Ϊδ��ȡ
            vision_receive.receive_state = UNLOADED;

            // ����ʱ��
            vision_receive.last_receive_time = vision_receive.current_receive_time;
            // ��¼��ǰ�������ݵ�ʱ��
            vision_receive.current_receive_time = TIME_MS_TO_S(HAL_GetTick());
            //����ʱ����
            vision_receive.interval_time = vision_receive.current_receive_time - vision_receive.last_receive_time;
        }
    }
}

/**
 * @brief ��ʼ����������Ĳ���
 * 
 * @param solve_trajectory ��������ṹ��
 * @param k1 ��������
 * @param init_flight_time ��ʼ����ʱ�����ֵ
 * @param time_bias ���м��ʱ��
 * @param z_static yaw������ǹ��ˮƽ��Ĵ�ֱ����
 * @param distance_static ǹ��ǰ�ƾ��� 
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
 * @brief ��ֵ���������һЩ�ɱ����
 *
 * @param solve_trajectory ��������ṹ��
 * @param current_pitch ��ǰ��̨��pitch
 * @param current_yaw ��ǰ��̨��yaw
 * @param current_bullet_speed ��ǰ����
 * @param time_bias ��ǰʱ��ƫ��ֵ
 */
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed, fp32 time_bias)
{
    solve_trajectory->current_yaw = current_yaw;
    solve_trajectory->current_pitch = current_pitch;
    solve_trajectory->current_bullet_speed = current_bullet_speed;
    solve_trajectory->time_bias = time_bias;
}

/**
 * @brief ѡ�����Ż���Ŀ��
 * 
 * @param solve_trajectory ��������ṹ�� 
 * @param vision_data �����Ӿ�����
 * @param optimal_target_position ����Ŀ��λ�� 
 */
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position)
{
    //����Ԥ��ʱ�� = ��һ�ε��ӵ�����ʱ�� + ����ƫ��ʱ��, ʱ����ܲ���ȷ�������Խ���
    solve_trajectory->predict_time = solve_trajectory->flight_time + solve_trajectory->time_bias;
    //�����ӵ�����Ŀ��ʱ��yaw�Ƕ�
    solve_trajectory->target_yaw = vision_data->yaw + vision_data->v_yaw * solve_trajectory->predict_time;

    //��ֵװ�װ�����
    solve_trajectory->armor_num = vision_data->armors_num;
    
    //����װ�װ�������λ�ñ����Ŀռ�
    if (solve_trajectory->all_target_position_point == NULL)
    {
       solve_trajectory->all_target_position_point = malloc(solve_trajectory->armor_num * sizeof(target_position_t));
    }

    //ѡ��Ŀ���������
    uint8_t select_targrt_num = 0;
    
    //��������װ�װ��λ��
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        //����װ�װ������������ľ��벻ͬ������һ�������Գƣ����Խ��м���װ�װ�λ��ʱ����0 2���õ�ǰ�뾶����1 3������һ�ΰ뾶
        fp32 r = (i % 2 == 0) ? vision_data->r1 : vision_data->r2;
        solve_trajectory->all_target_position_point[i].yaw = solve_trajectory->target_yaw + i * (ALL_CIRCLE / solve_trajectory->armor_num);
        solve_trajectory->all_target_position_point[i].x = vision_data->x - r * cos(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].y = vision_data->y - r * sin(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].z = (i % 2 == 0) ? vision_data->z : vision_data->z + vision_data->dz;
    }

    // ѡ�������������yaw��ֵ��С��Ŀ��,����ѡ����СĿ��
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
    // ��ѡ���װ�װ����ݣ�����������Ŀ����ȥ
    memcpy(optimal_target_position, &solve_trajectory->all_target_position_point[select_targrt_num], sizeof(target_position_t));
    //�ͷſ��ٵ��ڴ�
    free(solve_trajectory->all_target_position_point);
    //ָ���ÿ�
    solve_trajectory->all_target_position_point = NULL;
}

/**
 * @brief ����װ�װ���׼λ��
 * 
 * @param robot_gimbal_aim_vector ��������̨��׼����
 * @param target_position Ŀ��λ��
 * @param vx �����������ٶ�
 * @param vy �����������ٶ�
 * @param vz �����������ٶ�
 * @param predict_time Ԥ��ʱ��
 */
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time)
{
    //����Ŀ����۲����Ĵ���ͬһϵ���ٶ���ͬ
    robot_gimbal_aim_vector->x = target_position->x + vx * predict_time;
    robot_gimbal_aim_vector->y = target_position->y + vy * predict_time;
    robot_gimbal_aim_vector->z = target_position->z + vz * predict_time;
}



/**
 * @brief ���㵯����� -- ��ȫ��������ģ�� ��ģ�������ڴ����ǻ���Ļ���
 * 
 * @param solve_trajectory ��������ṹ��
 * @param x ����
 * @param bullet_speed ����
 * @param pitch ����
 * @return �������
 */
static float calc_bullet_drop_in_complete_air(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch)
{
    //�ӵ����߶�
    fp32 bullet_drop_z = 0;
    //�����ܷ���ʱ��
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(pitch)));
    
    if (pitch > 0) 
    {
        //������������ϵ�� ����ֱ����
        //���������� �ӵ��ٶȷ��������ĽǶ���������0����ֱ�������� hat(f_z) = f_z * sin(pitch) ���������� ��ˮƽ�������� hat(f_x) = f_x * cos(pitch) �������� f_x ������Ҫ����ֱ��������ϵ�����в���
        fp32 k_z = solve_trajectory->k1 * (1 / sin(pitch));
        // ������
        // ��ʼ��ֱ�����ٶ�
        fp32 v_z_0 = bullet_speed * sin(pitch);
        // ����������������ʱ��
        fp32 max_flight_up_time = (1 / sqrt(k_z * GRAVITY)) * atan(sqrt(k_z / GRAVITY) * v_z_0);
        // �ж��ܷ���ʱ���Ƿ�С������������ʱ��
        if (solve_trajectory->flight_time <= max_flight_up_time)
        {
            // �ӵ�����������
            bullet_drop_z = (1 / k_z) * log(cos(sqrt(k_z * GRAVITY) * (max_flight_up_time - solve_trajectory->flight_time)) / cos(sqrt(k_z * GRAVITY) * max_flight_up_time));
        }
        else
        {
            // ���������������ʱ�� -- �����½���
            // �������߶�
            fp32 z_max = (1 / (2 * k_z)) * log(1 + (k_z / GRAVITY) * pow(v_z_0, 2));
            // �����½�
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
 * @brief ��άƽ�浯��ģ�ͣ�����pitch��ĸ߶�
 * 
 * @param solve_tragectory ��������ṹ��
 * @param x ˮƽ����
 * @param z ��ֱ����
 * @param bullet_speed ����
 * @return ����pitch����ֵ
 */
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z)
{
    // �������߶�
    float bullet_drop_z = 0;
    // ��׼�߶�
    float aim_z = z;

    // ����
    float pitch = 0;
    // ����ֵ����ʵֵ֮������
    float calc_and_actual_error = 0;
    // ����������
    for (int i = 0; i < MAX_ITERATE_COUNT; i++)
    {
        // ��������
        pitch = atan2(aim_z, x);
        // �����ӵ����߶�
        bullet_drop_z = calc_bullet_drop_in_complete_air(solve_trajectory, x, solve_trajectory->current_bullet_speed, pitch);
        // �������
        calc_and_actual_error = z - bullet_drop_z;
        // ����׼�߶Ƚ��в���
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // �ж�����Ƿ���Ͼ���Ҫ��
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
    //����Ϊ����ϵ��pitchΪ����Ϊ���������ø�
    return -pitch;
}

static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}

//��ȡ��ǰ�Ӿ��Ƿ�ʶ��Ŀ��
bool_t judge_vision_appear_target(void)
{
    return vision_control.vision_target_appear_state == TARGET_APPEAR;
}

//�жϵ�ǰ������ģʽ�Ƿ�Ϊ�Զ��ƶ�ģʽ
bool_t judge_cur_mode_is_auto_move_mode(void)
{
    // return vision_control.robot_mode == AUTO_MOVE_TARGET_POINT;
    return 0;
}

// ��ȡ��λ����̨����
const gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// ��ȡ��λ����������
const shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}

//��ȡ���̿�������
const chassis_vision_control_t* get_vision_chassis_point(void)
{
    return &vision_control.chassis_vision_control;
}

//��ȡ�Զ���������ָ��
const auto_move_t* get_auto_move_point(void)
{
    return &vision_control.auto_move;
}


queue_t* queue_create(int16_t capacity)
{
    //�����ռ�
    queue_t* queue = malloc(sizeof(queue_t));
    if (queue == NULL)
    {
        return NULL;
    }
    //��ֵ���
    memset(queue, 0, sizeof(queue_t));
    //��ֵ����
    queue->capacity = capacity;
    //���ٿռ�
    queue->date = malloc(queue->capacity * sizeof(fp32));
    //��ֵ��ǰ�洢ֵ
    queue->cur_size = 0; 
    return queue;
}


void queue_append_data(queue_t* queue, fp32 append_data)
{
    
    if (queue->date != NULL && queue->capacity != 0)
    {
        //�ж��Ƿ�����
        if (queue->cur_size < queue->capacity)
        {
            //δ�� -- �������

            //���ݺ���
            for (int i = (queue->cur_size - 1); i >= 0; i--)
            {
                queue->date[i + 1] = queue->date[i];   
            }

            //�����������
            queue->date[0] = append_data;

            //�洢����1
            queue->cur_size += 1;
        }
        else
        {
            // ���� -- ������һλ����

            //���ݺ��ƶ�
            for (int i = queue->cur_size - 2; i >= 0; i--)
            {
                queue->date[i + 1] = queue->date[i];
            }

            //�����������
            queue->date[0] = append_data;

            //�洢������
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
        //�������⣬����-1
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
        //�����ݷ���0
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
    //�ͷŴ洢�ռ�
    free(queue->date);
    queue->date = NULL;
    //�ͷ�����ռ�
    free(queue);
    queue = NULL;
}

