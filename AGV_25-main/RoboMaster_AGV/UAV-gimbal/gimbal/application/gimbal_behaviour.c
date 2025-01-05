/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================

    如果要添加一个新的行为模式
    1.首先，在gimbal_behaviour.h文件中， 添加一个新行为名字在 gimbal_behaviour_e
    erum
    {
        ...
        ...
        GIMBAL_XXX_XXX, // 新添加的
    }gimbal_behaviour_e,

    2. 实现一个新的函数 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" 参数是云台运动控制输入量
        第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
    3.  在"gimbal_behavour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
        在gimbal_behaviour_mode_set函数最后，添加"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,然后选择一种云台控制模式
        3种:
        GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
    4.  在"gimbal_behaviour_control_set" 函数的最后，添加
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "user_lib.h"
#include "bsp_usart.h"
#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
 * @brief          通过判断角速度来判断云台是否到达极限位置
 */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief          云台控制，电机是角度控制，
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 */
static void gimbal_RC_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 * @author         RM
 */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief                     云台进入自动模式，云台姿态受视觉上位机控制，电机是绝对角度控制
 *
 * @param yaw                 yaw轴角度增量
 * @param pitch               pitch轴角度增量
 * @param gimbal_control_set  云台数据指针
 * @author                    yuanluochen
 */
static void gimbal_auto_attack_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
 * @brief                     云台进入自动扫描模式，云台yaw轴pitch轴浮动扫描
 *
 * @param yaw                 yaw 轴角度增量
 * @param pitch               pitch 轴角度增量
 * @param gimbal_control_set  云台指针
 */
static void gimbal_auto_scan_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
 * @brief                     云台进入自动移动模式，云台姿态受命令，电机是绝对角度控制
 *
 * @param yaw                 yaw 轴角度增量
 * @param pitch               pitch 轴角度增量
 * @param gimbal_control_set  云台指针
 */
static void gimbal_auto_move_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/*----------------------------------结构体---------------------------*/
// 云台行为状态机
// static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
/*----------------------------------内部变量---------------------------*/
int yaw_flag = 0;
fp32 Pitch_Set[8] = {0};
/*----------------------------------外部变量---------------------------*/
// 云台初始化完毕标志位
bool_t gimbal_init_finish_flag = 0;
extern vision_rxfifo_t *vision_rx;

/**
 * @brief          gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
 * @param[out]     gimbal_mode_set: 云台数据指针
 * @retval         none
 */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    // set gimbal_behaviour variable
    // 云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);
    // accoring to gimbal_behaviour, set motor control mode
    // 根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        // 无力模式下,设置为电机原始值控制，方便让电机处于无力态
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_RC)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;   // yaw轴通过陀螺仪的绝对角控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO; // pitch轴通过陀螺仪的绝对角控制
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;   // yaw轴通过陀螺仪的绝对角控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO; // pitch轴通过陀螺仪的绝对角控制
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_SCAN)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;   // yaw轴通过陀螺仪的绝对角控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO; // pitch轴通过陀螺仪的绝对角控制
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;   // yaw轴通过陀螺仪的绝对角控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO; // pitch轴通过陀螺仪的绝对角控制
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
 * @brief                          (修改版)云台行为模式控制，云台pitch轴采用相对角度控制，云台yaw轴采用绝对角度控制,原因为pitch绝对角难以限制
 *
 * @param add_yaw                  yaw 轴的角度增量 为指针
 * @param add_pitch                pitch 轴的角度增量 为指针
 * @param gimbal_control_set       云台结构体指针
 */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    // 判断云台模式，根据云台模式选择云台控制方式
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE) // 无力模式云台无力
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT) // 初始化模式，云台初始化
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RC) // 遥控器控制模式,绝对角控制
    {
        gimbal_RC_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS) // 无信号下的控制,即无力
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_ATTACK) // 自动袭击模式
    {
        gimbal_auto_attack_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_SCAN) // 自动扫描模式
    {
        gimbal_auto_scan_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_MOVE) // 自动跟随模式
    {
        gimbal_auto_move_control(add_yaw, add_pitch, gimbal_control_set);
    }
}

/**
 * @brief          云台在某些行为下，需要底盘不动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    // 判断是否为初始化模式
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        // 初始化时间
        static int init_time = 0;
        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            // 停止初始化
            init_time = 0;
        }
        else
        {
            init_time++; // 初始化时间增加
            // 是否初始化完成
            if ((fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) > GIMBAL_INIT_ANGLE_ERROR) ||
                (abs(gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd - gimbal_mode_set->gimbal_yaw_motor.frist_ecd) * MOTOR_ECD_TO_RAD > GIMBAL_INIT_ANGLE_ERROR))
            {
                // 初始化未完成，判断初始化时间
                if (init_time >= GIMBAL_INIT_TIME)
                {
                    // 不进行任何行为，直接判断进入其他模式,计时归零
                    init_time = 0;
                }
                else
                {
                    // 退出模式选择，依旧为初始化模式
                    return;
                }
            }
            else
            {
                // 初始化次数
                static int init_finish_count = 0;
                init_finish_count++;

                // 初始化完成,计时归零,重新计时
                init_time = 0;
                // 标志初始化完成
                gimbal_init_finish_flag = 1;
                if (init_finish_count == 1)
                {
                    // 保留数据 -- 记录场地正方向
                    gimbal_mode_set->yaw_positive_direction = gimbal_mode_set->gimbal_yaw_motor.absolute_angle_set;
                }
            }
        }
    }
    static int8_t press_r_last_s = 0;
    static int16_t last_key_G = 0;
    static int16_t move = 0;

    //  if (!last_key_G && gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G)
    //  {
    //    move = !move;
    //  }
    //
    static int mode = 0;
    if (press_r_last_s && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r)
    {
        mode = 3; // gimbal_mode_set->right_click_time++;
    }
    else
    {
        mode = 1;
    }
    //  static int mode = 0;
    //  if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
    //  {
    //    mode = 1;
    //  }
    //  if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
    //  {
    //    mode = 2;
    //  }
    //   if (gimbal_mode_set->right_click_time>40)
    //  {
    //    mode = 3;
    //  }
    //
    //    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    //    {
    //        // 遥控器控制模式
    //        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    //    }
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        if (mode == 3)
        {
            gimbal_behaviour = GIMBAL_AUTO_ATTACK;
        }
        else if (mode == 1)
        {
            gimbal_behaviour = GIMBAL_RC;
        }
        else if (mode == 2)
        {
            gimbal_behaviour = GIMBAL_RC;

            if (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)
            {
                gimbal_behaviour = GIMBAL_RC;
            }
        }
        //    else
        //    {
        //      gimbal_behaviour = GIMBAL_ZERO_FORCE;
        //    }
    }
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        // 切换到遥控器控制模式
        gimbal_behaviour = GIMBAL_RC;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {

        gimbal_behaviour = GIMBAL_AUTO_ATTACK;
        // // 切换到云台自动模式
        // // 判断当前模式是否为自动移动模式
        // if (judge_cur_mode_is_auto_move_mode())
        // {
        //     //是自动移动模式
        //     gimbal_behaviour = GIMBAL_AUTO_MOVE;  //云台自动移动模式
        // }
        // else
        // {
        //     // 不是自动移动模式
        //     // 根据视觉是否识别，自动控制模式
        //     if (judge_vision_appear_target())
        //     {
        //         // 识别到目标
        //         gimbal_behaviour = GIMBAL_AUTO_ATTACK; // 云台自动袭击模式
        //     }
        //     else
        //     {
        //         // 未识别到目标
        //         gimbal_behaviour = GIMBAL_AUTO_SCAN; // 云台自动扫描模式
        //     }
        // }
    }

    // 遥控器报错处理
    if (toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    // 判断进入初始化模式
    static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
    // 判断云台从无力模式转为其他模式,进入初始化状态
    if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
    {
        gimbal_behaviour = GIMBAL_INIT;
        // 标志初始化未完成
        gimbal_init_finish_flag = 0;
    }

    // 判断是否发生云台从其他模式切入自动扫描模式模式
    //    if (last_gimbal_behaviour != GIMBAL_AUTO_SCAN && gimbal_behaviour == GIMBAL_AUTO_SCAN)
    //    {
    //        // 用于自动扫描 -- 重新初始化扫描初始时间
    //        gimbal_mode_set->gimbal_auto_scan.scan_begin_time = TIME_MS_TO_S(HAL_GetTick());
    //        // 用于自动扫描 -- 重新设置云台yaw轴扫描中点，为当前位置
    //        gimbal_mode_set->gimbal_auto_scan.yaw_center_value = gimbal_mode_set->gimbal_yaw_motor.absolute_angle;
    //        gimbal_mode_set->gimbal_auto_scan.pitch_center_value = gimbal_mode_set->gimbal_pitch_motor.absolute_angle;
    //    }
    // 保存历史数据
    last_gimbal_behaviour = gimbal_behaviour;
    press_r_last_s = gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r;
}
/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
 * @brief          云台控制，电机是角度控制，遥控器控制数据
 */

static void gimbal_RC_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static int16_t yaw_channel_RC, yaw_channel;
    static int16_t pitch_channel_RC, pitch_channel;
    static fp32 rc_add_yaw, rc_add_yaw_RC;
    static fp32 rc_add_pit, rc_add_pit_RC;
    // 遥控器控制
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel_RC, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel_RC, RC_DEADBAND);

    rc_add_yaw_RC = yaw_channel_RC * YAW_RC_SEN;
    rc_add_pit_RC = -pitch_channel_RC * PITCH_RC_SEN;

    // 键盘控制
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.x, yaw_channel, 0);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.y, pitch_channel, 0);

    rc_add_yaw = -yaw_channel * YAW_MOUSE_SEN;
    rc_add_pit = pitch_channel * PITCH_MOUSE_SEN;


    //导航行走时设置的低速模式
    if (vision_rx->ang_z != 0)
    {
        *yaw = ((vision_rx->ang_z) * 0.000264f) * 1; // rc_add_yaw  +rc_add_yaw_RC *0.8;
        *pitch = (rc_add_pit + rc_add_pit_RC) * 0.8;
    }
    else
    {
        *yaw = rc_add_yaw + rc_add_yaw_RC * 0.8;
        *pitch = (rc_add_pit + rc_add_pit_RC) * 0.8;
    }
}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief                     云台进入自动袭击模式，云台姿态受视觉上位机控制，电机是绝对角度控制
 *
 * @param yaw                 yaw 轴角度增量
 * @param pitch               pitch 轴角度增量
 * @param gimbal_control_set  云台指针
 */
static void gimbal_auto_attack_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    // yaw pitch 轴设定值与当前值的差值
    fp32 pitch_error = 0;
    fp32 yaw_error = 0;

    // pitch轴yaw轴设定角度
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;

    // 计算过去设定角度与当前角度之间的差值
    yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    pitch_error = gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;
    //  获取上位机视觉数据
    pitch_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_pitch;
    yaw_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_yaw;
    // 赋值增量
    if (gimbal_control_set->gimbal_vision_point->gimbal_pitch == 0 || gimbal_control_set->gimbal_vision_point->gimbal_yaw == 0)
    {
        *yaw = 0.0f;
        *pitch = 0.0f;
    }
    else
    {
        *yaw = yaw_set_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle - yaw_error;
        *pitch = pitch_set_angle - gimbal_control_set->gimbal_pitch_motor.absolute_angle - pitch_error;
    }
}

/**
 * @brief                     云台进入自动扫描模式，云台yaw轴pitch轴浮动扫描
 *
 * @param yaw                 yaw 轴角度增量
 * @param pitch               pitch 轴角度增量
 * @param gimbal_control_set  云台指针
 */
static void gimbal_auto_scan_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    // yaw pitch 轴设定值与当前值的差值
    fp32 pitch_error = 0;
    fp32 yaw_error = 0;

    // pitch轴yaw轴设定角度
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;

    // 视觉模式判断pitch轴扫描中值
    if (gimbal_control_set->gimbal_vision_point->robot_mode == ATTACK_ENEMY_OUTPOST)
    {
        gimbal_control_set->gimbal_auto_scan.pitch_center_value = -gimbal_control_set->gimbal_auto_scan.pitch_range + 0.05f;
    }
    else
    {
        gimbal_control_set->gimbal_auto_scan.pitch_center_value = 0.05;
    }
    // 计算过去设定角度与当前角度之间的差值
    yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    pitch_error = gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;

    // 自动扫描设置浮动值
    fp32 auto_scan_AC_set_yaw = 0;
    fp32 auto_scan_AC_set_pitch = 0;
    // 计算运行时间
    gimbal_control_set->gimbal_auto_scan.scan_run_time = TIME_MS_TO_S(HAL_GetTick()) - gimbal_control_set->gimbal_auto_scan.scan_begin_time;
    // 云台自动扫描,设置浮动值
    scan_control_set(&auto_scan_AC_set_yaw, gimbal_control_set->gimbal_auto_scan.yaw_range, gimbal_control_set->gimbal_auto_scan.scan_yaw_period, gimbal_control_set->gimbal_auto_scan.scan_run_time);
    scan_control_set(&auto_scan_AC_set_pitch, gimbal_control_set->gimbal_auto_scan.pitch_range, gimbal_control_set->gimbal_auto_scan.scan_pitch_period, gimbal_control_set->gimbal_auto_scan.scan_run_time);
    // 赋值控制值  = 中心值 + 加上浮动函数
    yaw_set_angle = auto_scan_AC_set_yaw + gimbal_control_set->gimbal_auto_scan.yaw_center_value;
    pitch_set_angle = auto_scan_AC_set_pitch + gimbal_control_set->gimbal_auto_scan.pitch_center_value;

    // 一阶低通使数据平滑
    first_order_filter_cali(&gimbal_control_set->gimbal_auto_scan.yaw_auto_scan_first_order_filter, yaw_set_angle);
    first_order_filter_cali(&gimbal_control_set->gimbal_auto_scan.pitch_auto_scan_first_order_filter, pitch_set_angle);

    // pitch_set_angle = gimbal_control_set->gimbal_auto_scan.pitch_center_value;
    // yaw_set_angle = gimbal_control_set->gimbal_auto_scan.yaw_center_value;

    // 赋值增量
    *yaw = gimbal_control_set->gimbal_auto_scan.yaw_auto_scan_first_order_filter.out - gimbal_control_set->gimbal_yaw_motor.absolute_angle - yaw_error;
    *pitch = gimbal_control_set->gimbal_auto_scan.pitch_auto_scan_first_order_filter.out - gimbal_control_set->gimbal_pitch_motor.absolute_angle - pitch_error;
}

/**
 * @brief                     云台进入自动移动模式，云台姿态受命令，电机是绝对角度控制
 *
 * @param yaw                 yaw 轴角度增量
 * @param pitch               pitch 轴角度增量
 * @param gimbal_control_set  云台指针
 */
static void gimbal_auto_move_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    // yaw pitch 轴设定值与当前值的差值
    fp32 pitch_error = 0;
    fp32 yaw_error = 0;

    // pitch轴yaw轴设定角度
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;

    // 计算过去设定角度与当前角度之间的差值
    yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    pitch_error = gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;
    //  获取上位机视觉数据
    yaw_set_angle = gimbal_control_set->auto_move_point->command_yaw;
    pitch_set_angle = 0.0f;

    // 赋值增量
    *yaw = yaw_set_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle - yaw_error;
    *pitch = pitch_set_angle - gimbal_control_set->gimbal_pitch_motor.absolute_angle - pitch_error;
}

/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //    // 初始化状态控制量计算
    //    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR) // pitch轴回正
    //    {
    //        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
    //        *yaw = 0.0f;
    //    }
    //    else // yaw轴回归初始值
    //    {
    //        static fp32 yaw_error = 0;
    //        yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    //        // pitch轴保持不变yaw轴回归中值
    //        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
    //        // yaw轴绝对角计算控制yaw轴正方向
    //        *yaw = (gimbal_control_set->gimbal_yaw_motor.frist_ecd - gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd) * MOTOR_ECD_TO_RAD - yaw_error;
    //    }
    // 初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

void scan_control_set(fp32 *gimbal_set, fp32 range, fp32 period, fp32 run_time)
{
    // 计算单次运行的步长
    fp32 step = 4.0f * range / period;

    // 判断云台设置浮动角度是否超过最大值,限制最大值
    if (*gimbal_set >= range)
    {
        *gimbal_set = range;
    }
    else if (*gimbal_set <= -range)
    {
        *gimbal_set = -range;
    }

    // 处理运行时间，将运行时间处理到一个周期内
    fp32 calc_time = run_time - period * ((int16_t)(run_time / period));
    // 判断当前时间所处的位置，根据当前位置，判断数值计算方向
    if (calc_time < 0.25f * period)
    {
        *gimbal_set = step * calc_time;
    }
    else if (0.25f * period <= calc_time && calc_time < 0.75f * period)
    {
        *gimbal_set = -(step * calc_time) + 2 * range;
    }
    else if (0.75f * period <= calc_time)
    {
        *gimbal_set = step * calc_time - 4 * range;
    }
}

bool_t gimbal_control_vision_task(void)
{
    return gimbal_init_finish_flag;
}
