/**
 * @file chassis_task.c
 * @author yuanluochen
 * @brief 双板步兵底盘控制任务，读取遥控器数据，通过can总线发送到底盘
 * @version 0.1
 * @date 2023-09-19
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "can_comm_task.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "bsp_usart.h"
#define abs(x) ((x) > 0 ? (x) : (-x))

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
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
 * @brief 底盘数据更新
 *
 * @param chassis_move_feedback_update  底盘控制结构体
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_feedback_update);

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief 设置底盘控制量
 *
 * @param chassis_move_control 底盘控制结构体
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

extern gimbal_control_t gimbal_control;
extern vision_rxfifo_t *vision_rx;
int anglesr;
extern ext_game_robot_state_t robot_state;
chassis_move_t chassis_move; // 底盘运动数据
/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    if (can_comm_task_init_finish())
    {
        // 底盘初始化
        chassis_init(&chassis_move);
        // 判断底盘电机是否都在线
        while (1)
        {
            // 数据更新
            chassis_feedback_update(&chassis_move);
            // 设置底盘控制模式
            chassis_set_mode(&chassis_move);
            // 底盘控制量设置
            chassis_set_contorl(&chassis_move);

            if (toe_is_error(DBUS_TOE) || robot_state.power_management_chassis_output == 0)
            {
                // 当遥控器离线发送控制信号为零
                can_comm_board(0, 0, 0, 0);
            }
            else
            {
                // 发送控制数据
                can_comm_board(chassis_move.chassis_relative_ecd, chassis_move.vx_set, chassis_move.vy_set, chassis_move.chassis_behaviour);
                can_comm_referee((int16_t)(power_heat_data_t.chassis_power * 100), power_heat_data_t.chassis_power_buffer,
                                 gimbal_control.CAP_Output, robot_state.chassis_power_limit);
								CAN_CMD_cap((int16_t)(gimbal_control.gimbal_pitch_motor.relative_angle*100),(int16_t)(gimbal_control.gimbal_pitch_motor.absolute_angle*100),(int16_t)chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL);
						}
            // 系统延时
            vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
            chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
    }
}

/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    // 获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    // 获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    // 数据更新
    chassis_feedback_update(chassis_move_init);
}

/**
 * @brief 底盘数据更新
 *
 * @param chassis_move_feedback_update  底盘控制结构体
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_feedback_update)
{
    chassis_move_feedback_update->chassis_relative_ecd = chassis_move_feedback_update->chassis_yaw_motor->gimbal_motor_measure->ecd - chassis_move_feedback_update->chassis_yaw_motor->frist_ecd;
    if (chassis_move_feedback_update->chassis_relative_ecd > 4096)
        chassis_move_feedback_update->chassis_relative_ecd -= 8191;
    else if (chassis_move_feedback_update->chassis_relative_ecd < -4096)
        chassis_move_feedback_update->chassis_relative_ecd += 8191;
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    static int mode = RC_Control;
    static int last_Key_F = 0;
    static int chassis_spin_flag = 0;

    if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_B)
    {
        mode = RUDDER_FOLLOW_GIMBAL;
    }
    if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
    {
        mode = CHASSIS_FOLLOW_GIMBAL;
    }

    // F一键小陀螺
    if (!last_Key_F && chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
    {
        mode = CHASSIS_SPIN;
        chassis_spin_flag = !chassis_spin_flag;
    }
    last_Key_F = chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_F;

    
    // remote control  set chassis behaviour mode
    // 遥控器设置模式
    if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        // 遥控器拨到下侧挡位为底盘无力模式
        chassis_move_mode->chassis_behaviour = CHASSIS_ZERO_FORCE;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) || switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        if (mode == RC_Control)
        {
            // 遥控器中挡以及上档为底盘有力模式
            if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
            {
                // 舵跟随云台
                chassis_move_mode->chassis_behaviour = CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW;
            }
            else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
            {
                // 底盘跟随云台
                chassis_move_mode->chassis_behaviour = CHASSIS_FOLLOW_GIMBAL_YAW;
            }
            else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
            {
                // 陀螺
                chassis_move_mode->chassis_behaviour = CHASSIS_SPIN;
            }
        }
        else
        {
            if (mode == RUDDER_FOLLOW_GIMBAL)
            {
                chassis_move_mode->chassis_behaviour = CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW;
            }
            else if (mode == CHASSIS_FOLLOW_GIMBAL)
            {
                chassis_move_mode->chassis_behaviour = CHASSIS_FOLLOW_GIMBAL_YAW;
            }
            else if (chassis_spin_flag)
            {
                chassis_move_mode->chassis_behaviour = CHASSIS_SPIN;
            }
            else
            {
                chassis_move_mode->chassis_behaviour = CHASSIS_FOLLOW_GIMBAL_YAW;
            }

            if (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)  //按shift超电加速
            {
                gimbal_control.CAP_Output = CAP_OUTPUT_to_CHASSIS;
            }
						else if(chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_E)  //飞坡模式
						{
								gimbal_control.CAP_Output = CAP_OUTPUT_to_CHASSIS_FLY;
						}
            else
            {
                gimbal_control.CAP_Output = 0;
            }
        }
    }
    else if (toe_is_error(DBUS_TOE))
    {
        chassis_move_mode->chassis_behaviour = CHASSIS_NO_MOVE;
    }
}

/**
 * @brief 设置底盘控制量
 *
 * @param chassis_move_control 底盘控制结构体
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    fp32 vx_set_channel_RC, vy_set_channel_RC;
    int16_t vx_channel_RC, vy_channel_RC;

    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadband_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel_RC, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel_RC, CHASSIS_RC_DEADLINE);

    vx_set_channel_RC = vx_channel_RC * CHASSIS_VX_RC_SEN;
    vy_set_channel_RC = vy_channel_RC * CHASSIS_VY_RC_SEN;

    // 设置速度
    chassis_move_control->vx_set += vx_set_channel_RC;
    chassis_move_control->vy_set += vy_set_channel_RC;

    if (chassis_move_control->chassis_RC->key.v & KEY_PRESSED_OFFSET_A)
    {

        chassis_move_control->vy_set = -550;
    }
    else if (chassis_move_control->chassis_RC->key.v & KEY_PRESSED_OFFSET_D)
    {

        chassis_move_control->vy_set = 550;
    }
    else
    {
        chassis_move_control->vy_set = 0;
    }

    if (chassis_move_control->chassis_RC->key.v & KEY_PRESSED_OFFSET_W)
    {

        chassis_move_control->vx_set = 600;
    }
    else if (chassis_move_control->chassis_RC->key.v & KEY_PRESSED_OFFSET_S)
    {

        chassis_move_control->vx_set = -600;
    }
    else
    {
        chassis_move_control->vx_set = 0;
    }

    if (vision_rx->vx != 0 || vision_rx->vy != 0)
    {
        chassis_move_control->vx_set = (vision_rx->vx) * 2;
        chassis_move_control->vy_set = (vision_rx->vy) * 2;
    }
    else
    {
        chassis_move_control->vx_set += vx_set_channel_RC;
        chassis_move_control->vy_set += vy_set_channel_RC;
    }
}
