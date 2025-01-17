/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot_task.h"
#include "pid.h"
#include "PID_control.h"
#include "stm32.h"
#include "stm32_private.h"
#include "can_comm_task.h"
#include "chassis_task.h"
#include "bsp_usart.h"
#include "Dji_motor.h"
#include "DM_motor.h"


//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

static void gimbal_init(gimbal_control_t *init);

static void gimbal_set_mode(gimbal_control_t *set_mode);

static void gimbal_feedback_update(gimbal_control_t *feedback_update);

static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

static void gimbal_set_control(gimbal_control_t *set_control);

static void gimbal_control_loop(gimbal_control_t *control_loop);

static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t* controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed,fp32 kd,fp32 max_out, fp32 min_out);

static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t* controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current);

static void Pitch_Keep(gimbal_control_t *keep_alive);
//云台任务结构体
gimbal_control_t gimbal_control;

vision_rxfifo_t *vision_rx;

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    // wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    if (can_comm_task_init_finish())
    {
        // gimbal init
        // 云台初始化
        gimbal_init(&gimbal_control);

        while (1)
        {
            gimbal_set_mode(&gimbal_control);                    // 设置云台控制模式
            gimbal_mode_change_control_transit(&gimbal_control); // 控制模式切换 控制数据过渡
            gimbal_feedback_update(&gimbal_control);             // 云台数据反馈
            gimbal_set_control(&gimbal_control);                 // 设置云台控制量
            gimbal_control_loop(&gimbal_control);                // 云台控制计算
					
						if (toe_is_error(DBUS_TOE)||gimbal_control.gimbal_rc_ctrl->rc.s[1] == 2)
						{
							// 判断遥控器是否掉线
							can_cmd_gimbal_and_shoot(0,0);
							DM4310_Disable();
						}
						else
						{
							can_cmd_gimbal_and_shoot(gimbal_control.YAW_.yaw_motor.give_cmd_current,Ammo_booster.trigger_motor.trigger.give_cmd_current);
							if(DM4310_Motor.dm_motor.state == 1)
								DM4310_Tx_Date(&gimbal_control.PITCH_.pitch_motor);
							else
								DM4310_Enbale();
						}
            vTaskDelay(GIMBAL_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
            gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
    }
}
/****************************************
*函 数 名: gimbal_init
*功能说明: 云台初始化
*形    参: 无
*返 回 值: 无
*****************************************/
static void gimbal_init(gimbal_control_t *init)
{
    const static fp32 gimbal_yaw_auto_scan_order_filter[1] = {GIMBAL_YAW_AUTO_SCAN_NUM};
    const static fp32 gimbal_pitch_auto_scan_order_filter[1] = {GIMBAL_PITCH_AUTO_SCAN_NUM};

    // 电机初始化
    GM6020_Init(&init->YAW_.yaw_motor,YAW_ID,GIMBAL_YAW_OFFSET_ENCODE);
		vTaskDelay(1000);
		DM4310_Init(&init->PITCH_.pitch_motor,0,MIT,0);
		
    init->gimbal_INS_point = get_INS_point();
    // 遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    // 获取上位机视觉数据指针
    init->gimbal_vision_point = get_vision_gimbal_point();
    // 获取自动移动结构体
    init->auto_move_point = get_auto_move_point();
    // 初始化电机模式
    init->YAW_.gimbal_motor_mode = init->YAW_.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->PITCH_.gimbal_motor_mode = init->PITCH_.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //初始化云台电机二阶控制器
   // gimbal_motor_second_order_linear_controller_init(&init->YAW_.gimbal_motor_second_order_linear_controller, YAW_FEED_FORWARD, K_YAW_ANGLE_ERROR, K_YAW_ANGLE_SPEED,YAW_D,YAW_MAX_OUT, YAW_MIX_OUT);
    //gimbal_motor_second_order_linear_controller_init(&init->gimbal_pitch_motor.gimbal_motor_second_order_linear_controller, PITCH_FEED_FORWARD, K_PITCH_ANGLE_ERROR, K_PITCH_ANGLE_SPEED, PITCH_MAX_OUT, PITCH_MIX_OUT);

    // 云台数据更新     
    gimbal_feedback_update(init);
		
		//stm32pid初始化
		 stm32_pid_init_pitch(); 
     stm32_pid_init_yaw();
    // yaw轴电机初始化
    init->YAW_.absolute_angle_set = init->YAW_.absolute_angle;
    init->YAW_.relative_angle_set = init->YAW_.relative_angle;
    init->YAW_.motor_gyro_set = init->YAW_.motor_gyro;
    // pitch轴电机初始化
    init->PITCH_.absolute_angle_set = init->PITCH_.absolute_angle;
    init->PITCH_.relative_angle_set = init->PITCH_.relative_angle;
    init->PITCH_.motor_gyro_set = init->PITCH_.motor_gyro;
	 
     //初始化云台自动扫描低通滤波
    first_order_filter_init(&init->gimbal_auto_scan.pitch_auto_scan_first_order_filter, GIMBAL_CONTROL_TIME, gimbal_pitch_auto_scan_order_filter);
    first_order_filter_init(&init->gimbal_auto_scan.yaw_auto_scan_first_order_filter, GIMBAL_CONTROL_TIME, gimbal_yaw_auto_scan_order_filter);

    // 初始化云台自动扫描结构体的扫描范围
    init->gimbal_auto_scan.pitch_range = PITCH_SCAN_RANGE;
    init->gimbal_auto_scan.yaw_range = YAW_SCAN_RANGE;

//    // 初始化云台自动扫描周期
//    init->gimbal_auto_scan.scan_pitch_period = PITCH_SCAN_PERIOD;
//    init->gimbal_auto_scan.scan_yaw_period = YAW_SCAN_PERIOD;

    /* //获取云台自动扫描初始化时间
    init->gimbal_auto_scan.scan_begin_time = TIME_MS_TO_S(HAL_GetTick());
    //pitch轴扫描中心值
    init->gimbal_auto_scan.pitch_center_value = init->gimbal_auto_scan.pitch_range; */

    // 设置pitch轴相对角最大值
    init->PITCH_.max_relative_angle = -motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, init->PITCH_.pitch_motor.Encoder_Offset);
    init->PITCH_.min_relative_angle = -motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, init->PITCH_.pitch_motor.Encoder_Offset);
    init->PITCH_.max_absolute_angle = PITCH_ABS_MAX;
    init->PITCH_.min_absolute_angle = PITCH_ABS_MIN;
		
	
		//pitch轴绝对角度控制PID初始化
		PID_UP_Init(&init->PITCH_.Absloute_Angle_PID,PITCH_ABSLOUTE_ANGLE_PID_KP, PITCH_ABSLOUTE_ANGLE_PID_KI, PITCH_ABSLOUTE_ANGLE_PID_KD, 
    PITCH_ABSLOUTE_ANGLE_PID_KF, PITCH_ABSLOUTE_ANGLE_PID_I_MAXOUT, PITCH_ABSLOUTE_ANGLE_PID_MAX_OUT, PID_D_T, PITCH_ABSLOUTE_ANGLE_PID_DEAD_ZONE,
    PITCH_ABSLOUTE_ANGLE_I_Variable_Speed_A, PITCH_ABSLOUTE_ANGLE_I_Variable_Speed_B, PITCH_ABSLOUTE_ANGLE_I_Separate_Threshold, PID_D_First_ENABLE);
		
		//视觉
		vision_rx=get_vision_fifo();

}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    // 云台数据更新
    feedback_update->PITCH_.absolute_angle = feedback_update->gimbal_INS_point->Pitch;
    feedback_update->PITCH_.relative_angle = -motor_ecd_to_angle_change(feedback_update->PITCH_.pitch_motor.ecd,
                                                                                   feedback_update->PITCH_.pitch_motor.Encoder_Offset);
    feedback_update->PITCH_.motor_gyro = feedback_update->gimbal_INS_point->Gyro[0];

    feedback_update->YAW_.absolute_angle = feedback_update->gimbal_INS_point->Yaw;
    feedback_update->YAW_.relative_angle = motor_ecd_to_angle_change(feedback_update->YAW_.yaw_motor.ecd,
																																									 feedback_update->YAW_.yaw_motor.Encoder_Offset);
    feedback_update->YAW_.motor_gyro = arm_cos_f32(feedback_update->PITCH_.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[Z]) - arm_sin_f32(feedback_update->PITCH_.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[X]);
		
		GM6020_Rx_Date(&feedback_update->YAW_.yaw_motor);	
}

/****************************************
*函 数 名: motor_ecd_to_angle_change
*功能说明: 云台模式切换数据保存
*形    参: ecd：当前编码值
					 offset_ecd：零点编码值
*返 回 值: 相对角度编码值
*****************************************/
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    fp32 relative_ecd = ecd - offset_ecd;

    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/****************************************
*函 数 名: gimbal_mode_change_control_transit
*功能说明: 云台模式切换数据保存
*形    参: 无
*返 回 值: 无
*****************************************/
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    // yaw电机状态机切换保存数据
    if (gimbal_mode_change->YAW_.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->YAW_.raw_cmd_current = gimbal_mode_change->YAW_.yaw_motor.give_cmd_current;
    }
    else if (gimbal_mode_change->YAW_.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->YAW_.absolute_angle_set = gimbal_mode_change->YAW_.absolute_angle;
    }
    else if (gimbal_mode_change->YAW_.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->YAW_.relative_angle_set = gimbal_mode_change->YAW_.relative_angle;
    }
    gimbal_mode_change->YAW_.last_gimbal_motor_mode = gimbal_mode_change->YAW_.gimbal_motor_mode;

    // pitch电机状态机切换保存数据
    if (gimbal_mode_change->PITCH_.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->PITCH_.raw_cmd_current = gimbal_mode_change->PITCH_.pitch_motor.give_torque_current;
    }
    else if (gimbal_mode_change->PITCH_.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->PITCH_.absolute_angle_set = gimbal_mode_change->PITCH_.absolute_angle;
    }
    else if (gimbal_mode_change->PITCH_.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->PITCH_.relative_angle_set = gimbal_mode_change->PITCH_.relative_angle;
    }

    gimbal_mode_change->PITCH_.last_gimbal_motor_mode = gimbal_mode_change->PITCH_.gimbal_motor_mode;
}
/****************************************
*函 数 名: gimbal_set_control
*功能说明: 云台控制设定值
*形    参: 无
*返 回 值: 无
*****************************************/
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    // yaw电机模式控制
    if (set_control->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->YAW_.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->YAW_, add_yaw_angle);
    }
    else if (set_control->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->YAW_, add_yaw_angle);
    }

    // pitch电机模式控制
    if (set_control->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->PITCH_.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->PITCH_, add_pitch_angle);
    }
    else if (set_control->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->PITCH_, add_pitch_angle);
    }
}

static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set_yaw = 0;

    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_motor == &gimbal_control.YAW_)
    {
//        angle_set_yaw = gimbal_motor->absolute_angle_set;
//        gimbal_motor->absolute_angle_set = rad_format(angle_set_yaw + add);
			   gimbal_motor->absolute_angle_set += add;
    }
    else if(gimbal_motor == &gimbal_control.PITCH_)
    {
//        // 当前误差角度
//        static fp32 error_angle = 0;
//        static fp32 angle_set = 0;
//        error_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
//        // 云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
//        if (gimbal_motor->relative_angle + error_angle + add < gimbal_motor->max_relative_angle)
//        {
//            // 如果是往最大机械角度控制方向
//            if (add < 0.0f)
//            {
//                // 计算出一个最大的添加角度，
//                add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - error_angle;
//            }
//        }
//        else if (gimbal_motor->relative_angle + error_angle + add > gimbal_motor->min_relative_angle)
//        {
//            if (add > 0.0f)
//            {
//                add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - error_angle;
//            }
//        }
//        angle_set = gimbal_motor->absolute_angle_set;
//        gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
				gimbal_motor->absolute_angle_set += add;
				gimbal_motor->absolute_angle_set = Math_Constrain(&gimbal_motor->absolute_angle_set,PITCH_ABS_MIN,PITCH_ABS_MAX);
    }
}
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
		if(gimbal_motor == &gimbal_control.YAW_)
		{
			// 云台yaw轴相对角度限制
			if (gimbal_motor->relative_angle_set < 0)
			{
					gimbal_motor->relative_angle_set = 2 * PI + gimbal_motor->relative_angle_set;
			}
			else if (gimbal_motor->relative_angle_set > 2 * PI)
			{
					gimbal_motor->relative_angle_set = gimbal_motor->relative_angle_set - 2 * PI;
			}
		}
		else if(gimbal_motor == &gimbal_control.PITCH_)
		{
					// 云台pitch轴相对角度限制，防止pitch轴转动过度
			if (gimbal_motor->relative_angle_set >= motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE,gimbal_motor->pitch_motor.Encoder_Offset))
			{
					gimbal_motor->relative_angle_set = motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE,gimbal_motor->pitch_motor.Encoder_Offset);
			}
			else if (gimbal_motor->relative_angle_set <= motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE,gimbal_motor->pitch_motor.Encoder_Offset))
			{
					gimbal_motor->relative_angle_set = motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE,gimbal_motor->pitch_motor.Encoder_Offset);
			} 
			
		}
}

/****************************************
*函 数 名: gimbal_control_loop
*功能说明: 云台控制循环
*形    参: 无
*返 回 值: 无
*****************************************/
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    if (control_loop->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->YAW_);
    }
    else if (control_loop->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->YAW_);
    }
    else if (control_loop->YAW_.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->YAW_);
    }
		
		if(control_loop->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_INIT)
		{
				DM4310_Tx_Date(&control_loop->PITCH_.pitch_motor);
		}
    else if (control_loop->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->PITCH_);
    }
    else if (control_loop->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->PITCH_);
    }
    else if (control_loop->PITCH_.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->PITCH_);
    }
}

/****************************************
*函 数 名: gimbal_motor_absolute_angle_control
*功能说明: 陀螺仪欧拉角控制
*形    参: 无
*返 回 值: 无
*****************************************/
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		if(gimbal_motor == &gimbal_control.YAW_)
		{
			//计算云台电机控制电流 
		
//			gimbal_motor->YAW_Absloute_Angle_PID.Target = gimbal_motor->absolute_angle_set;
//			gimbal_motor->YAW_Absloute_Angle_PID.Now = gimbal_motor->absolute_angle;
//			PID_TIM_Adjust_PeriodElapsedCallback(&gimbal_motor->YAW_Absloute_Angle_PID);
//			gimbal_motor->yaw_motor.give_cmd_current = gimbal_motor->YAW_Absloute_Angle_PID.Out;
//			gimbal_motor->yaw_motor.give_cmd_current = (int16_t)gimbal_motor_second_order_linear_controller_calc(&gimbal_motor->gimbal_motor_second_order_linear_controller, gimbal_motor->absolute_angle_set,
//			gimbal_motor->absolute_angle,gimbal_motor->yaw_motor.get_omega, gimbal_motor->yaw_motor.give_cmd_current);
			stm32_step_yaw(gimbal_motor->absolute_angle_set,gimbal_motor->absolute_angle,gimbal_motor->motor_gyro);
			gimbal_motor->yaw_motor.give_cmd_current = (int16_t)stm32_Y_yaw.Out1;
			
		}
		else if(gimbal_motor == &gimbal_control.PITCH_)
		{
			gimbal_motor->Absloute_Angle_PID.Target = gimbal_motor->absolute_angle_set;
			gimbal_motor->Absloute_Angle_PID.Now = gimbal_motor->absolute_angle;
			PID_TIM_Adjust_PeriodElapsedCallback(&gimbal_motor->Absloute_Angle_PID);
			if(gimbal_motor->pitch_motor.get_angle > 1 && gimbal_motor->pitch_motor.give_pos > 1 && gimbal_motor->absolute_angle_set > PITCH_ABS_MAX) //限位1
				gimbal_motor->pitch_motor.give_pos = 1;
			else if(gimbal_motor->pitch_motor.get_angle<-1 && gimbal_motor->pitch_motor.give_pos < -1 && gimbal_motor->absolute_angle_set<PITCH_ABS_MIN)//限位-1
				gimbal_motor->pitch_motor.give_pos = -1;
			else
				gimbal_motor->pitch_motor.give_pos += gimbal_motor->Absloute_Angle_PID.Out; //负号，如果电机外装改为﹢
			gimbal_motor->pitch_motor.give_pos = Math_Constrain(&gimbal_motor->pitch_motor.give_pos,DM4310_POS_MIN,DM4310_POS_MAX);

		}
}

/****************************************
*函 数 名: gimbal_motor_relative_angle_control
*功能说明: 编码值相对角控制
*形    参: 无
*返 回 值: 无
*****************************************/
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		if(gimbal_motor == &gimbal_control.YAW_)
		{
			//计算云台电机控制电流
			gimbal_motor->yaw_motor.give_cmd_current = 
			(int16_t)gimbal_motor_second_order_linear_controller_calc(&gimbal_motor->gimbal_motor_second_order_linear_controller, gimbal_motor->relative_angle_set, 
															gimbal_motor->relative_angle, gimbal_motor->yaw_motor.get_speed, gimbal_motor->yaw_motor.give_cmd_current);
		}
		else if(gimbal_motor == &gimbal_control.PITCH_)
		{
			
		}
//	stm32_step_pitch(gimbal_motor->relative_angle_set,gimbal_motor->relative_angle,0);
//	gimbal_motor->given_current = stm32_Y_pitch.Out1;
}

/****************************************
*函 数 名: gimbal_motor_raw_angle_control
*功能说明: 电流值直接发送到CAN
*形    参: 无
*返 回 值: 无
*****************************************/
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->yaw_motor.give_cmd_current = (int16_t)gimbal_motor->raw_cmd_current;
		gimbal_motor->pitch_motor.give_torque_current = (int16_t)gimbal_motor->raw_cmd_current;
}


/**
 * @brief 云台二阶线性控制器初始化
 *
 * @param controller 云台二阶线性控制器结构体
 * @param k_feed_forward 前馈系数
 * @param k_angle_error 角度误差系数
 * @param k_angle_speed 角速度系数
 * @param max_out 最大输出值
 * @param min_out 最小输出值
 */
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed,fp32 kd, fp32 max_out, fp32 min_out)
{
    // 前馈项系数
    controller->k_feed_forward = k_feed_forward;
    // 反馈矩阵系数
    controller->k_angle_error = k_angle_error;
    controller->k_angle_speed = k_angle_speed;
	   controller->kd=kd;
    // 设置最大输出值
    controller->max_out = max_out;
    // 设置最小输出值
    controller->min_out = min_out;
}

/**
 * @brief 云台二阶线性控制器计算
 *
 * @param controller 云台二阶线性控制器结构体
 * @param set_angle 角度设置值
 * @param cur_angle 当前角度
 * @param cur_angle_speed 当前角速度
 * @param cur_current 当前电流
 * @return 返回系统输入
 */
static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t *controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current)
{
  // 赋值
  controller->cur_angle = cur_angle;
  controller->set_angle = set_angle;
  controller->cur_angle_speed = cur_angle_speed;
  // 将当前电流值乘以一个小于1的系数当作阻挡系统固有扰动的前馈项
  controller->feed_forward = controller->k_feed_forward * cur_current;
  
	controller->angle_error[2] = controller->angle_error[1];
	controller->angle_error[1] = controller->angle_error[0];
	// 计算误差 = 设定角度 - 当前角度
  controller->angle_error[0] = controller->set_angle - controller->cur_angle;
	//计算d项
	controller->Dbuf = controller->angle_error[0] - controller->angle_error[1];
  // 将误差值限制 -PI ~ PI 之间
  controller->angle_error[0] = rad_format(controller->angle_error[0]);
  // 计算输出值 = 前馈值 + 角度误差值 * 系数 + 角速度 * 系数 + d项 * 系数
  controller->output = controller->feed_forward + controller->angle_error[0] * controller->k_angle_error
      	+ (-controller->cur_angle_speed * controller->k_angle_speed)	+ controller->kd * controller->Dbuf;

  // 限制输出值，防止出现电机崩溃的情况
  if (controller->output >= controller->max_out)
  {
    controller->output = controller->max_out;
  }
  else if (controller->output <= controller->min_out)
  {
    controller->output = controller->min_out;
  }

  return controller->output;
}
//获取场地正方向
fp32 get_yaw_positive_direction(void)
{
    return gimbal_control.yaw_positive_direction;
}
