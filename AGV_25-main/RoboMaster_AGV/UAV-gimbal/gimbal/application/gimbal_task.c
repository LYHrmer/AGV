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
#include "stm32.h"
#include "stm32_private.h"
#include "can_comm_task.h"
#include "chassis_task.h"
#include "bsp_usart.h"
// motor enconde value format, range[0-8191]
// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }
		
uint32_t Counter = 0;
		
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init);

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode);

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_INIT
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_init_angle_control(gimbal_motor_t *gimbal_motor);

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
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t* controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out);

/**
 * @brief 云台二阶线性控制器计算 
 * 
 * @param controller 云台二阶线性控制器结构体
 * @param set_angle 角度设置值
 * @param cur_angle 当前角度
 * @param cur_angle_speed 当前角速度 
 * @param cur_current 当前电流
 * @return 返回系统输入 即电机电流值 
 */
static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t* controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current);

//保持达妙电机存活
void DM__keep_alive(gimbal_control_t *keep_alive);

extern chassis_move_t chassis_move;
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
    // gimbal init
    // 云台初始化
    gimbal_init(&gimbal_control);
    //判断电机是否都上线
    gimbal_feedback_update(&gimbal_control); // 云台数据反馈
    while (1)
    {
      gimbal_set_mode(&gimbal_control);                    // 设置云台控制模式
      gimbal_mode_change_control_transit(&gimbal_control); // 控制模式切换 控制数据过渡
      gimbal_feedback_update(&gimbal_control);             // 云台数据反馈
      gimbal_set_control(&gimbal_control);                 // 设置云台控制量
      gimbal_control_loop(&gimbal_control);                // 云台控制计算
			DM__keep_alive(&gimbal_control); 
    if (toe_is_error(DBUS_TOE))
    // 判断遥控器是否掉线
    CAN_cmd_gimbal(0, 0);   
			//Motor_DM_Normal_CAN_Send_Disable(&gimbal_control.DM_j4310.motor_j4310); //失能}
    else
    {
//			Motor_DM_Normal_CAN_Send_Enable(&gimbal_control.DM_j4310.motor_j4310);
		  CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.given_current,0);
		  Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&gimbal_control.DM_j4310.motor_j4310);
    }
      vTaskDelay(1);
//#if INCLUDE_uxTaskGetStackHighWaterMark
//            gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
//    }
}

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
	  return &gimbal_control.DM_j4310;
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
		const static fp32 pitch_increment_error_pid[3] = {0.8, 0.0, 0.01}; //内置PID自稳云台
		
    const static fp32 gimbal_yaw_auto_scan_order_filter[1] = {GIMBAL_YAW_AUTO_SCAN_NUM};
    const static fp32 gimbal_pitch_auto_scan_order_filter[1] = {GIMBAL_PITCH_AUTO_SCAN_NUM};
		
    // 给底盘跟随云台模式用的
    gimbal_control.gimbal_yaw_motor.zero_ecd_flag = GIMBAL_YAW_LAST_OFFSET_ENCODE;
    gimbal_control.gimbal_yaw_motor.last_zero_ecd = GIMBAL_YAW_LAST_OFFSET_ENCODE;

    // 云台计算相对角用 
    gimbal_control.gimbal_yaw_motor.frist_ecd = GIMBAL_YAW_OFFSET_ENCODE;
		
    // 6020电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    // 陀螺仪数据指针获取
    init->gimbal_INS_point = get_INS_point();
    // 遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
//    // 获取上位机视觉数据指针
//    init->gimbal_vision_point = get_vision_gimbal_point();
//    // 获取自动移动结构体
//    init->auto_move_point = get_auto_move_point();
    // 初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->DM_j4310.gimbal_motor_mode = init->DM_j4310.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //初始化云台电机二阶控制器
    gimbal_motor_second_order_linear_controller_init(&init->gimbal_yaw_motor.gimbal_motor_second_order_linear_controller, YAW_FEED_FORWARD, K_YAW_ANGLE_ERROR, K_YAW_ANGLE_SPEED, YAW_MAX_OUT, YAW_MIX_OUT);
    gimbal_motor_second_order_linear_controller_init(&init->DM_j4310.gimbal_motor_second_order_linear_controller, PITCH_FEED_FORWARD, K_PITCH_ANGLE_ERROR, K_PITCH_ANGLE_SPEED, PITCH_MAX_OUT, PITCH_MIX_OUT);

    // 云台数据更新     
    gimbal_feedback_update(init);
		
		//stm32pid初始化
		 stm32_pid_init_pitch(); 
    
    // yaw轴数据初始化
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
		
    // pitch轴数据初始化
    init->DM_j4310.absolute_angle_set = init->DM_j4310.absolute_angle;
    init->DM_j4310.relative_angle_set = init->DM_j4310.relative_angle;
    init->DM_j4310.motor_gyro_set = init->DM_j4310.motor_gyro;
		
    //达妙电机初始化
		Motor_DM_Normal_CAN_Send_Enable(&init->DM_j4310.motor_j4310);   //达妙电机使能
		init->DM_j4310.Motor_DM_Control_Method = Motor_DM_Control_Method_NORMAL_MIT; //达妙电机模式设置
		Motor_DM_Normal_Init(&init->DM_j4310.motor_j4310,&hcan1,
	                     0x11,0x01,Motor_DM_Control_Method_NORMAL_MIT,
	                     Angle_Max,Omega_Max,Torque_Max,Current_Max); //达妙电机初始化
		init->DM_j4310.motor_j4310.Control_Omega = 0.0f;
		init->DM_j4310.motor_j4310.Control_Current = 0.0f;
		init->DM_j4310.motor_j4310.K_P = 40.0f;
		init->DM_j4310.motor_j4310.K_D = 1.0f;
		
     //初始化云台自动扫描低通滤波
    first_order_filter_init(&init->gimbal_auto_scan.pitch_auto_scan_first_order_filter, GIMBAL_CONTROL_TIME, gimbal_pitch_auto_scan_order_filter);
    first_order_filter_init(&init->gimbal_auto_scan.yaw_auto_scan_first_order_filter, GIMBAL_CONTROL_TIME, gimbal_yaw_auto_scan_order_filter);

    //yaw轴云台初始化相对角度
    init->gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ENCODE;
    // pitch轴云台初始化相对角度
    init->DM_j4310.offset_ecd = GIMBAL_PITCH_OFFSET_ENCODE; 

    // 初始化云台自动扫描结构体的扫描范围
    init->gimbal_auto_scan.pitch_range = PITCH_SCAN_RANGE;
    init->gimbal_auto_scan.yaw_range = YAW_SCAN_RANGE;

    // 初始化云台自动扫描周期
    init->gimbal_auto_scan.scan_pitch_period = PITCH_SCAN_PERIOD;
    init->gimbal_auto_scan.scan_yaw_period = YAW_SCAN_PERIOD;

    // 设置pitch轴相对角最大值
    init->DM_j4310.max_relative_angle = motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, init->DM_j4310.offset_ecd);
    init->DM_j4310.min_relative_angle = motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, init->DM_j4310.offset_ecd);
    init->DM_j4310.max_absolute_angle = PI /6;
    init->DM_j4310.min_absolute_angle = -PI /6;
		
//		// 设置yaw轴相对角最大值，飞机yaw轴专用，舵步忽略
//    init->gimbal_yaw_motor.max_relative_angle = motor_ecd_to_angle_change(GIMBAL_YAW_MAX_ENCODE, init->gimbal_yaw_motor.offset_ecd);
//    init->gimbal_yaw_motor.min_relative_angle = motor_ecd_to_angle_change(GIMBAL_YAW_MIN_ENCODE, init->gimbal_yaw_motor.offset_ecd);
		
		//pitch轴绝对角度控制PID初始化
    PID_UP_Init(&init->DM_j4310.Absloute_Angle_PID,PITCH_ABSLOUTE_ANGLE_PID_KP, PITCH_ABSLOUTE_ANGLE_PID_KI, PITCH_ABSLOUTE_ANGLE_PID_KD, 
    PITCH_ABSLOUTE_ANGLE_PID_KF, PITCH_ABSLOUTE_ANGLE_PID_I_MAXOUT, PITCH_ABSLOUTE_ANGLE_PID_MAX_OUT, PID_D_T, PITCH_ABSLOUTE_ANGLE_PID_DEAD_ZONE,
    PITCH_ABSLOUTE_ANGLE_I_Variable_Speed_A, PITCH_ABSLOUTE_ANGLE_I_Variable_Speed_B, PITCH_ABSLOUTE_ANGLE_I_Separate_Threshold, PID_D_First_ENABLE);
   
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
    feedback_update->DM_j4310.absolute_angle = feedback_update->gimbal_INS_point->Pitch;
    feedback_update->DM_j4310.motor_gyro = feedback_update->gimbal_INS_point->Gyro[0];

    feedback_update->gimbal_yaw_motor.absolute_angle = feedback_update->gimbal_INS_point->Yaw;
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd, feedback_update->gimbal_yaw_motor.frist_ecd);
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[Z]) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (feedback_update->gimbal_INS_point->Gyro[X]);
}

/**
 * @brief          calculate the relative angle between ecd and offset_ecd
 * @param[in]      ecd: motor now encode
 * @param[in]      offset_ecd: gimbal offset encode
 * @retval         relative angle, unit rad
 */
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
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

/**
 * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
 * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    // yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    // pitch电机状态机切换保存数据
		if (gimbal_mode_change->DM_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_INIT && gimbal_mode_change->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_INIT)
		{
				gimbal_mode_change->DM_j4310.motor_j4310.Control_Angle = 0.0f;
		}
    if (gimbal_mode_change->DM_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->DM_j4310.raw_cmd_current = gimbal_mode_change->DM_j4310.current_set = gimbal_mode_change->DM_j4310.given_current;
    }
    else if (gimbal_mode_change->DM_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->DM_j4310.absolute_angle_set = gimbal_mode_change->DM_j4310.absolute_angle;
    }
    else if (gimbal_mode_change->DM_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->DM_j4310.relative_angle_set = gimbal_mode_change->DM_j4310.relative_angle;
    }

    gimbal_mode_change->DM_j4310.last_gimbal_motor_mode = gimbal_mode_change->DM_j4310.gimbal_motor_mode;
}
/**
 * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
 * @param[out]     gimbal_set_control: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
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
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    // pitch电机模式控制
    if (set_control->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->DM_j4310.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->DM_j4310, add_pitch_angle);
    }
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set_yaw = 0;

    if (gimbal_motor == NULL)
    {
        return;
    }
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        angle_set_yaw = gimbal_motor->absolute_angle_set;
        gimbal_motor->absolute_angle_set = rad_format(angle_set_yaw + add);
			
//        // 当前误差角度   飞机yaw轴使用，舵步可以忽略
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
    }
    else if (gimbal_motor == &gimbal_control.DM_j4310)
    {
        // 当前误差角度
			static fp32 angle_set = 0;
			angle_set = gimbal_motor->absolute_angle_set;		
			gimbal_motor->absolute_angle_set = angle_set + add;
			if(gimbal_motor->absolute_angle_set > gimbal_motor->max_absolute_angle)
			{
				gimbal_motor->absolute_angle_set = gimbal_motor->max_absolute_angle;
			}
			if(gimbal_motor->absolute_angle_set < gimbal_motor->min_absolute_angle)
			{
				gimbal_motor->absolute_angle_set = gimbal_motor->min_absolute_angle;
			}
		
    }
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    gimbal_motor->relative_angle_set += add;

    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
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
    else if (gimbal_motor == &gimbal_control.DM_j4310)
    {
        // 云台pitch轴相对角度限制，防止pitch轴转动过度
        if (gimbal_motor->relative_angle_set >= motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, gimbal_motor->offset_ecd))
        {
            gimbal_motor->relative_angle_set = motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, gimbal_motor->offset_ecd);
        }
        else if (gimbal_motor->relative_angle_set <= motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, gimbal_motor->offset_ecd))
        {
            gimbal_motor->relative_angle_set = motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, gimbal_motor->offset_ecd);
        }
    }
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_INIT)
		{
				gimbal_motor_init_angle_control(&control_loop->DM_j4310);
		}
    else if (control_loop->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->DM_j4310);
    }
    else if (control_loop->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->DM_j4310);
    }
    else if (control_loop->DM_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->DM_j4310);
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
		//计算云台电机控制电流
    gimbal_motor->current_set = gimbal_motor_second_order_linear_controller_calc(&gimbal_motor->gimbal_motor_second_order_linear_controller, gimbal_motor->absolute_angle_set, gimbal_motor->absolute_angle, gimbal_motor->motor_gyro, gimbal_motor->gimbal_motor_measure->given_current);
    //赋值电流值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		}
		else
		{
		//计算绝对角度控制
	  gimbal_motor->Absloute_Angle_PID.Target = gimbal_motor->absolute_angle_set;
    gimbal_motor->Absloute_Angle_PID.Now = gimbal_motor->absolute_angle;
    PID_TIM_Adjust_PeriodElapsedCallback(&gimbal_motor->Absloute_Angle_PID);
    if(gimbal_motor->motor_j4310.Rx_Data.Now_Angle>1 && gimbal_motor->motor_j4310.Control_Angle>1 &&gimbal_motor->absolute_angle_set>1) //限位1
			gimbal_motor->motor_j4310.Control_Angle = 1;
		else if(gimbal_motor->motor_j4310.Rx_Data.Now_Angle<-1 && gimbal_motor->motor_j4310.Control_Angle<-1 &&gimbal_motor->absolute_angle_set<-1)//限位-1
			gimbal_motor->motor_j4310.Control_Angle = -1;
		else
			gimbal_motor->motor_j4310.Control_Angle -= gimbal_motor->Absloute_Angle_PID.Out; //负号，如果电机外装改为﹢
    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&gimbal_motor->motor_j4310);
		}
}
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //计算云台电机控制电流
    gimbal_motor->current_set = gimbal_motor_second_order_linear_controller_calc(&gimbal_motor->gimbal_motor_second_order_linear_controller, gimbal_motor->relative_angle_set, gimbal_motor->relative_angle, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->gimbal_motor_measure->given_current);
    //赋值电流值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_init_angle_control(gimbal_motor_t *gimbal_motor)
{
	Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&gimbal_motor->motor_j4310);
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
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out)
{
    // 前馈项系数
    controller->k_feed_forward = k_feed_forward;
    // 反馈矩阵系数
    controller->k_angle_error = k_angle_error;
    controller->k_angle_speed = k_angle_speed;
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
    // 计算误差 = 设定角度 - 当前角度
    controller->angle_error = controller->set_angle - controller->cur_angle;
    // 将误差值限制 -PI ~ PI 之间
    controller->angle_error = rad_format(controller->angle_error);
    // 计算输出值 = 前馈值 + 角度误差值 * 系数 + 角速度 * 系数
    controller->output = controller->feed_forward + controller->angle_error * controller->k_angle_error + (-controller->cur_angle_speed * controller->k_angle_speed);

    //限制输出值，防止出现电机崩溃的情况
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

void DM__keep_alive(gimbal_control_t *keep_alive)
{
	if(keep_alive->DM_j4310.gimbal_motor_mode == GIMBAL_ZERO_FORCE)
	{
		Motor_DM_Normal_CAN_Send_Disable(&keep_alive->DM_j4310.motor_j4310);
	}
	else
	{
		static uint32_t Counter_KeepAlive = 0;
		if (Counter_KeepAlive++ > 100)
		{
			Counter_KeepAlive = 0;
			
			Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&keep_alive->DM_j4310.motor_j4310);
		}
	}

}