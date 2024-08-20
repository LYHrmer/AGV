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
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "vision_task.h"

//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP         2000.0f
#define PITCH_SPEED_PID_KI         1.0f
#define PITCH_SPEED_PID_KD         10.0f
#define PITCH_SPEED_PID_MAX_OUT    30000.0f
#define PITCH_SPEED_PID_MAX_IOUT   10000.0f

//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        3600.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 80.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 1.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 5.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 100//401
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0
//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   20


#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  0.000004f //0.0050.000005f

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00006f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

#define PITCH_TURN  1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
////云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
//#define GIMBAL_INIT_ANGLE_ERROR     0.01f
//#define GIMBAL_INIT_STOP_TIME       100
//#define GIMBAL_INIT_TIME            5000
//#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.001f//0.003f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

//低通滤波系数
#define GIMBAL_YAW_AUTO_SCAN_NUM 133.3f
#define GIMBAL_PITCH_AUTO_SCAN_NUM 133.3f

//云台pitch轴最大值相对角度  0x004D
#define GIMBAL_PITCH_MAX_ENCODE 2627//0x004D
//云台pitch轴最小相对角
#define GIMBAL_PITCH_MIN_ENCODE 1635//0x1796
//云台pitch轴中值
#define GIMBAL_PITCH_OFFSET_ENCODE 1973
//云台yaw轴中值
#define GIMBAL_YAW_OFFSET_ENCODE 2379
//yaw轴后侧中值
#define GIMBAL_YAW_LAST_OFFSET_ENCODE (((GIMBAL_YAW_OFFSET_ENCODE + HALF_ECD_RANGE) > ECD_RANGE) ? (GIMBAL_YAW_OFFSET_ENCODE + HALF_ECD_RANGE - ECD_RANGE) : (GIMBAL_YAW_OFFSET_ENCODE + HALF_ECD_RANGE))
//云台yaw轴陀螺仪误差

#define INS_YAW_ERROR 0


//云台初始化计数最大值
#define INIT_STOP_COUNT  200
//云台摇摆yaw轴摇摆运动限幅,该数据为yaw轴中值到两边极限的角度范围,该角为绝对角
#define GIMBAL_YAW_SWING_RANGE (PI / 2)
//云台pitch轴摇摆运动向下最大编码值
#define GIMBAL_PITCH_SWING_DOWN_ECD GIMBAL_PITCH_MAX_ENCODE
//云台pitch轴摇摆运动向上最大编码值
#define GIMBAL_PITCH_SWING_UO_ECD GIMBAL_PITCH_MIN_ENCODE
//云台yaw轴运动步长(单位为rad)
#define GIMBAL_YAW_SWING_STE 0.01f;
//云台pitch轴运动步长(单位为rad)
#define GIMBAL_PITCH_SWING_STEP 0.01f
//云台yaw轴pitch轴设置最大时间
#define GIMBAL_SWING_STOP_COUNT 1000

//yaw轴扫描范围，以中心为基础 半个范围
#define YAW_SCAN_RANGE  PI
//pitch轴扫描范围，以中心值为基础
#define PITCH_SCAN_RANGE 0.20f

//yaw轴扫描步长 rad/S
#define YAW_SCAN_SPEED 1.2f
//pitch轴扫描步长 rad/s
#define PITCH_SCAN_SPEED 1.2f

//yaw轴扫描周期
#define YAW_SCAN_PERIOD (2 * YAW_SCAN_RANGE / YAW_SCAN_SPEED)
//pitch轴扫描周期
#define PITCH_SCAN_PERIOD (2 * PITCH_SCAN_RANGE / PITCH_SCAN_SPEED)


//线性控制器前馈系数
#define YAW_FEED_FORWARD 0.9f
#define PITCH_FEED_FORWARD 0.95f

//角度误差项系数
#define K_YAW_ANGLE_ERROR 80000.0f//60000.0f
#define K_PITCH_ANGLE_ERROR 450000.0f//850000.0f//550000.0f//450000.0f


//速度项系数
#define K_YAW_ANGLE_SPEED 6000.0f//5500.0f
#define K_PITCH_ANGLE_SPEED 5000.0f//7000.0f//8000.0f//3500.0f

//最大最小输出
#define YAW_MAX_OUT 32000.0f
#define YAW_MIX_OUT -32000.0f
#define PITCH_MAX_OUT 30000.0f
#define PITCH_MIX_OUT -30000.0f

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

//哨兵扫描结构体，用于在上位机未识别到目标时自动扫描做准备
typedef struct 
{
    // 扫描低通滤波结构体
    first_order_filter_type_t pitch_auto_scan_first_order_filter;
    first_order_filter_type_t yaw_auto_scan_first_order_filter;

    //yaw轴中心值
    fp32 yaw_center_value;
    //pitch轴中心值
    fp32 pitch_center_value;


    //yaw轴运动幅度
    fp32 yaw_range;
    //pitch轴运动幅度
    fp32 pitch_range;

    //当前运行时间 单位s
    fp32 scan_run_time;
    //初始计时时间 单位s
    fp32 scan_begin_time;

    //yaw轴扫描周期 单位s
    fp32 scan_yaw_period;
    //pitch轴扫描周期  单位s
    fp32 scan_pitch_period;

} scan_t;


//云台电机二阶线性控制器
typedef struct 
{
    //设定值
    fp32 set_angle;
    //当前角度   一阶状态
    fp32 cur_angle;
    //当前角速度 二阶状态
    fp32 cur_angle_speed;
    //角度误差项 一阶状态误差
    fp32 angle_error;
    //前馈项，用于消除系统固有扰动
    fp32 feed_forward;
    //输出值
    fp32 output;
    //最大输出值
    fp32 max_out;
    //最小输出值
    fp32 min_out;

    //前馈项系数
    fp32 k_feed_forward;
    //误差项系数
    fp32 k_angle_error;
    //二阶角速度项系数
    fp32 k_angle_speed;

}gimbal_motor_second_order_linear_controller_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;

    //二阶线性控制器
    gimbal_motor_second_order_linear_controller_t gimbal_motor_second_order_linear_controller;
  
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad

    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
	int frist_ecd;
	int zero_ecd_flag;
	int last_zero_ecd;
} gimbal_motor_t;

typedef struct
{
    //远程遥控器指针
    const RC_ctrl_t *gimbal_rc_ctrl;

    //获取视觉上位机数据
    const gimbal_vision_control_t* gimbal_vision_point;
    //云台自动移动结构体
    const auto_move_t* auto_move_point;

    //自动扫描结构体
    scan_t gimbal_auto_scan;

    //场地yaw轴正方向
    fp32 yaw_positive_direction;
    
    const INS_t* gimbal_INS_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;

	fp32 right_click_time;
	
} gimbal_control_t;


/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

//获取场地正方向
fp32 get_yaw_positive_direction(void);

extern void gimbal_task(void const *pvParameters);

#endif
