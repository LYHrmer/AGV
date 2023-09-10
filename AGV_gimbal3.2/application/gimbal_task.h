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
#include "rm_usart.h"
#include "kalman.h"

//线性控制器前馈系数
#define YAW_FEED_FORWARD 0.9f
#define PITCH_FEED_FORWARD -0.95f

//角度误差项系数
#define K_YAW_ANGLE_ERROR 50000.0f
#define K_PITCH_ANGLE_ERROR 100000.0f

//速度项系数
#define K_YAW_ANGLE_SPEED 5000.0f
#define K_PITCH_ANGLE_SPEED 4300.0f

//最大最小输出
#define YAW_MAX_OUT 32000.0f
#define YAW_MIX_OUT -32000.0f
#define PITCH_MAX_OUT 30000.0f
#define PITCH_MIX_OUT -30000.0f

//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        2000.0f
#define PITCH_SPEED_PID_KI       1.0f
#define PITCH_SPEED_PID_KD        10.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

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
#define GIMBAL_TASK_INIT_TIME 100
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
#define RC_DEADBAND   10


#define YAW_RC_SEN    -0.00001f
#define PITCH_RC_SEN  -0.00001f //0.0050.000005f

#define YAW_MOUSE_SEN   0.0004f
#define PITCH_MOUSE_SEN 0.0003f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen   0.00006f   //80
#define Pitch_Mouse_Sen  0.00007f  //35
#define Z_Mouse_Sen       0.00010f  //10

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
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
#define GIMBAL_ACCEL_X_NUM  200.0f
#define GIMBAL_ACCEL_Y_NUM 133.3f
#define GIMBAL_ACCEL_Y_GYRO_NUM 170.3f
#define GIMBAL_ACCEL_Z_NUM 170.3f
#endif

/**
 * @brief 云台自瞄模式详解
 * 
 */

//云台自瞄模式
#define GIMBAL_AUTO_MODE 1//若该自瞄模式不需要了，则将该位置0即可
 //使能编译云台自瞄模式
#if GIMBAL_AUTO_MODE
/**kalman filter  Q R 的数值
 * R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值,反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
 */
// yaw 轴的
#define GIMBAL_YAW_KALMAN_FILTER_Q 400
#define GIMBAL_YAW_KALMAN_FILTER_R 400
// pitch 轴
#define GIMBAL_PITCH_KALMAN_FILTER_Q 200
#define GIMBAL_PITCH_KALMAN_FILTER_R 400

#endif // !GIMBAL_AUTO_MODE

//云台pitch轴最大值相对角度
#define GIMBAL_PITCH_MAX_ENCODE 5112
//云台pitch轴最小相对角
#define GIMBAL_PITCH_MIN_ENCODE 3842

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

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
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
#if GIMBAL_AUTO_MODE
    // 电机卡尔曼滤波
    kalman gimbal_motor_kalman_filter;
#endif
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
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
	int frist_ecd;
	int ZERO_ECD_flag;
	int LAST_ZERO_ECD;
	int power_gimbal;

} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
#if GIMBAL_AUTO_MODE
    //视觉上位机数据
    const vision_rxfifo_t* gimbal_vision_control;
#endif
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
	

	
    // 滤波数据——>遥控器
    first_order_filter_type_t gimbal_cmd_slow_set_vx_RC;
    first_order_filter_type_t gimbal_cmd_slow_set_vy_RC;
    // 滤波数据——>键盘
    first_order_filter_type_t gimbal_cmd_slow_set_vx;
    first_order_filter_type_t gimbal_cmd_slow_set_vy;
    first_order_filter_type_t gimbal_cmd_slow_set_vz;
    first_order_filter_type_t gimbal_cmd_slow_set_vy_gyro;
    first_order_filter_type_t gimbal_cmd_slow_set_vx_auto;
    first_order_filter_type_t gimbal_cmd_slow_set_vy_auto;
	 //双板通信部分
		fp32 Gimbal_send_RE_angle;
		fp32 chassis_vx;
		fp32 chassis_vy;
		int16_t chassis_mode_e_Cansend;
		int16_t chassis_wz;
		int16_t key_C;
		fp32 cap_capvot;
		fp32 cap;
		
		int chassis_move_transit_mode_flag;
		fp32 angle_max;
		fp32 angle_min;
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

extern void gimbal_task(void const *pvParameters);

/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      pitch_offset:pitch 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:pitch 最大相对角度
  * @param[in]      min_yaw:pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
#endif
