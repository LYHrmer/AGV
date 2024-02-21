#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "Remote_Control.h"
#include "pid.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "stm32.h"
#include "struct_typedef.h"

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

#define CHASSIS_RC_DEADLINE 10

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.2f
#define MOTOR_DISTANCE_TO_CENTER   0.32 //0.32f

#define DEG2R(x) ((x)*PI /180.0f)

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002

//底盘低通滤波系数
#define CHASSIS_ACCEL_X_NUM 0.02f
#define CHASSIS_ACCEL_Y_NUM 0.01f

//X，Y角度与遥控器输入比例
#define X_RC_SEN   0.0006f
#define Y_RC_SEN -0.0005f //0.005

//Y,Y角度和鼠标输入的比例
#define X_Mouse_Sen 0.0002f
#define Y_Mouse_Sen 0.00025f

//X,Y控制通道
#define X_Channel 2
#define Y_Channel 3

//电机码盘值最大以及中值
#define Half_ecd_range 395  //395  7796
#define ecd_range 8191

//编码-弧度
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.00076708402f //      2*  PI  /8192
#endif

//底盘电机最大速度
#define MAX_WHEEL_SPEED 20.0f

//功率控制相关参数
#define toque_coefficient 1.99688994e-6f // (20/16384)*(0.3)*(187/3591)/9.55  此参数将电机电流转换为扭矩
#define k2 1.23e-07						 // 铜损系数
#define k1 1.453e-07					 // 磁损系数
#define constant_3508 4.081f  //控制器静态误差

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//舵电机的编码值零点
#define Forward_L_ecd 45//左前轮  1号
#define Forward_R_ecd 688    //右前轮  2号
#define Back_L_ecd 7487       //左后轮  3号
#define Back_R_ecd 5432       //右后轮  4号

//底盘电机速度环PID
#define CHASSIS_KP 4000.f  //2000
#define CHASSIS_KI 0.0f //0
#define CHASSIS_KD 20.f //20
#define CHASSIS_MAX_OUT 16000.f
#define CHASSIS_MAX_IOUT 2000.f


//舵电机PID
#define RUDDER_P_P 18.1f     //18.1
#define RUDDER_P_I 0.0f      //0
#define RUDDER_P_D 2.0f      //2.0
#define RUDDER_P_N 1.5f      //1.5
#define RUDDER_S_P 2.0f      //1.0
#define RUDDER_S_I 0.0f      //0.0
#define RUDDER_S_D 0.0f      //0.0
#define RUDDER_S_N 0.0f      //0.0


//底盘旋转跟随PID

#define CHASSIS_FOLLOW_GIMBAL_PID_KP  3.0f//3.0f               // 3.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI  0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD  10.0f//20.f//10.0f 
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   4//4
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KF 0.0f
#define CHASSIS_FOLLOW_GIMBAL_F_divider 0.0
#define CHASSIS_FOLLOW_GIMBAL_F_out_limit 0.0f

//底盘电机功率环PID
#define M3505_MOTOR_POWER_PID_KP 1.0f
#define M3505_MOTOR_POWER_PID_KI 0.5f
#define M3505_MOTOR_POWER_PID_KD 0.f
#define M3505_MOTOR_POWER_PID_MAX_OUT 30.0f  //60 
#define M3505_MOTOR_POWER_PID_MAX_IOUT 10.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//1/60*0.003335*3.1415926
#define GM6020_RPM_TO_VECTOR 0.001746201886833

//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.02f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.02f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘跟随云台
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //底盘自主
	CHASSIS_VECTOR_SPIN,                //小陀螺
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //底盘不跟随
  CHASSIS_VECTOR_RAW,									//底盘原始控制
  RUDDER_VECTOR_FOLLOW_GIMBAL_YAW     //舵跟随云台
} chassis_mode_e;

typedef struct
{
  fp32 relative_angle;
	int16_t relative_angle_receive;
} Gimbal_data;

typedef struct
{
	fp32 totalCurrentTemp;
	fp32 totalSpeed;
	fp32 current[4];
	fp32 power_current[4];
	fp32 speed[4];
	fp32 POWER_MAX;
	fp32 MAX_current[4];
  fp32 SPEED_MIN;	
	fp32 K;
	fp32 power_charge; //超电充电
	fp32 forecast_motor_power[4]; // 预测单个电机功率
	fp32 forecast_total_power; // 预测总功率
} Power_Control;

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
		
} chassis_PID_t;
typedef struct
{
    const motor_measure_t *gimbal_motor_measure;   //云台数据
	  Rudder_control rudder_control;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad
	
	  fp32 control;
    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;    //角速度设定
    fp32 motor_speed;     
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
		int16_t  ecd_add; 
    int16_t last_ecd_add;
	  int16_t ecd_temp_error;
		int16_t ecd_set;  
    int16_t ecd_set_final;	
		int16_t last_ecd_set;
		int16_t ecd_error;
		int16_t ecd_error_true;
		int16_t ecd_turn;
    int16_t ecd_change_MIN;
		fp32 wheel_speed;
		fp32 rudder_angle;
		fp32 last_rudder_angle;
		int16_t ecd_zero_set;
		fp32 Judge_Speed_Direction;
		fp32 Judge_Speed_cosk;
} Rudder_Motor_t;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
	PidTypeDef chassis_pid;
} Chassis_Motor_t;

typedef struct
{        
	  const RC_ctrl_t *chassis_rc_ctrl;
	  Chassis_Motor_t motor_chassis[4]; 
	  chassis_mode_e chassis_motor_mode;
    chassis_mode_e last_chassis_motor_mode;
	 
	  PidTypeDef chassis_angle_pid;   //底盘跟随角度pid
		PidTypeDef buffer_pid;     //功率环PID
  	const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
	  Power_Control  power_control;
	  Power_Control  rudder_power_control;
	
	  fp32 POWER_MAX_SET;
	  Gimbal_data gimbal_data;
    Rudder_Motor_t Forward_L;
    Rudder_Motor_t Forward_R;
	  Rudder_Motor_t Back_R;
	  Rudder_Motor_t Back_L; 
	  
	  fp32 vx;    //底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;   //底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;  //底盘旋转角速度，逆时针为正 单位 rad/s
	
	  fp32 vx_set;      
    fp32 vy_set;                         
    fp32 wz_set;                         
		fp32 chassis_relative_angle_set; //设置相对云台控制角度
		
		fp32 chassis_relative_last;
		fp32 vx_set_CANsend;      
    fp32 vy_set_CANsend;                         
    fp32 wz_set_CANsend;  
	  int16_t chassis_mode_CANsend;		 //模式
		fp32 chassis_power;
		int16_t chassis_power_MAX;    //裁判系统最大功率，双板通信数据
		int16_t chassis_power_buffer; //缓冲能量，双板通信数据
		int16_t key_C;
			
	  first_order_filter_type_t chassis_cmd_slow_set_vx;   // 滤波数据
		first_order_filter_type_t chassis_cmd_slow_set_vy;
    
		ramp_function_source_t vx_ramp;   //斜坡函数
		ramp_function_source_t vy_ramp;
		
		fp32 rudder_given_current[4];
		fp32 rudder_speed[4];
}chassis_move_t;


extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
