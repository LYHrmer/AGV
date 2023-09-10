#ifndef RUDDER_H
#define RUDDER_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"


//舵电机控制周期
#define RUDDER_CONTROL_TIME 1

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10.0f
//#define RC_Pit_deadband 0.5;

//X,Y角度与遥控器输入比例
#define X_RC_SEN   0.0006f
#define Y_RC_SEN -0.0005f //0.005

//X,Y角度和鼠标输入的比例
#define X_Mouse_Sen 0.0002f
#define Y_Mouse_Sen 0.00025f

//X,Y控制通道以及状态开关通道
#define XChannel 2
#define YChannel 3
#define ModeChannel 0

//电机码盘值最大以及中值
#define Half_ecd_range 395  //395  7796
#define ecd_range 8191

//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.00076708402f //      2*  PI  /8192

#endif

typedef enum
{
    RUDDER_MOTOR_RAW = 0, //电机原始值控制
    RUDDER_MOTOR_GYRO,    //电机陀螺仪角度控制
	  RUDDER_MOTOR_ENCONDE,  //电机编码器角度控制
} RUDDER_motor_mode;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;
	  fp32 kf;

	
    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;
	
		fp32 integral_uplimit;			//积分正向抗饱和
	  fp32 integral_downlimit;		//积分负向抗饱和

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	  fp32 Fout;
	
//	  fp32 max_err;
//	  fp32 mid_err;
//		fp32 min_err;
	
	  fp32 F_divider;//前馈分离
		fp32 F_out_limit;//前馈限幅
		
    fp32 out;
		
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;  
    Gimbal_PID_t gimbal_motor_relative_angle_pid;
    PidTypeDef gimbal_motor_gyro_pid;
    RUDDER_motor_mode rudder_motor_mode;
    RUDDER_motor_mode last_rudder_motor_mode;
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
	fp32  ecd_add;       
	fp32 ecd_set;       
	fp32 last_ecd_set;
	fp32 ecd_error;
	fp32 ecd_turn;
	fp32 ture_angle;
	fp32  Flase_angle1;
	fp32  Flase_angle2;
	fp32 wheel_speed;
	fp32 rudder_angle;
	fp32 last_rudder_angle;
	fp32 ecd_zero_set;
} Gimbal_Motor_t;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
   Gimbal_PID_t chassis_pid;
} Chassis_Motor_t;

typedef struct
{        
	  const RC_ctrl_t *gimbal_rc_ctrl;
	  Chassis_Motor_t motor_chassis[4]; 
	  const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    Gimbal_Motor_t gimbal_yaw_motor;
	Gimbal_Motor_t  gimbal_pitch_motor;
    Gimbal_Motor_t Forward_R;
	Gimbal_Motor_t Back_R;
	Gimbal_Motor_t Back_L; 

		first_order_filter_type_t gimbal_cmd_slow_set_vx;   // 滤波数据
		first_order_filter_type_t gimbal_cmd_slow_set_vy;
    fp32 SPIN;
} Gimbal_Control_t;

extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hand_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_Forward_R_motor_point(void);
extern const Gimbal_Motor_t *get_Back_R_motor_point(void);
extern const Gimbal_Motor_t *get_Back_L_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);


//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18



#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f


#endif