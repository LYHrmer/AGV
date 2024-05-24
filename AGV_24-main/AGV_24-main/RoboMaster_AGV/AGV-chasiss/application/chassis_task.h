#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "Remote_Control.h"
#include "pid.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "stm32.h"
#include "struct_typedef.h"

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 2
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

#define CHASSIS_RC_DEADLINE 10

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.2f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.2f
#define MOTOR_DISTANCE_TO_CENTER   0.32 //0.32f

#define DEG2R(x) ((x)*PI /180.0f)

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002

//���̵�ͨ�˲�ϵ��
#define CHASSIS_ACCEL_X_NUM 0.02f
#define CHASSIS_ACCEL_Y_NUM 0.01f

//X��Y�Ƕ���ң�����������
#define X_RC_SEN   0.0006f
#define Y_RC_SEN -0.0005f //0.005

//Y,Y�ǶȺ��������ı���
#define X_Mouse_Sen 0.0002f
#define Y_Mouse_Sen 0.00025f

//X,Y����ͨ��
#define X_Channel 2
#define Y_Channel 3

//�������ֵ����Լ���ֵ
#define Half_ecd_range 395  //395  7796
#define ecd_range 8191

//����-����
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.00076708402f //      2*  PI  /8192
#endif

//���̵������ٶ�
#define MAX_WHEEL_SPEED 15.0f

//���ʿ�����ز���
#define toque_coefficient 1.99688994e-6f // (20/16384)*(0.3)*(187/3591)/9.55  �˲������������ת��ΪŤ��
#define k2 1.23e-07						 // ͭ��ϵ��
#define k1 1.453e-07					 // ����ϵ��
#define constant_3508 4.081f  //��������̬���

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//�����ı���ֵ���
#define Forward_L_ecd 680  // ��ǰ��  1��
#define Forward_R_ecd 2668 // ��ǰ��  2��
#define Back_L_ecd 2740	   // �����  3��
#define Back_R_ecd 7574	   // �Һ���  4��

//�����ı���ֵ���
//#define Forward_L_ecd 5759  // ��ǰ��  1��
//#define Forward_R_ecd 7875 // ��ǰ��  2��
//#define Back_L_ecd 7727	   // �����  3��
//#define Back_R_ecd 4220	   // �Һ���  4��

//���̵���ٶȻ�PID
#define CHASSIS_KP 3000.f  //2000
#define CHASSIS_KI 0.0f //0
#define CHASSIS_KD 20.f //20
#define CHASSIS_MAX_OUT 16000.f
#define CHASSIS_MAX_IOUT 2000.f


// �ǶȲ�������
#define Power_120_AngleCompensation -0.25
#define Power_100_AngleCompensation -0.20
#define Power_80_AngleCompensation -0.15
#define Power_70_AngleCompensation -0.15
#define Power_60_AngleCompensation -0.13
#define Power_50_AngleCompensation -0.10

//����PID
#define RUDDER_P_P 15.0f     //18.1
#define RUDDER_P_I 0.0f      //0
#define RUDDER_P_D 2.0f      //2.0
#define RUDDER_P_N 1.5f      //1.5
#define RUDDER_S_P 1.0f     //1.0
#define RUDDER_S_I 0.0f      //0.0
#define RUDDER_S_D 0.0f      //0.0
#define RUDDER_S_N 0.0f      //0.0


//������ת����PID

#define CHASSIS_FOLLOW_GIMBAL_PID_KP  5.0f//3.0f               // 3.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI  0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD  80.0f//20.f//10.0f 
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT   4//4
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KF 0.0f
#define CHASSIS_FOLLOW_GIMBAL_F_divider 0.0
#define CHASSIS_FOLLOW_GIMBAL_F_out_limit 0.0f

//���̵�����ʻ�PID
#define M3505_MOTOR_POWER_PID_KP 1.4f//1.0f
#define M3505_MOTOR_POWER_PID_KI 0.0f
#define M3505_MOTOR_POWER_PID_KD 0.0f
#define M3505_MOTOR_POWER_PID_MAX_OUT 15.0f  //60 
#define M3505_MOTOR_POWER_PID_MAX_IOUT 0.0f

//��������ʱ�ķŵ��С����
#define CAP_OUTPUT_to_CHASSIS 8000
#define CAP_OUTPUT_to_CHASSIS_FLY 7000

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//1/60*0.003335*3.1415926
#define GM6020_RPM_TO_VECTOR 0.001746201886833

//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.02f

//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.02f

//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //���̸�����̨
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //��������
	CHASSIS_VECTOR_SPIN,                //С����
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //���̲�����
  CHASSIS_VECTOR_RAW,									//����ԭʼ����
  RUDDER_VECTOR_FOLLOW_GIMBAL_YAW     //�������̨
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
	fp32 power_charge; //������
	fp32 forecast_motor_power[4]; // Ԥ�ⵥ���������
	fp32 forecast_total_power; // Ԥ���ܹ���
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
    const motor_measure_t *gimbal_motor_measure;   //��̨����
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
    fp32 motor_gyro_set;    //���ٶ��趨
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
	 
	  PidTypeDef chassis_angle_pid;   //���̸���Ƕ�pid
		PidTypeDef buffer_pid;     //���ʻ�PID
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
	  
	  fp32 vx;    //�����ٶ� ǰ������ ǰΪ������λ m/s
    fp32 vy;   //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
    fp32 wz;  //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
	  fp32 vx_set;      
    fp32 vy_set;                         
    fp32 wz_set;                         
		fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
		
		fp32 chassis_relative_last;
		fp32 vx_set_CANsend;      
    fp32 vy_set_CANsend;                         
    fp32 wz_set_CANsend;  
	  int16_t chassis_mode_CANsend;		 //ģʽ
		fp32 chassis_power;
		int16_t chassis_power_MAX;    //����ϵͳ����ʣ�˫��ͨ������
		int16_t chassis_power_buffer; //����������˫��ͨ������
		int16_t key_C;
			
	  first_order_filter_type_t chassis_cmd_slow_set_vx;   // �˲�����
		first_order_filter_type_t chassis_cmd_slow_set_vy;
    
		ramp_function_source_t vx_ramp;   //б�º���
		ramp_function_source_t vy_ramp;
		
		ramp_function_source_t_2 vx_ramp_2;   //б�º���
		ramp_function_source_t_2 vy_ramp_2;
		
		fp32 rudder_given_current[4];
		fp32 rudder_speed[4];
		
		fp32 pitch_relative_angle;
		fp32 pitch_absolute_angle;
		fp32 pitch_angle_error;
}chassis_move_t;


extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
