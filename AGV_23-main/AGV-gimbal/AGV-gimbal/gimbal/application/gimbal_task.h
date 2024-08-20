/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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
//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP         2000.0f
#define PITCH_SPEED_PID_KI         1.0f
#define PITCH_SPEED_PID_KD         10.0f
#define PITCH_SPEED_PID_MAX_OUT    30000.0f
#define PITCH_SPEED_PID_MAX_IOUT   10000.0f

//yaw speed close-loop PID params, max out and max iout
//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP        3600.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 80.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 1.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 5.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 100//401
//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0
//turn 180��
//��ͷ180 ����
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//��ͷ��̨�ٶ�
#define TURN_SPEED    0.04f
//���԰�����δʹ��
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
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

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
////��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
//#define GIMBAL_INIT_ANGLE_ERROR     0.01f
//#define GIMBAL_INIT_STOP_TIME       100
//#define GIMBAL_INIT_TIME            5000
//#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED     0.001f//0.003f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//��̨У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//�ж�ң�����������ʱ���Լ�ң�����������жϣ�������̨yaw����ֵ�Է�������Ư��
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

//��ͨ�˲�ϵ��
#define GIMBAL_YAW_AUTO_SCAN_NUM 133.3f
#define GIMBAL_PITCH_AUTO_SCAN_NUM 133.3f

//��̨pitch�����ֵ��ԽǶ�  0x004D
#define GIMBAL_PITCH_MAX_ENCODE 2627//0x004D
//��̨pitch����С��Խ�
#define GIMBAL_PITCH_MIN_ENCODE 1635//0x1796
//��̨pitch����ֵ
#define GIMBAL_PITCH_OFFSET_ENCODE 1973
//��̨yaw����ֵ
#define GIMBAL_YAW_OFFSET_ENCODE 2379
//yaw������ֵ
#define GIMBAL_YAW_LAST_OFFSET_ENCODE (((GIMBAL_YAW_OFFSET_ENCODE + HALF_ECD_RANGE) > ECD_RANGE) ? (GIMBAL_YAW_OFFSET_ENCODE + HALF_ECD_RANGE - ECD_RANGE) : (GIMBAL_YAW_OFFSET_ENCODE + HALF_ECD_RANGE))
//��̨yaw�����������

#define INS_YAW_ERROR 0


//��̨��ʼ���������ֵ
#define INIT_STOP_COUNT  200
//��̨ҡ��yaw��ҡ���˶��޷�,������Ϊyaw����ֵ�����߼��޵ĽǶȷ�Χ,�ý�Ϊ���Խ�
#define GIMBAL_YAW_SWING_RANGE (PI / 2)
//��̨pitch��ҡ���˶�����������ֵ
#define GIMBAL_PITCH_SWING_DOWN_ECD GIMBAL_PITCH_MAX_ENCODE
//��̨pitch��ҡ���˶�����������ֵ
#define GIMBAL_PITCH_SWING_UO_ECD GIMBAL_PITCH_MIN_ENCODE
//��̨yaw���˶�����(��λΪrad)
#define GIMBAL_YAW_SWING_STE 0.01f;
//��̨pitch���˶�����(��λΪrad)
#define GIMBAL_PITCH_SWING_STEP 0.01f
//��̨yaw��pitch���������ʱ��
#define GIMBAL_SWING_STOP_COUNT 1000

//yaw��ɨ�跶Χ��������Ϊ���� �����Χ
#define YAW_SCAN_RANGE  PI
//pitch��ɨ�跶Χ��������ֵΪ����
#define PITCH_SCAN_RANGE 0.20f

//yaw��ɨ�貽�� rad/S
#define YAW_SCAN_SPEED 1.2f
//pitch��ɨ�貽�� rad/s
#define PITCH_SCAN_SPEED 1.2f

//yaw��ɨ������
#define YAW_SCAN_PERIOD (2 * YAW_SCAN_RANGE / YAW_SCAN_SPEED)
//pitch��ɨ������
#define PITCH_SCAN_PERIOD (2 * PITCH_SCAN_RANGE / PITCH_SCAN_SPEED)


//���Կ�����ǰ��ϵ��
#define YAW_FEED_FORWARD 0.9f
#define PITCH_FEED_FORWARD 0.95f

//�Ƕ������ϵ��
#define K_YAW_ANGLE_ERROR 80000.0f//60000.0f
#define K_PITCH_ANGLE_ERROR 450000.0f//850000.0f//550000.0f//450000.0f


//�ٶ���ϵ��
#define K_YAW_ANGLE_SPEED 6000.0f//5500.0f
#define K_PITCH_ANGLE_SPEED 5000.0f//7000.0f//8000.0f//3500.0f

//�����С���
#define YAW_MAX_OUT 32000.0f
#define YAW_MIX_OUT -32000.0f
#define PITCH_MAX_OUT 30000.0f
#define PITCH_MIX_OUT -30000.0f

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

//�ڱ�ɨ��ṹ�壬��������λ��δʶ��Ŀ��ʱ�Զ�ɨ����׼��
typedef struct 
{
    // ɨ���ͨ�˲��ṹ��
    first_order_filter_type_t pitch_auto_scan_first_order_filter;
    first_order_filter_type_t yaw_auto_scan_first_order_filter;

    //yaw������ֵ
    fp32 yaw_center_value;
    //pitch������ֵ
    fp32 pitch_center_value;


    //yaw���˶�����
    fp32 yaw_range;
    //pitch���˶�����
    fp32 pitch_range;

    //��ǰ����ʱ�� ��λs
    fp32 scan_run_time;
    //��ʼ��ʱʱ�� ��λs
    fp32 scan_begin_time;

    //yaw��ɨ������ ��λs
    fp32 scan_yaw_period;
    //pitch��ɨ������  ��λs
    fp32 scan_pitch_period;

} scan_t;


//��̨����������Կ�����
typedef struct 
{
    //�趨ֵ
    fp32 set_angle;
    //��ǰ�Ƕ�   һ��״̬
    fp32 cur_angle;
    //��ǰ���ٶ� ����״̬
    fp32 cur_angle_speed;
    //�Ƕ������ һ��״̬���
    fp32 angle_error;
    //ǰ�����������ϵͳ�����Ŷ�
    fp32 feed_forward;
    //���ֵ
    fp32 output;
    //������ֵ
    fp32 max_out;
    //��С���ֵ
    fp32 min_out;

    //ǰ����ϵ��
    fp32 k_feed_forward;
    //�����ϵ��
    fp32 k_angle_error;
    //���׽��ٶ���ϵ��
    fp32 k_angle_speed;

}gimbal_motor_second_order_linear_controller_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;

    //�������Կ�����
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
    //Զ��ң����ָ��
    const RC_ctrl_t *gimbal_rc_ctrl;

    //��ȡ�Ӿ���λ������
    const gimbal_vision_control_t* gimbal_vision_point;
    //��̨�Զ��ƶ��ṹ��
    const auto_move_t* auto_move_point;

    //�Զ�ɨ��ṹ��
    scan_t gimbal_auto_scan;

    //����yaw��������
    fp32 yaw_positive_direction;
    
    const INS_t* gimbal_INS_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;

	fp32 right_click_time;
	
} gimbal_control_t;


/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          ����pitch �������ָ��
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

//��ȡ����������
fp32 get_yaw_positive_direction(void);

extern void gimbal_task(void const *pvParameters);

#endif
