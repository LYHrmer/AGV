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
#include "rm_usart.h"
#include "kalman.h"

//���Կ�����ǰ��ϵ��
#define YAW_FEED_FORWARD 0.9f
#define PITCH_FEED_FORWARD -0.95f

//�Ƕ������ϵ��
#define K_YAW_ANGLE_ERROR 50000.0f
#define K_PITCH_ANGLE_ERROR 100000.0f

//�ٶ���ϵ��
#define K_YAW_ANGLE_SPEED 5000.0f
#define K_PITCH_ANGLE_SPEED 4300.0f

//�����С���
#define YAW_MAX_OUT 32000.0f
#define YAW_MIX_OUT -32000.0f
#define PITCH_MAX_OUT 30000.0f
#define PITCH_MIX_OUT -30000.0f

//pitch speed close-loop PID params, max out and max iout
//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP        2000.0f
#define PITCH_SPEED_PID_KI       1.0f
#define PITCH_SPEED_PID_KD        10.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

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
#define GIMBAL_TASK_INIT_TIME 100
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
#define RC_DEADBAND   10


#define YAW_RC_SEN    -0.00001f
#define PITCH_RC_SEN  -0.00001f //0.0050.000005f

#define YAW_MOUSE_SEN   0.0004f
#define PITCH_MOUSE_SEN 0.0003f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

//yaw,pitch�ǶȺ��������ı���
#define Yaw_Mouse_Sen   0.00006f   //80
#define Pitch_Mouse_Sen  0.00007f  //35
#define Z_Mouse_Sen       0.00010f  //10

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//��̨����ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  1
#define YAW_TURN    0

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED     0.004f
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
#define GIMBAL_ACCEL_X_NUM  200.0f
#define GIMBAL_ACCEL_Y_NUM 133.3f
#define GIMBAL_ACCEL_Y_GYRO_NUM 170.3f
#define GIMBAL_ACCEL_Z_NUM 170.3f
#endif

/**
 * @brief ��̨����ģʽ���
 * 
 */

//��̨����ģʽ
#define GIMBAL_AUTO_MODE 1//��������ģʽ����Ҫ�ˣ��򽫸�λ��0����
 //ʹ�ܱ�����̨����ģʽ
#if GIMBAL_AUTO_MODE
/**kalman filter  Q R ����ֵ
 * R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ,��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
 */
// yaw ���
#define GIMBAL_YAW_KALMAN_FILTER_Q 400
#define GIMBAL_YAW_KALMAN_FILTER_R 400
// pitch ��
#define GIMBAL_PITCH_KALMAN_FILTER_Q 200
#define GIMBAL_PITCH_KALMAN_FILTER_R 400

#endif // !GIMBAL_AUTO_MODE

//��̨pitch�����ֵ��ԽǶ�
#define GIMBAL_PITCH_MAX_ENCODE 5112
//��̨pitch����С��Խ�
#define GIMBAL_PITCH_MIN_ENCODE 3842

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
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
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
#if GIMBAL_AUTO_MODE
    // ����������˲�
    kalman gimbal_motor_kalman_filter;
#endif
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
    //�Ӿ���λ������
    const vision_rxfifo_t* gimbal_vision_control;
#endif
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
	

	
    // �˲����ݡ���>ң����
    first_order_filter_type_t gimbal_cmd_slow_set_vx_RC;
    first_order_filter_type_t gimbal_cmd_slow_set_vy_RC;
    // �˲����ݡ���>����
    first_order_filter_type_t gimbal_cmd_slow_set_vx;
    first_order_filter_type_t gimbal_cmd_slow_set_vy;
    first_order_filter_type_t gimbal_cmd_slow_set_vz;
    first_order_filter_type_t gimbal_cmd_slow_set_vy_gyro;
    first_order_filter_type_t gimbal_cmd_slow_set_vx_auto;
    first_order_filter_type_t gimbal_cmd_slow_set_vy_auto;
	 //˫��ͨ�Ų���
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

extern void gimbal_task(void const *pvParameters);

/**
  * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ����
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     pitch ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     pitch �����ԽǶ� ָ��
  * @param[out]     pitch ��С��ԽǶ� ָ��
  * @retval         ����1 ����ɹ�У׼��ϣ� ����0 ����δУ׼��
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  * @param[in]      yaw_offse:yaw ��ֵ
  * @param[in]      pitch_offset:pitch ��ֵ
  * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
  * @param[in]      min_yaw:yaw ��С��ԽǶ�
  * @param[in]      max_yaw:pitch �����ԽǶ�
  * @param[in]      min_yaw:pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
#endif
