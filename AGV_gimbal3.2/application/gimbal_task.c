/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
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
#include "rm_usart.h"
#include "referee.h"
//motor enconde value format, range[0-8191]
//�������ֵ���� 0��8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define Motor_Ecd_to_rad 0.00076708402f
#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }
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
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);

/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_RAW������ֱֵ�ӷ��͵�CAN����.
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          ��GIMBAL_MOTOR_GYROģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          ��GIMBAL_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          ��̨�Ƕ�PID��ʼ��, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      maxout: pid������
  * @param[in]      intergral_limit: pid���������
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
  * @brief          ��̨�Ƕ�PID����, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      get: �Ƕȷ���
  * @param[in]      set: �Ƕ��趨
  * @param[in]      error_delta: ���ٶ�
  * @retval         pid ���
  */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

void Angle_Error_Compare(int now_angle,int zero_angle,int last_zero_angle) ;
/**
 * @brief ��̨�������Կ�������ʼ��
 * 
 * @param controller ��̨���׿������ṹ��
 * @param k_feed_forward ǰ��ϵ��
 * @param k_angle_error �Ƕ����ϵ��
 * @param k_angle_speed ���ٶ�ϵ��
 * @param max_out ������ֵ
 * @param min_out ��С���ֵ
 */
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t* controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out);

/**
 * @brief ��̨���׿��������� 
 * 
 * @param controller ��̨���׿������ṹ��
 * @param set_angle �Ƕ�����ֵ
 * @param cur_angle ��ǰ�Ƕ�
 * @param cur_angle_speed ��ǰ���ٶ� 
 * @param cur_current ��ǰ����
 * @return ����ϵͳ���� ���������ֵ 
 */
static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t* controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current);
void SERIO_Control(void);
/*----------------------------------�ڲ�����---------------------------*/
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
int x_flag=0;  //������־λ�����ڽǶȴ���
int X_FLAG=0;
float MAX_PITCH=8000*Motor_Ecd_to_rad;  float MIN_PITCH=6500*Motor_Ecd_to_rad;
#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2000
int PWM=1130;
/*----------------------------------�ṹ��------------------------------*/
gimbal_control_t gimbal_control;
FuzzyPID fuzzy_pid_gimbal_speed;
FuzzyPID fuzzy_pid_gimbal_angle;
/*----------------------------------�ⲿ����---------------------------*/
extern ExtY_stm32 stm32_Y; 
extern ExtY_stm32 stm32_Y_pitch;
extern TIM_HandleTypeDef htim8;
/*----------------------------------�ⲿ����---------------------------*/
extern void stm32_pid_init_pitch(void);
extern void stm32_relative_pid_init_pitch(void);
extern void sm32_pid_init(void);
extern fp32 INS_angle[3];
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
/**
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
    //�ȴ������������������������
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //��̨��ʼ��
    gimbal_init(&gimbal_control);
    // ��ѭ��
    while (1)
    {
				SERIO_Control();
        gimbal_set_mode(&gimbal_control);                    //������̨����ģʽ
        gimbal_mode_change_control_transit(&gimbal_control); //����ģʽ�л� �������ݹ���
        gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
        gimbal_set_control(&gimbal_control);                 //������̨������
        gimbal_control_loop(&gimbal_control);                //��̨���Ƽ���

        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
							if (toe_is_error(DBUS_TOE))
							 {CAN_cmd_gimbal(0,0,0);
							 CAN_cmd_to_chassis(0,0,0,0);
							 CAN_cmd_to_chassis_key(0,0,0,0);}
					else
					{ CAN_cmd_gimbal(yaw_can_set_current,pitch_can_set_current,0);
				CAN_cmd_to_chassis(gimbal_control.Gimbal_send_RE_angle, gimbal_control.chassis_vx ,
	                       gimbal_control.chassis_vy, gimbal_control.chassis_mode_e_Cansend);  //��̨��Խ�,VX,VY,����ģʽ
				CAN_cmd_to_chassis_key(robot_state.robot_level,power_heat_data_t.chassis_power_buffer,
						             gimbal_control.key_C,robot_state.mains_power_chassis_output);  //�����˵ȼ���
//		 CAN_cmd_to_chassis_key(robot_state.robot_level,power_heat_data_t.chassis_power,gimbal_control.key_C,robot_state.mains_power_chassis_output);  //�����˵ȼ���
					}
      vTaskDelay(1);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

int B_flag=0;
void SERIO_Control(void)
{
	if(((gimbal_control.gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_B)||gimbal_control.gimbal_rc_ctrl->rc.ch[4]<-100)&&B_flag==0)
	{   B_flag=1;
		if(PWM==1130)
		{
		PWM=1800;
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,PWM);
		}
       else if(PWM==1800)
	   {
		   PWM=1130;
       __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,PWM);
	   }		   
	}
	if(((gimbal_control.gimbal_rc_ctrl->key.v &KEY_PRESSED_OFFSET_B)==0)&&(gimbal_control.gimbal_rc_ctrl->rc.ch[4]>-10))
	{
	B_flag=0;	
	}
	if(PWM == 1130)
	{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          ����pitch �������ָ��
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{
    const static fp32 gimbal_x_order_filter[1] = {GIMBAL_ACCEL_X_NUM};
    const static fp32 gimbal_y_order_filter[1] = {GIMBAL_ACCEL_Y_NUM + 7};
    const static fp32 gimbal_y_gyro_order_filter[1] = {GIMBAL_ACCEL_Y_GYRO_NUM};
    const static fp32 gimbal_z_order_filter[1] = {GIMBAL_ACCEL_Z_NUM};
    const static fp32 gimbal_x_order_filter_RC[1] = {GIMBAL_ACCEL_X_NUM};
    const static fp32 gimbal_y_order_filter_RC[1] = {GIMBAL_ACCEL_Y_NUM};
    const static fp32 gimbal_x_order_filter_auto[1] = {GIMBAL_ACCEL_X_NUM - 50};
    const static fp32 gimbal_y_order_filter_auto[1] = {GIMBAL_ACCEL_Y_NUM};

    //�����̸�����̨ģʽ�õ�
    gimbal_control.gimbal_yaw_motor.frist_ecd = 3367;
		gimbal_control.gimbal_yaw_motor.ZERO_ECD_flag = 3367;
		gimbal_control.gimbal_yaw_motor.LAST_ZERO_ECD = 3367;

    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    volatile static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    // �������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    // ����������ָ���ȡ
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    // ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();
#if GIMBAL_AUTO_MODE
    // �Ӿ�����ָ���ȡ
    init->gimbal_vision_control = get_vision_rxfifo_point();
#endif 
    // ��ʼ�����ģʽ
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    // һ�׵�ͨ�˲���ʼ��
    first_order_filter_init(&init->gimbal_cmd_slow_set_vx, GIMBAL_CONTROL_TIME, gimbal_x_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy, GIMBAL_CONTROL_TIME, gimbal_y_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy_gyro, GIMBAL_CONTROL_TIME, gimbal_y_gyro_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vx_RC, GIMBAL_CONTROL_TIME, gimbal_x_order_filter_RC);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy_RC, GIMBAL_CONTROL_TIME, gimbal_y_order_filter_RC);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vz, GIMBAL_CONTROL_TIME, gimbal_z_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vx_auto, GIMBAL_CONTROL_TIME, gimbal_x_order_filter_auto);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy_auto, GIMBAL_CONTROL_TIME, gimbal_y_order_filter_auto);
    // ��ʼ��pitch���pid
    stm32_pid_init();
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    // �������PID
    gimbal_total_pid_clear(init);
    gimbal_feedback_update(init);
    // yaw������ʼ��
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    // pitch������ʼ��
    init->gimbal_pitch_motor.offset_ecd=7252;
		
		init->angle_max=-0.20f;
		init->angle_min=0.38f;

    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.offset_ecd*Motor_Ecd_to_rad;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
		
    init->gimbal_yaw_motor.offset_ecd = 3367;
		    gimbal_motor_second_order_linear_controller_init(&init->gimbal_pitch_motor.gimbal_motor_second_order_linear_controller, PITCH_FEED_FORWARD, K_PITCH_ANGLE_ERROR, K_PITCH_ANGLE_SPEED, PITCH_MAX_OUT, PITCH_MIX_OUT);
    gimbal_motor_second_order_linear_controller_init(&init->gimbal_yaw_motor.gimbal_motor_second_order_linear_controller, YAW_FEED_FORWARD, K_YAW_ANGLE_ERROR, K_YAW_ANGLE_SPEED, YAW_MAX_OUT, YAW_MIX_OUT);

			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,PWM);
		

#if GIMBAL_AUTO_MODE
    //��ʼ��һάkalman filter
    kalmanCreate(&init->gimbal_yaw_motor.gimbal_motor_kalman_filter, GIMBAL_YAW_KALMAN_FILTER_Q, GIMBAL_PITCH_KALMAN_FILTER_R); 
    kalmanCreate(&init->gimbal_pitch_motor.gimbal_motor_kalman_filter, GIMBAL_PITCH_KALMAN_FILTER_Q, GIMBAL_PITCH_KALMAN_FILTER_R); 
#endif
}

/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
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
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
		//pitch�Ჿ��
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->gimbal_pitch_motor.relative_angle = feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd*Motor_Ecd_to_rad ;
//	feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
//                                                                                  feedback_update->gimbal_pitch_motor.offset_ecd);
		feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
		//yaw�Ჿ��
		feedback_update->gimbal_yaw_motor.absolute_angle = INS_angle[0];
//  feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_yaw_motor.frist_ecd);   
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))     
                                                   - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
		//˫��ͨ�Ų���
		feedback_update->Gimbal_send_RE_angle=feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd-feedback_update->gimbal_yaw_motor.frist_ecd;
//  if(fabs(feedback_update->Gimbal_send_RE_angle)<=70)  //{feedback_update->Gimbal_send_RE_angle=0;}
		if(feedback_update->Gimbal_send_RE_angle>4096) {feedback_update->Gimbal_send_RE_angle =feedback_update->Gimbal_send_RE_angle-8191 ;}
		else if(feedback_update->Gimbal_send_RE_angle<-4096) feedback_update->Gimbal_send_RE_angle=feedback_update->Gimbal_send_RE_angle+8191;
		
}

/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
	fp32 relative_ecd = ecd - offset_ecd;
	
    if (relative_ecd > gimbal_control.gimbal_yaw_motor.frist_ecd+4096)
    {
       // relative_ecd -= 8191;
    }
    return relative_ecd * Motor_Ecd_to_rad;
}

/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     gimbal_mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw���״̬���л���������
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

    //pitch���״̬���л���������
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
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
		int16_t vx_channel_RC, vy_channel_RC;
		
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    //yaw���ģʽ����
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    //pitch���ģʽ����
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
		
		
		 //ģʽ����
		 if switch_is_up(set_control->gimbal_rc_ctrl->rc.s[1])
		{
			if (set_control->gimbal_rc_ctrl->key.v &  KEY_PRESSED_OFFSET_F || set_control->gimbal_rc_ctrl->rc.ch[4]>50)
			{
				set_control->chassis_mode_e_Cansend=20000;      //�������̨
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
			}
			else if (set_control->gimbal_rc_ctrl->key.v &  KEY_PRESSED_OFFSET_R)
			{
				set_control->chassis_mode_e_Cansend=10000;
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
			}
			else if (set_control->gimbal_rc_ctrl->key.v &  KEY_PRESSED_OFFSET_SHIFT)
			{
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);   //С����
				set_control->chassis_mode_e_Cansend=30000;
			}
			else if ((set_control->gimbal_rc_ctrl->key.v &  KEY_PRESSED_OFFSET_CTRL))
			{
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
				set_control->chassis_mode_e_Cansend=40000;       //���̲���
			}
			else
			{		
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);				
				set_control->chassis_mode_e_Cansend=10000;    //���̸�����̨
			}
				//��������
			
			if (set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_A)
			{
				if(robot_state.robot_level==1)
					set_control->chassis_vy = -520;
				else if(robot_state.robot_level==2)
					set_control->chassis_vy = -550;
				else if(robot_state.robot_level==3)
					set_control->chassis_vy = -590;
				else
					set_control->chassis_vy = -550;
			}
			else if (set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_D)
			{
				if(robot_state.robot_level==1)
					set_control->chassis_vy = 520;
				else if(robot_state.robot_level==2)
					set_control->chassis_vy =550;
				else if(robot_state.robot_level==3)
					set_control->chassis_vy = 590;
        else
					set_control->chassis_vy = 550;
			}
			else
			{
        set_control->chassis_vy = 0;
			}
			
			if (set_control->gimbal_rc_ctrl->key.v &  KEY_PRESSED_OFFSET_W)
			{
				if(robot_state.robot_level==1)
					set_control->chassis_vx = 520;
				else if(robot_state.robot_level==2)
					set_control->chassis_vx = 565;
				else if(robot_state.robot_level==3)
					set_control->chassis_vx = 610;
				else
					set_control->chassis_vx = 550;
			}
			else if (set_control->gimbal_rc_ctrl->key.v &  KEY_PRESSED_OFFSET_S)
			{
				if(robot_state.robot_level==1)
					set_control->chassis_vx = -520;
				else if(robot_state.robot_level==2)
					set_control->chassis_vx = -550;
				else if(robot_state.robot_level==3)
					set_control->chassis_vx = -560;
				else
					set_control->chassis_vx = -550;
			}
			else
			{
        set_control->chassis_vx = 0;
			}
			
			
			if (set_control->gimbal_rc_ctrl->mouse.press_r)
			{
				set_control->key_C=8000;
			}
			else 
				set_control->key_C=0;	
		//deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[1], vx_channel_RC, 60);
    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[0], vy_channel_RC, 60);
			
		set_control->chassis_vx+=vx_channel_RC;
		set_control->chassis_vy+=vy_channel_RC;
			
		}
			else if switch_is_mid(set_control->gimbal_rc_ctrl->rc.s[1])
		{
			set_control->chassis_mode_e_Cansend=30000;
	 //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[1], vx_channel_RC, 60);
    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[0], vy_channel_RC, 60);
			
		set_control->chassis_vx=vx_channel_RC;
		set_control->chassis_vy=vy_channel_RC;

		}
		else if switch_is_down(set_control->gimbal_rc_ctrl->rc.s[1])
		{
			    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[1], vx_channel_RC, 15);
    rc_deadband_limit(set_control->gimbal_rc_ctrl->rc.ch[0], vy_channel_RC, 15);
			set_control->chassis_mode_e_Cansend=40000;
						if(set_control->gimbal_rc_ctrl->rc.ch[1]>60||set_control->gimbal_rc_ctrl->rc.ch[1]<-60)
			{set_control->chassis_vx=set_control->gimbal_rc_ctrl->rc.ch[1];}
			else 
				set_control->chassis_vx=0;
			if(set_control->gimbal_rc_ctrl->rc.ch[0]>60||set_control->gimbal_rc_ctrl->rc.ch[0]<-60)
			{set_control->chassis_vy=set_control->gimbal_rc_ctrl->rc.ch[0];}
			else 
				set_control->chassis_vy=0;
		}


}
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
	 static fp32 angle_set_yaw;  static fp32 angle_set_pitch;
   if (gimbal_motor == NULL)
   {
        return;
   }

		if(gimbal_motor==&gimbal_control.gimbal_yaw_motor)
	{
    angle_set_yaw = gimbal_motor->absolute_angle_set; //����������
//		gimbal_motor->absolute_angle_set = rad_format(angle_set_yaw + add);
		gimbal_motor->absolute_angle_set = angle_set_yaw + add;

	}
	else
	{
			angle_set_pitch=gimbal_motor->absolute_angle_set;
		  gimbal_motor->absolute_angle_set=rad_format(angle_set_pitch+ add);

		 if(gimbal_motor->absolute_angle_set<gimbal_control.angle_max)
		{
			gimbal_motor->absolute_angle_set=gimbal_control.angle_max;
		}
		else if(gimbal_motor->absolute_angle_set>gimbal_control.angle_min)
		{
			gimbal_motor->absolute_angle_set=gimbal_control.angle_min;
		}
	
	}
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		
    gimbal_motor->relative_angle_set += add;

    
	if( gimbal_motor->relative_angle_set == gimbal_control.gimbal_yaw_motor.relative_angle_set)
	{
        if (gimbal_motor->relative_angle_set < 0)
        {
			gimbal_motor->relative_angle_set = 2 * PI + gimbal_motor->relative_angle_set;	
		}
        else if (gimbal_motor->relative_angle_set > 2 * PI)
        {
            gimbal_motor->relative_angle_set = gimbal_motor->relative_angle_set - 2 * PI;
        }
  }
	
	  else if( gimbal_motor->relative_angle_set==gimbal_control.gimbal_pitch_motor.relative_angle_set)
	{
		if( gimbal_motor->relative_angle_set<MIN_PITCH)
		{
			gimbal_motor->relative_angle_set=MIN_PITCH;	
		}
	  else if( gimbal_motor->relative_angle_set >MAX_PITCH)
	  {
		  gimbal_motor->relative_angle_set=MAX_PITCH;
	  }
  }

}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
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

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
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
			stm32_pid_init();
      stm32_step(gimbal_motor->absolute_angle_set, gimbal_motor->absolute_angle, 0);
	  gimbal_motor->current_set=stm32_Y.Out1 ;
		gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
			
	}
	else if(gimbal_motor==&gimbal_control.gimbal_pitch_motor)
	{
		
		stm32_pid_init_pitch();
		stm32_step_pitch(gimbal_motor->absolute_angle_set,gimbal_motor->absolute_angle, 0);
		gimbal_motor->current_set=stm32_Y_pitch.Out1 ;

    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		
		
    }
}
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
//     stm32_step(gimbal_motor->relative_angle_set, gimbal_motor->relative_angle, 0);
//     gimbal_motor->current_set = stm32_Y.Out1;
//     gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);	
	}
	else if(gimbal_motor==&gimbal_control.gimbal_pitch_motor)
	{
		stm32_relative_pid_init_pitch();
		stm32_step_pitch(gimbal_motor->relative_angle_set, gimbal_motor->relative_angle, 0);
    gimbal_motor->current_set = stm32_Y_pitch.Out1;
    // ����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
	}
}


//�ͽ���λ�Ƕȴ���ȡ�ӻ�
void Angle_Error_Compare(int now_angle,int zero_angle,int last_zero_angle)  
{
	fp32 flag_angle[2]={0};
	if(zero_angle>4096)
	{
		flag_angle[0]=abs((now_angle-zero_angle));
		if(flag_angle[0]>4096)  flag_angle[0]=8191-	zero_angle+now_angle;	
    flag_angle[1]=abs((now_angle-last_zero_angle));
    if(flag_angle[1]>4096)  flag_angle[1]=8191-	now_angle+last_zero_angle;	
    if(flag_angle[0]>flag_angle[1])
		{
		 zero_angle-=4096;	X_FLAG++;
		 gimbal_control.gimbal_yaw_motor.frist_ecd=zero_angle;
		}
	}
	else if(zero_angle<=4096)
	{ 
		flag_angle[0]=abs((now_angle-zero_angle));
		if(flag_angle[0]>4096)  flag_angle[0]=8191-	now_angle+zero_angle;		
    flag_angle[1]=abs((now_angle-last_zero_angle));
    if(flag_angle[1]>4096)  flag_angle[1]=8191-	last_zero_angle+now_angle;		
    if(	flag_angle[0]>flag_angle[1])
		{		
		zero_angle+=4096;	X_FLAG++;
		gimbal_control.gimbal_yaw_motor.frist_ecd=zero_angle;
		}
	}	
}


/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_RAW������ֱֵ�ӷ��͵�CAN����.
  * @param[out]     gimbal_motor:yaw�������pitch���
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

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
		stm32_Y.Out1=0;
		stm32_Y_pitch.Out1=0;
}











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
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}



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
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
        //�������ʱ������ݣ���Ϊ��ʼ���ݣ����ж������Сֵ
        gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle      = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle      = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle    = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle    = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          ��̨У׼���㣬��У׼��¼����ֵ,��� ��Сֵ
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     pitch ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     pitch �����ԽǶ� ָ��
  * @param[out]     pitch ��С��ԽǶ� ָ��
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);
    
    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}
/**
 * @brief ��̨�������Կ�������ʼ��
 *
 * @param controller ��̨���׿������ṹ��
 * @param k_feed_forward ǰ��ϵ��
 * @param k_angle_error �Ƕ����ϵ��
 * @param k_angle_speed ���ٶ�ϵ��
 * @param max_out ������ֵ
 * @param min_out ��С���ֵ
 */
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out)
{
    // ǰ����ϵ��
    controller->k_feed_forward = k_feed_forward;
    // ��������ϵ��
    controller->k_angle_error = k_angle_error;
    controller->k_angle_speed = k_angle_speed;
    // ����������ֵ
    controller->max_out = max_out;
    // ������С���ֵ
    controller->min_out = min_out;
}

/**
 * @brief ��̨���׿���������
 * @param controller ��̨���׿������ṹ��
 * @param set_angle �Ƕ�����ֵ
 * @param cur_angle ��ǰ�Ƕ�
 * @param cur_angle_speed ��ǰ���ٶ�
 * @param cur_current ��ǰ����
 * @return ����ϵͳ����
 */
static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t *controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current)
{
    // ��ֵ
    controller->cur_angle = cur_angle;
    controller->set_angle = set_angle;
    controller->cur_angle_speed = cur_angle_speed;
    // ����ǰ����ֵ����һ��С��1��ϵ�������赲ϵͳ�����Ŷ���ǰ����
    controller->feed_forward = controller->k_feed_forward * cur_current;
    // ������� = �趨�Ƕ� - ��ǰ�Ƕ�
    controller->angle_error = controller->set_angle - controller->cur_angle;
    // �����ֵ���� -PI ~ PI ֮��
    controller->angle_error = rad_format(controller->angle_error);
    // �������ֵ = ǰ��ֵ + �Ƕ����ֵ * ϵ�� + ���ٶ� * ϵ��
    controller->output = controller->feed_forward + controller->angle_error * controller->k_angle_error + (-controller->cur_angle_speed * controller->k_angle_speed);

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