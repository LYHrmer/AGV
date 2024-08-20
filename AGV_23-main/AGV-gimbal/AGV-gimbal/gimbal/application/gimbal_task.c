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
#include "can_comm_task.h"
#include "chassis_task.h"
#include "bsp_usart.h"
// motor enconde value format, range[0-8191]
// �������ֵ���� 0��8191
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
 * @brief ��̨�������Կ�������ʼ��
 * 
 * @param controller ��̨�������Կ������ṹ��
 * @param k_feed_forward ǰ��ϵ��
 * @param k_angle_error �Ƕ����ϵ��
 * @param k_angle_speed ���ٶ�ϵ��
 * @param max_out ������ֵ
 * @param min_out ��С���ֵ
 */
static void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t* controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out);

/**
 * @brief ��̨�������Կ��������� 
 * 
 * @param controller ��̨�������Կ������ṹ��
 * @param set_angle �Ƕ�����ֵ
 * @param cur_angle ��ǰ�Ƕ�
 * @param cur_angle_speed ��ǰ���ٶ� 
 * @param cur_current ��ǰ����
 * @return ����ϵͳ���� ���������ֵ 
 */
static fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t* controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current);
extern chassis_move_t chassis_move;
//��̨����ṹ��
gimbal_control_t gimbal_control;

vision_rxfifo_t *vision_rx;

/**
 * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void gimbal_task(void const *pvParameters)
{
    // �ȴ������������������������
    // wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    if (can_comm_task_init_finish())
    {
        // gimbal init
        // ��̨��ʼ��
        gimbal_init(&gimbal_control);
//        //�жϵ���Ƿ�����
//       while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
//       {
//           // �ȴ�������ߣ���ֹ���������
//           vTaskDelay(GIMBAL_CONTROL_TIME);
//           gimbal_feedback_update(&gimbal_control); // ��̨���ݷ���
//       }
        while (1)
        {
            gimbal_set_mode(&gimbal_control);                    // ������̨����ģʽ
            gimbal_mode_change_control_transit(&gimbal_control); // ����ģʽ�л� �������ݹ���
            gimbal_feedback_update(&gimbal_control);             // ��̨���ݷ���
            gimbal_set_control(&gimbal_control);                 // ������̨������
            gimbal_control_loop(&gimbal_control);                // ��̨���Ƽ���

            if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
            {
                if (toe_is_error(DBUS_TOE))
                {
                    // �ж�ң�����Ƿ����
                    CAN_cmd_gimbal(0, 0);
                }
                else
                {
                    CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.given_current, -gimbal_control.gimbal_pitch_motor.given_current);
//                    can_can_comm_referee((int16_t)(power_heat_data_t.chassis_power*100),power_heat_data_t.chassis_power_buffer,
//		                                    0/*gimbal_control.key_C*/,robot_state.chassis_power_limit);
                }
            }

            vTaskDelay(GIMBAL_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
            gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
    }
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

    const static fp32 gimbal_yaw_auto_scan_order_filter[1] = {GIMBAL_YAW_AUTO_SCAN_NUM};
    const static fp32 gimbal_pitch_auto_scan_order_filter[1] = {GIMBAL_PITCH_AUTO_SCAN_NUM};
	
	

    // �����̸�����̨ģʽ�õ�
    gimbal_control.gimbal_yaw_motor.zero_ecd_flag = GIMBAL_YAW_LAST_OFFSET_ENCODE;
    gimbal_control.gimbal_yaw_motor.last_zero_ecd = GIMBAL_YAW_LAST_OFFSET_ENCODE;

    // ��̨������Խ��� 
    gimbal_control.gimbal_yaw_motor.frist_ecd = GIMBAL_YAW_OFFSET_ENCODE;
    // �������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    // ����������ָ���ȡ
    init->gimbal_INS_point = get_INS_point();
    // ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();
    // ��ȡ��λ���Ӿ�����ָ��
    init->gimbal_vision_point = get_vision_gimbal_point();
    // ��ȡ�Զ��ƶ��ṹ��
    init->auto_move_point = get_auto_move_point();
    // ��ʼ�����ģʽ
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //��ʼ����̨������׿�����
    gimbal_motor_second_order_linear_controller_init(&init->gimbal_yaw_motor.gimbal_motor_second_order_linear_controller, YAW_FEED_FORWARD, K_YAW_ANGLE_ERROR, K_YAW_ANGLE_SPEED, YAW_MAX_OUT, YAW_MIX_OUT);
    gimbal_motor_second_order_linear_controller_init(&init->gimbal_pitch_motor.gimbal_motor_second_order_linear_controller, PITCH_FEED_FORWARD, K_PITCH_ANGLE_ERROR, K_PITCH_ANGLE_SPEED, PITCH_MAX_OUT, PITCH_MIX_OUT);

    // ��̨���ݸ���     
    gimbal_feedback_update(init);
    
    // yaw������ʼ��
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    // pitch������ʼ��
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

     //��ʼ����̨�Զ�ɨ���ͨ�˲�
    first_order_filter_init(&init->gimbal_auto_scan.pitch_auto_scan_first_order_filter, GIMBAL_CONTROL_TIME, gimbal_pitch_auto_scan_order_filter);
    first_order_filter_init(&init->gimbal_auto_scan.yaw_auto_scan_first_order_filter, GIMBAL_CONTROL_TIME, gimbal_yaw_auto_scan_order_filter);

    //yaw����̨��ʼ����ԽǶ�
    init->gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ENCODE;
    // pitch����̨��ʼ����ԽǶ�
    init->gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_OFFSET_ENCODE; 

    // ��ʼ����̨�Զ�ɨ��ṹ���ɨ�跶Χ
    init->gimbal_auto_scan.pitch_range = PITCH_SCAN_RANGE;
    init->gimbal_auto_scan.yaw_range = YAW_SCAN_RANGE;

    // ��ʼ����̨�Զ�ɨ������
    init->gimbal_auto_scan.scan_pitch_period = PITCH_SCAN_PERIOD;
    init->gimbal_auto_scan.scan_yaw_period = YAW_SCAN_PERIOD;

    /* //��ȡ��̨�Զ�ɨ���ʼ��ʱ��
    init->gimbal_auto_scan.scan_begin_time = TIME_MS_TO_S(HAL_GetTick());
    //pitch��ɨ������ֵ
    init->gimbal_auto_scan.pitch_center_value = init->gimbal_auto_scan.pitch_range; */

    // ����pitch����Խ����ֵ
    init->gimbal_pitch_motor.max_relative_angle = -motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, init->gimbal_pitch_motor.offset_ecd);
    init->gimbal_pitch_motor.min_relative_angle = -motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, init->gimbal_pitch_motor.offset_ecd);

	vision_rx=get_vision_fifo();
	
//	vision_rx->vx = 0.3f;
//	vision_rx->ang_z = 0.3f;
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
    // ��̨���ݸ���
    feedback_update->gimbal_pitch_motor.absolute_angle = feedback_update->gimbal_INS_point->Pitch;
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                    feedback_update->gimbal_pitch_motor.offset_ecd);
    feedback_update->gimbal_pitch_motor.motor_gyro = feedback_update->gimbal_INS_point->Gyro[0];

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
 * @brief          ����ecd��offset_ecd֮�����ԽǶ�
 * @param[in]      ecd: �����ǰ����
 * @param[in]      offset_ecd: �����ֵ����
 * @retval         ��ԽǶȣ���λrad
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
    // yaw���״̬���л���������
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

    // pitch���״̬���л���������
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

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    // yaw���ģʽ����
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    // pitch���ģʽ����
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
 * @param[out]     gimbal_motor:yaw�������pitch���
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
    }
    else
    {
        // ��ǰ���Ƕ�
        static fp32 error_angle = 0;
        static fp32 angle_set = 0;
        error_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
        // ��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
        if (gimbal_motor->relative_angle + error_angle + add < gimbal_motor->max_relative_angle)
        {
            // �����������е�Ƕȿ��Ʒ���
            if (add < 0.0f)
            {
                // �����һ��������ӽǶȣ�
                add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - error_angle;
            }
        }
        else if (gimbal_motor->relative_angle + error_angle + add > gimbal_motor->min_relative_angle)
        {
            if (add > 0.0f)
            {
                add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - error_angle;
            }
        }
        angle_set = gimbal_motor->absolute_angle_set;
        gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
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

    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        // ��̨yaw����ԽǶ�����
        if (gimbal_motor->relative_angle_set < 0)
        {
            gimbal_motor->relative_angle_set = 2 * PI + gimbal_motor->relative_angle_set;
        }
        else if (gimbal_motor->relative_angle_set > 2 * PI)
        {
            gimbal_motor->relative_angle_set = gimbal_motor->relative_angle_set - 2 * PI;
        }
    }
    else if (gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        // ��̨pitch����ԽǶ����ƣ���ֹpitch��ת������
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
    //������̨������Ƶ���
    gimbal_motor->current_set = gimbal_motor_second_order_linear_controller_calc(&gimbal_motor->gimbal_motor_second_order_linear_controller, gimbal_motor->absolute_angle_set, gimbal_motor->absolute_angle, gimbal_motor->motor_gyro, gimbal_motor->gimbal_motor_measure->given_current);
    //��ֵ����ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);

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
    //������̨������Ƶ���
    gimbal_motor->current_set = gimbal_motor_second_order_linear_controller_calc(&gimbal_motor->gimbal_motor_second_order_linear_controller, gimbal_motor->relative_angle_set, gimbal_motor->relative_angle, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->gimbal_motor_measure->given_current);
    //��ֵ����ֵ
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);

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


/**
 * @brief ��̨�������Կ�������ʼ��
 *
 * @param controller ��̨�������Կ������ṹ��
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
 * @brief ��̨�������Կ���������
 *
 * @param controller ��̨�������Կ������ṹ��
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

    //�������ֵ����ֹ���ֵ�����������
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

//��ȡ����������
fp32 get_yaw_positive_direction(void)
{
    return gimbal_control.yaw_positive_direction;
}

