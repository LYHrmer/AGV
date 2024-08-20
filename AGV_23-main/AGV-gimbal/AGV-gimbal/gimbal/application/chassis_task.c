/**
 * @file chassis_task.c
 * @author yuanluochen
 * @brief ˫�岽�����̿������񣬶�ȡң�������ݣ�ͨ��can���߷��͵�����
 * @version 0.1
 * @date 2023-09-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "chassis_task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "can_comm_task.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "bsp_usart.h"
#define abs(x) ((x) > 0 ? (x) : (-x))

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
/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
 * @brief �������ݸ��� 
 * 
 * @param chassis_move_feedback_update  ���̿��ƽṹ��
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_feedback_update); 

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief ���õ��̿�����
 * 
 * @param chassis_move_control ���̿��ƽṹ��
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

extern gimbal_control_t gimbal_control;
extern vision_rxfifo_t *vision_rx;
chassis_move_t chassis_move;       // �����˶�����
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    if (can_comm_task_init_finish())
    {
        // ���̳�ʼ��
        chassis_init(&chassis_move);
        // �жϵ��̵���Ƿ�����
        while (1)
        {
            // ���ݸ���
            chassis_feedback_update(&chassis_move);
            // ���õ��̿���ģʽ
            chassis_set_mode(&chassis_move);
            // ���̿���������
            chassis_set_contorl(&chassis_move);

            if (toe_is_error(DBUS_TOE))
            {
                // ��ң�������߷��Ϳ����ź�Ϊ��
                can_comm_board(0, 0, 0, 0);
            }
            else
            {
                // ���Ϳ�������
                can_comm_board(chassis_move.chassis_relative_ecd, chassis_move.vx_set, chassis_move.vy_set, chassis_move.chassis_behaviour);
				 can_can_comm_referee((int16_t)(power_heat_data_t.chassis_power*100),power_heat_data_t.chassis_power_buffer,
		                                    0/*gimbal_control.key_C*/,robot_state.chassis_power_limit);
            }
            // ϵͳ��ʱ
            vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
            chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
        }
    }

}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //��ȡ��̨�������ָ��
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    //���ݸ���
    chassis_feedback_update(chassis_move_init);
}

/**
 * @brief �������ݸ��� 
 * 
 * @param chassis_move_feedback_update  ���̿��ƽṹ��
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_feedback_update)
{
    chassis_move_feedback_update->chassis_relative_ecd = chassis_move_feedback_update->chassis_yaw_motor->gimbal_motor_measure->ecd - chassis_move_feedback_update->chassis_yaw_motor->frist_ecd;
    if (chassis_move_feedback_update->chassis_relative_ecd > 4096)
        chassis_move_feedback_update->chassis_relative_ecd -= 8191;
    else if (chassis_move_feedback_update->chassis_relative_ecd < -4096)
        chassis_move_feedback_update->chassis_relative_ecd += 8191;
}


/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    // remote control  set chassis behaviour mode
    // ң��������ģʽ
/*     if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        // ң���������²൲λΪ��������ģʽ
        chassis_move_mode->chassis_behaviour = CHASSIS_ZERO_FORCE;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) || switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        // ң�����е��Լ��ϵ�Ϊ��������ģʽ
        if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
        {
            //�������̨
            chassis_move_mode->chassis_behaviour = CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW;
        }
        else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
        {
            //���̸�����̨
            chassis_move_mode->chassis_behaviour = CHASSIS_FOLLOW_GIMBAL_YAW;
        }
        else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
        {
            // ����
            chassis_move_mode->chassis_behaviour = CHASSIS_SPIN;
        }
    } */


    	static int mode = 0;
		if(chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_B )
		{
				mode = 1;
		}
		if(chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_V )
		{
				mode = 2;
		}
    //remote control  set chassis behaviour mode
    //ң��������ģʽ
		if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
             // ң���������²൲λΪ��������ģʽ
        chassis_move_mode->chassis_behaviour = CHASSIS_ZERO_FORCE;
		}	

        else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) || switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
        {
        if(mode == 0){
                        // ң�����е��Լ��ϵ�Ϊ��������ģʽ
                        if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
                        {
                            //�������̨
                            chassis_move_mode->chassis_behaviour = CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW;
                        }
                        else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
                        {
                            //���̸�����̨
                            chassis_move_mode->chassis_behaviour = CHASSIS_FOLLOW_GIMBAL_YAW;
                        }
                        else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_RUN_MODE_CHANNEL]))
                        {
                            // ����
                            chassis_move_mode->chassis_behaviour = CHASSIS_SPIN;
                        }
        }
	
        else{
                        if(mode == 1)
				{
						chassis_move_mode->chassis_behaviour = CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW;
				}
				else if(mode == 2)
				{
						chassis_move_mode->chassis_behaviour = CHASSIS_FOLLOW_GIMBAL_YAW;
                        }
											
						if(chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT )
						{
								chassis_move_mode->chassis_behaviour = CHASSIS_SPIN;
						}
        }
    }
    else if (toe_is_error(DBUS_TOE))
    {
        //���ź�, ��������
        // chassis_move_mode->chassis_behaviour = CHASSIS_ZERO_FORCE;
        chassis_move_mode->chassis_behaviour = CHASSIS_NO_MOVE;
    }
//    else
//    {
//        //������������
//        // chassis_move_mode->chassis_behaviour = CHASSIS_ZERO_FORCE;
//        chassis_move_mode->chassis_behaviour = CHASSIS_NO_MOVE;
//    }
    //when gimbal in some mode, such as init mode, chassis must's move
    //����̨��ĳЩģʽ�£����ʼ���� ���̲���
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_move_mode->chassis_behaviour = CHASSIS_NO_MOVE;
    }
}

/**
 * @brief ���õ��̿�����
 * 
 * @param chassis_move_control ���̿��ƽṹ��
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    fp32 vx_set_channel_RC, vy_set_channel_RC;
    int16_t vx_channel_RC, vy_channel_RC;
    // deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    // �������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel_RC, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_control->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel_RC, CHASSIS_RC_DEADLINE);

    vx_set_channel_RC = vx_channel_RC * CHASSIS_VX_RC_SEN;
    vy_set_channel_RC = vy_channel_RC * CHASSIS_VY_RC_SEN;

    //�����ٶ�
    chassis_move_control->vx_set += vx_set_channel_RC;
    chassis_move_control->vy_set += vy_set_channel_RC;

    
			if (chassis_move_control->chassis_RC->key.v & KEY_PRESSED_OFFSET_A)
			{
	
					chassis_move_control->vy_set = -550;
			}
			else if (chassis_move_control->chassis_RC->key.v & KEY_PRESSED_OFFSET_D)
			{

					chassis_move_control->vy_set = 550;
			}
			else
			{
        chassis_move_control->vy_set = 0;
			}
			
			if (chassis_move_control->chassis_RC->key.v &  KEY_PRESSED_OFFSET_W)
			{

					chassis_move_control->vx_set = 600;
			}
			else if (chassis_move_control->chassis_RC->key.v &  KEY_PRESSED_OFFSET_S)
			{

					chassis_move_control->vx_set = -600;
			}
			else
			{
        chassis_move_control->vx_set = 0;
}
//			    //�����ٶ�
//    chassis_move_control->vx_set += vx_set_channel_RC;
//    chassis_move_control->vy_set += vy_set_channel_RC;
    if(vision_rx->vx != 0  ||  vision_rx->vy!= 0)
	{
    chassis_move_control->vx_set = (vision_rx->vx)*2;
    chassis_move_control->vy_set = (vision_rx->vy)*2;
	}
	else
	{
	chassis_move_control->vx_set += vx_set_channel_RC;
    chassis_move_control->vy_set += vy_set_channel_RC;	
	}

}
