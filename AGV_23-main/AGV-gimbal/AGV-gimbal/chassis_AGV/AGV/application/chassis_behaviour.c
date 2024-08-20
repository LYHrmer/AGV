#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"


/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

//������Ϊ״̬��
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;


/**
  * @brief          ���̿���ģʽ����
  * @author         LYH
  * @param[in]      chassis_move_mode����������
  * @retval         ���ؿ�
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{   

   if (chassis_move_mode->chassis_mode_CANsend==10000)   //���̸�����̨
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }	
    else if (chassis_move_mode->chassis_mode_CANsend==30000)   //С����
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_SPIN;
    }
		else if (chassis_move_mode->chassis_mode_CANsend==20000)   //�������̨
    {
        chassis_behaviour_mode = RUDDER_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
		else                                                     //��ֹ
		{
		   chassis_behaviour_mode = CHASSIS_NO_MOVE;
		}
		
    //������Ϊ״̬��ѡ�����״̬��
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_RAW; //����Ϊ�ǵ��������������õ���״̬��Ϊ raw��ԭ��״̬����
    }		
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //����Ϊ�ǵ��̲��ƶ��������õ���״̬��Ϊ ���̲�����Ƕ� ״̬����
    }	
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //����Ϊ����������������̨�������õ���״̬��Ϊ ���̸�����̨�Ƕ� ״̬����
    }
    else if (chassis_behaviour_mode == RUDDER_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_motor_mode = RUDDER_VECTOR_FOLLOW_GIMBAL_YAW; //����Ϊ�Ƕ������̨�������õ���״̬��Ϊ �������̨�Ƕ� ״̬����
    }		
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_SPIN)
    {
        chassis_move_mode->chassis_motor_mode = CHASSIS_VECTOR_SPIN; //С����
    }		

}

/**
  * @brief          ���̿���������
  * @author         XQL 1.0
	*                 LYH 2.0
  * @param[in]      *vx_set��x�����趨�ٶ�
  * @param[in]      *vy_set��y�����趨�ٶ�
  * @param[in]      *angle_set���趨�Ƕ�
  * @param[in]      chassis_move_rc_to_vector����������
  * @retval         ���ؿ�
  */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }	

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
		else if (chassis_behaviour_mode == RUDDER_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
		else if (chassis_behaviour_mode == CHASSIS_INFANTRY_SPIN)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }	
}



/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}


/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}


/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_set��ǰ�����ٶ�
  * @param[in]      vy_set�����ҵ��ٶ�
  * @param[in]      angle_set����������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector����������
  * @retval         ���ؿ�
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *Chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || Chassis_move_rc_to_vector == NULL)
    {
        return;
    }
		
    chassis_rc_to_control_vector(vx_set, vy_set, Chassis_move_rc_to_vector);
    *angle_set = 0.0f;

}
