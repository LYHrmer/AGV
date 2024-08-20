#include "chassis_task.h"   
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_laser.h"
#include "pid.h"
#include "Remote_Control.h"
#include "pid.h"
#include "arm_math.h"
#include "INS_Task.h"
#include "CAN_Receive.h"
#include "chassis_behaviour.h"
#include "stm32.h"
#include "struct_typedef.h"
#include "math.h"
extern cap_measure_t get_cap;

#define pi 3.1415926
#define half_pi 1.5707963

/** 
  * @brief          ��������
	* @author         XQL
  * @param[in]      input��  ����ֵ
	* @param[in]      output�� ���ֵ
	* @param[in]      dealine������ֵ
  * @retval         none
  */
#define rc_deadline_limit(input, output, dealine)        \
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
  * @brief         ȡ����ֵ
	* @author        XQL
  * @param[in]     ����
  * @retval        none
  */		
#define abs(x) ((x) > 0 ? (x) : (-x))

		
/** 
  * @brief          �������ֵ���� 0��8191
	* @author         XQL
  * @param[in]      �������ֵ
  * @retval         none
  */		
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }                           \
	
	
/**
  * @brief 	        ����ת����
  * @author         XQL
  * @param[in]      angle������ֵ
  * @retval         none
  */
#define rand(angle)  angle=angle/8191*3.1415926f*2 

	
/**
  * @brief 	        ȡ��Сֵ
	* @author         XQL
  * @param[in]      angle1������Ƕ�1
	* @param[in]      angle2������Ƕ�2
	* @param[in]      ture��  ����Ƕ�
  * @retval         none
  */
#define compare(angle1,angle2,ture) \
   {                                \
      if(angle1>angle2)             \
	     {                            \
         ture=angle2;               \
	     }                            \
      else                          \
      {                             \
	      ture=angle1;                \
	    }	                            \
   }	                            
 
	 
/**
  * @brief 	        �Ƕ�ת����
  * @author         XQL
  * @param[in]      angle������Ƕ�
  * @retval         none
  */                            
#define rad(angle) angle=angle/180*3.1415926f  

//���̿��������������
chassis_move_t chassis_move;	 
int power_flag1=0, power_flag2=0;
fp32 power_cet;fp32 power_charge=0;
int UI_flag=0;
fp32 kx=1.f,ky=1.f,kw=1.f; //�ٶ�ת���ļ���ϵ��
int	linkState_2=0;
int	linkState_1=0;  //�ж�˫��ͨ���Ƿ�����
int rudder_t=0;
int std_vector=0;
int16_t ecd_ready=0;  //�ж϶����Ƿ񵽴�Ŀ��λ��
/** 
  * @brief          ���ض��� 6020�������ָ��
	* @author         XQL
  * @param[in]      none
  * @retval         �������ָ��
  */
const Rudder_Motor_t *get_Forward_L_motor_point(void)
{
    return &chassis_move.Forward_L;
}
const Rudder_Motor_t *get_Forward_R_motor_point(void)
{
	return &chassis_move.Forward_R;
}
const Rudder_Motor_t *get_Back_R_motor_point(void)
{
	return &chassis_move.Back_R;
}
const Rudder_Motor_t *get_Back_L_motor_point(void)
{
	return &chassis_move.Back_L;
}

//����PID��ʼ��
static void RUDDER_PID_INIT(chassis_move_t*rudder_init,const fp32 PID[8]);
//�ֵ���ٶ�����
static void chassis_speed_control_set(chassis_move_t*chassis_speed_set);
//���̳�ʼ��
static void chassis_init(chassis_move_t *chassis_move_init);
//�������ݸ���
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//����ģʽ����
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//�����л�ģʽ״̬����
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//���̿���������
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//�֣���Ƕȣ��ٶȳ�������
static void chassic_rudder_preliminary_A_S_solution(chassis_move_t*chassic_rudder_preliminary_solution);
//���������
static void rudder_control_loop(chassis_move_t *rudder_move_control_loop);
//�ֵ���������
static void CHASSIC_MOTOR_PID_CONTROL(chassis_move_t *chassis_motor);
//�����������
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t *rudder_motor);
//���������
static void Rudder_motor_relative_angle_control(Rudder_Motor_t *chassis_motor);
//ң��������������������
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
//���������߽紦��
static void Rudder_motor_ecd_judge(Rudder_Motor_t *Rudder_ecd_judge);
//����������̨��㴦��
static void Rudder_motor_zero_ecd_set(chassis_move_t *Rudder_ecd_set);
//��Ƕ����Ž����������
static void rudder_optimal_angle_solution(Rudder_Motor_t *rudder_optimal_angle);
//�������ʿ���
static void RUDDER_POWER_CONTROL(chassis_move_t *rudder_power);
//�ֵ����̬���ʿ���
void chassis_power_move_control(chassis_move_t *chassis_motor);
//�������ݳ��
void Power_Charge(fp32 power);
//ɲ��ģʽ
int Rear_Brake();
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
/** 
  * @brief          ��������
  * @author         XQL 1.0
	*                 LYH 2.0
  * @param[in]      chassis_move_init����������ָ��
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{      
		chassis_init(&chassis_move);  
		while(1)
		{	 
		chassis_set_mode(&chassis_move); 
		chassis_mode_change_control_transit(&chassis_move);
		chassis_feedback_update(&chassis_move);	
		chassis_set_contorl(&chassis_move);
		rudder_control_loop(&chassis_move);
		RUDDER_POWER_CONTROL(&chassis_move);
    CHASSIC_MOTOR_PID_CONTROL(&chassis_move);

		linkState_1++;   
		linkState_2++;
		if(linkState_1>50||linkState_2>50)  //��˫��ͨ��û���յ����ݣ������ֶ�����
		{
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_rudder(0,0,0,0);
		}		
		else
		{
				CAN_cmd_rudder(chassis_move.rudder_given_current[0],chassis_move.rudder_given_current[1],
			             chassis_move.rudder_given_current[2],chassis_move.rudder_given_current[3]);	
				vTaskDelay(1);
				CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current,chassis_move.motor_chassis[1].give_current,
										chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
			
			CAN_cmd_gimbal(get_cap.capvot*100, 0); //���͵��ݵ�ѹ
			
		}
		vTaskDelay(2);  //��Ϊ1���ܻ���ַ賵����Ϊͨ��Ƶ��
			
		}
}

fp32 motor_speed_pid[3]={CHASSIS_KP, CHASSIS_KI, CHASSIS_KD};
fp32 rudder_speed_pid[3]={20, 0, 0};
fp32 rudder_angle_pid[3]={10, 0, 0};
const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};		
fp32 rudder_pid[8]={RUDDER_P_P, RUDDER_P_I, RUDDER_P_D,RUDDER_P_N,RUDDER_S_P,RUDDER_S_I,RUDDER_S_D,RUDDER_S_N};

/** 
  * @brief          ��ʼ����������
  * @author         XQL 1.0
	*                 LYH 2.0
  * @param[in]      chassis_move_init����������ָ��
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	

	  if (chassis_move_init == NULL)
    {
        return;
    }
		const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	  //��������ָ���ȡ
	  chassis_move_init->Forward_L.gimbal_motor_measure = get_Forward_L_motor_measure_point(); 
	  chassis_move_init->Forward_R.gimbal_motor_measure = get_Forward_R_motor_measure_point(); 
	  chassis_move_init->Back_R.gimbal_motor_measure = get_Back_R_motor_measure_point(); 
	  chassis_move_init->Back_L.gimbal_motor_measure = get_Back_L_motor_measure_point(); 
  
		chassis_move_init->chassis_motor_mode = CHASSIS_VECTOR_RAW;
		
		//�ֵ������ָ���ȡ��PID��ʼ��
    int i;		
	  for(i=0;i<4;i++)
     {	
	   chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
     PID_Init(&chassis_move_init->motor_chassis[i].chassis_pid, PID_POSITION, motor_speed_pid, CHASSIS_MAX_OUT, CHASSIS_MAX_IOUT);
     }

		 //����PID��ʼ��
		 RUDDER_PID_INIT(chassis_move_init,rudder_pid);
		 //������̨PID
		 PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
     //ң��������ָ���ȡ
     chassis_move_init->chassis_rc_ctrl = get_remote_control_point();
		 //һ�׵�ͨ�˲���ʼ��
		 first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
     first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
		 //�ֵ��ת�������ʼ��
		 chassis_move_init->Forward_L.Judge_Speed_Direction=chassis_move_init->Forward_R.Judge_Speed_Direction=
		 chassis_move_init->Back_L.Judge_Speed_Direction=chassis_move_init->Back_R.Judge_Speed_Direction=1.0f;
		  //�������ݳ�ʼ��
		 chassis_move_init->chassis_relative_last=0.0f;
		 chassis_move_init->relative_angle_Dbuf=0.0f;
     chassis_feedback_update(chassis_move_init);
		 //��������ֵ��ʼ��
		 chassis_move.Forward_L.ecd_zero_set=1544;
		 chassis_move.Forward_R.ecd_zero_set=1010;  //5033
		 chassis_move.Back_R.ecd_zero_set=3625;  
		 chassis_move.Back_L.ecd_zero_set=5168; 
		 	chassis_move.power_control.SPEED_MIN=0.1f; 
			

}


/** 
  * @brief          ���µ�������
  * @author         XQL  1.0
	*                 LYH  2.0
  * @param[in]      chassis_move_update����������ָ��
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }	
		
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {   
    chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		}
		
    chassis_move_update->rudder_speed[0]=chassis_move_update->Forward_L.gimbal_motor_measure->speed_rpm*GM6020_RPM_TO_VECTOR;
    chassis_move_update->rudder_speed[1]=chassis_move_update->Back_L.gimbal_motor_measure->speed_rpm*GM6020_RPM_TO_VECTOR;
    chassis_move_update->rudder_speed[2]=chassis_move_update->Back_R.gimbal_motor_measure->speed_rpm*GM6020_RPM_TO_VECTOR;
    chassis_move_update->rudder_speed[3]=chassis_move_update->Forward_R.gimbal_motor_measure->speed_rpm*GM6020_RPM_TO_VECTOR;		
		
		//��̨����ԽǶ�
		chassis_move.gimbal_data.relative_angle=((fp32)(chassis_move.gimbal_data.relative_angle_receive))*Motor_Ecd_to_Rad;
		chassis_move.relative_angle_Dbuf=chassis_move.chassis_relative_last-chassis_move.gimbal_data.relative_angle;
		chassis_move.chassis_relative_last=chassis_move.gimbal_data.relative_angle;
		
}


/** 
  * @brief          ���̿���ģʽ����
  * @author         XQL
  * @param[in]      chassis_move_mode����������ָ��
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
	  chassis_behaviour_mode_set(chassis_move_mode);
}


/** 
  * @brief          �����л�ģʽ���ݻ���
  * @author         XQL 1.0
	*                 LYH 2.0
  * @param[in]      chassis_move_transit����������ָ��
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL||chassis_move_transit->last_chassis_motor_mode == chassis_move_transit->chassis_motor_mode)
    {
        return;
    }
		int i = 0;
    if ((chassis_move_transit->last_chassis_motor_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) &&    //�л������̸�����̨
			   chassis_move_transit->chassis_motor_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
         chassis_move_transit->chassis_relative_angle_set = 0.0f;  
			   //���¶������
		 chassis_move.Forward_L.ecd_zero_set=1544;
		 chassis_move.Forward_R.ecd_zero_set=1010;  //5033
		     chassis_move_transit->Back_R.ecd_zero_set=3625;  
					     chassis_move_transit->Back_L.ecd_zero_set=5168; 
				 for (i = 0; i < 4; i++)
					PID_clear(&chassis_move_transit->motor_chassis[i].chassis_pid);
    }
    else if ((chassis_move_transit->last_chassis_motor_mode != CHASSIS_VECTOR_SPIN) &&      //�л���С����ģʽ
			        chassis_move_transit->chassis_motor_mode == CHASSIS_VECTOR_SPIN)
    {
         chassis_move_transit->chassis_relative_angle_set = 0.0f;
						   //���¶������
		 chassis_move.Forward_L.ecd_zero_set=1544;
		 chassis_move.Forward_R.ecd_zero_set=1010;  //5033
		chassis_move_transit->Back_R.ecd_zero_set=3625;  
	  chassis_move_transit->Back_L.ecd_zero_set=5168; 
				 for (i = 0; i < 4; i++)
					PID_clear(&chassis_move_transit->motor_chassis[i].chassis_pid);
    } 
	  else if ((chassis_move_transit->last_chassis_motor_mode != RUDDER_VECTOR_FOLLOW_GIMBAL_YAW) &&   //�л����������̨ģʽ
			        chassis_move_transit->chassis_motor_mode == RUDDER_VECTOR_FOLLOW_GIMBAL_YAW)
    {
				chassis_move_transit->chassis_relative_angle_set = 0.0f;
				chassis_move_transit->wz_set = 0.0f;
				chassis_move_transit->vx_set = 0.0f;
				chassis_move_transit->vy_set = 0.0f;
			vTaskDelay(500);
			vTaskDelay(500);
//				chassis_move.Forward_L.ecd_zero_set=1544;
//		    chassis_move.Forward_R.ecd_zero_set=1173;  //5033
//		    chassis_move_transit->Back_R.ecd_zero_set=3625;  
//				chassis_move_transit->Back_L.ecd_zero_set=5168;
				for (i = 0; i < 4; i++)
				 PID_clear(&chassis_move_transit->motor_chassis[i].chassis_pid);
    }
    chassis_move_transit->last_chassis_motor_mode = chassis_move_transit->chassis_motor_mode;
}
 

/** 
  * @brief          ����������������ü�ת��
  * @author         LYH 1.0
  * @param[in]      chassis_move_control����������ָ��
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
	
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
		fp32 relative_angle=0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
		
		if(chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)  //���̸�����̨ģʽ
		{       
			  fp32 sin_yaw,cos_yaw=0.0f;	   fp32 relative_angle=0.0f;	
			  chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
			  relative_angle=chassis_move_control->gimbal_data.relative_angle;
			  if(relative_angle>PI) relative_angle=-2*PI+relative_angle;
			  
			  sin_yaw = arm_sin_f32((relative_angle));
        cos_yaw = arm_cos_f32((relative_angle));		
				
        chassis_move_control->vx_set= cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -1.0f*sin_yaw * vx_set + cos_yaw * vy_set;	
	
				if(fabs(relative_angle)<0.1f&&(chassis_move_control->vx_set_CANsend==0.0f&&chassis_move_control->vy_set_CANsend==0.0f))    //һ������²���Բ�����ٶȸ���
				{
					chassis_move_control->wz_set = 0;
				}
				else
				{	
					if(Rear_Brake(chassis_move_control)==0)
						chassis_move_control->wz_set = 0;
					else
					chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, relative_angle, chassis_move_control->chassis_relative_angle_set); 

				}	
			}	
		else if(chassis_move_control->chassis_motor_mode == RUDDER_VECTOR_FOLLOW_GIMBAL_YAW)  //�������̨ģʽ
		{   				
			  fp32 sin_yaw,cos_yaw=0.0f;	   fp32 relative_angle=0.0f;	
			  chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
			  relative_angle=chassis_move_control->gimbal_data.relative_angle;
			  if(relative_angle>PI) relative_angle=-2*PI+relative_angle;
			  
			  chassis_move_control->vx_set =  -chassis_move_control->vx_set_CANsend/100;
				chassis_move_control->vy_set =  -chassis_move_control->vy_set_CANsend/100;
//				sin_yaw = arm_sin_f32((relative_angle));
//        cos_yaw = arm_cos_f32((relative_angle));			
//        //�����
//        chassis_move_control->vx_set= cos_yaw * vx_set + sin_yaw * vy_set;
//        chassis_move_control->vy_set = -1.0f*sin_yaw * vx_set + cos_yaw * vy_set;	
//				chassis_move_control->wz_set = 0;
				Rudder_motor_zero_ecd_set(chassis_move_control);
		}
		else if(chassis_move_control->chassis_motor_mode==CHASSIS_VECTOR_SPIN)  //С����ģʽ
		{
				fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
			  relative_angle=chassis_move_control->gimbal_data.relative_angle;
			
			  if(relative_angle>PI) relative_angle=-2*PI+relative_angle;
				else if(relative_angle<-PI) relative_angle=2*PI+relative_angle;
				else relative_angle=relative_angle;
			
				sin_yaw = arm_sin_f32((relative_angle));
        cos_yaw = arm_cos_f32((relative_angle));
			
//				Rudder_motor_zero_ecd_set(chassis_move_control);
        chassis_move_control->vx_set= cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -1.0f*sin_yaw * vx_set + cos_yaw * vy_set;
			chassis_move_control->chassis_relative_angle_set = rad_format(0.0);
//				chassis_move_control->wz_set=1.0f;
			if(fabs(chassis_move_control->vx_set_CANsend)>20 || fabs(chassis_move_control->vy_set_CANsend)>20)
			{

					chassis_move_control->wz_set =1.0f;
      }
			else   //��ԭ��ʱ���Ӵ�ת��
				{
				chassis_move_control->wz_set= 4.0f;
				}
		}
		else if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set =vy_set;
			  chassis_move_control->wz_set = angle_set;
    }
    else if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
    }
		
		chassic_rudder_preliminary_A_S_solution(chassis_move_control);
}


/** 
  * @brief          ��Ƕ����Ž����������
  * @author         XQL  1.0
	*                 LYH  2.0
  * @param[in]      rudder_optimal_angle����������ָ��
  * @retval         none
  */
static void rudder_optimal_angle_solution(Rudder_Motor_t *rudder_optimal_angle)
{ 
	  rudder_optimal_angle->ecd_temp_error=rudder_optimal_angle->ecd_add-rudder_optimal_angle->last_ecd_add;
	
	  if(rudder_optimal_angle->ecd_temp_error>2048)
		{
			rudder_optimal_angle->ecd_add-=4096;
			rudder_optimal_angle->Judge_Speed_Direction=-1;
		}
	  else if(rudder_optimal_angle->ecd_temp_error<-2048)
		{
			rudder_optimal_angle->ecd_add+=4096;
			rudder_optimal_angle->Judge_Speed_Direction=-1;
		}
		else
		{
			rudder_optimal_angle->Judge_Speed_Direction=1;
		}
}


/** 
  * @brief          �֣�������Ƕ��ٶȿ���������
  * @author         XQL  1.0
	*                 LYH  2.0
  * @param[in]      chassic_rudder_preliminary_solution����������ָ��
  * @retval         ��������΢���ع�
  */
static void chassic_rudder_preliminary_A_S_solution(chassis_move_t*chassic_rudder_preliminary_solution)
{
	  fp32 vx_set,vy_set,vw_set=0.0f;
	  vx_set=chassic_rudder_preliminary_solution->vx_set;
	  vy_set=chassic_rudder_preliminary_solution->vy_set;
	  vw_set=chassic_rudder_preliminary_solution->wz_set;
	  //���¶����Ƕ�
	  chassic_rudder_preliminary_solution->Forward_L.last_rudder_angle=chassic_rudder_preliminary_solution->Forward_L.rudder_angle;
	  chassic_rudder_preliminary_solution->Back_L.last_rudder_angle=chassic_rudder_preliminary_solution->Back_L.rudder_angle;
	  chassic_rudder_preliminary_solution->Back_R.last_rudder_angle=chassic_rudder_preliminary_solution->Back_R.rudder_angle;
	  chassic_rudder_preliminary_solution->Forward_R.last_rudder_angle=chassic_rudder_preliminary_solution->Forward_R.rudder_angle; 
	
	
	  //���������ٶ����ֵ��ת��
    chassic_rudder_preliminary_solution->Forward_L.wheel_speed
		=-sqrt((pow((vy_set+vw_set*arm_cos_f32(45)),2)+pow((vx_set+vw_set*arm_sin_f32(45)),2)));
	chassic_rudder_preliminary_solution->Back_L.wheel_speed
		=-sqrt((pow((vy_set-vw_set*arm_cos_f32(45)),2)+pow((vx_set+vw_set*arm_sin_f32(45)),2)));
	chassic_rudder_preliminary_solution->Back_R.wheel_speed
		=sqrt((pow((vy_set-vw_set*arm_cos_f32(45)),2)+pow((vx_set-vw_set*arm_sin_f32(45)),2)));
	chassic_rudder_preliminary_solution->Forward_R.wheel_speed
		=sqrt((pow((vy_set+vw_set*arm_cos_f32(45)),2)+pow((vx_set-vw_set*arm_sin_f32(45)),2)));
	
	
	//�����ٶȷ����Ǻ�����Ƕ�                                                                                                                              //��ʱ����ת
	chassic_rudder_preliminary_solution->Forward_L.rudder_angle
		=atan2((vy_set+vw_set*arm_cos_f32(45)),(vx_set+vw_set*arm_sin_f32(45)));                   // 0  3 
	chassic_rudder_preliminary_solution->Back_L.rudder_angle
		=atan2((vy_set-vw_set*arm_cos_f32(45)),(vx_set+vw_set*arm_sin_f32(45)));                   // 1  2
	chassic_rudder_preliminary_solution->Back_R.rudder_angle
		=atan2((vy_set-vw_set*arm_cos_f32(45)),(vx_set-vw_set*arm_sin_f32(45)));
	chassic_rudder_preliminary_solution->Forward_R.rudder_angle
		=atan2((vy_set+vw_set*arm_cos_f32(45)),(vx_set-vw_set*arm_sin_f32(45)));	
															
															
    //�����ֵ����
		chassic_rudder_preliminary_solution->Forward_L.ecd_add = chassic_rudder_preliminary_solution->Forward_L.rudder_angle/Motor_Ecd_to_Rad;
		chassic_rudder_preliminary_solution->Back_L.ecd_add = chassic_rudder_preliminary_solution->Back_L.rudder_angle/Motor_Ecd_to_Rad;
		chassic_rudder_preliminary_solution->Back_R.ecd_add = chassic_rudder_preliminary_solution->Back_R.rudder_angle/Motor_Ecd_to_Rad;
		chassic_rudder_preliminary_solution->Forward_R.ecd_add = chassic_rudder_preliminary_solution->Forward_R.rudder_angle/Motor_Ecd_to_Rad;
	  //���ŽǴ���
		rudder_optimal_angle_solution(&chassic_rudder_preliminary_solution->Forward_L);  
		rudder_optimal_angle_solution(&chassic_rudder_preliminary_solution->Back_L);
		rudder_optimal_angle_solution(&chassic_rudder_preliminary_solution->Back_R);
		rudder_optimal_angle_solution(&chassic_rudder_preliminary_solution->Forward_R);
	  //���ݸ���
		chassic_rudder_preliminary_solution->Forward_L.last_ecd_add=chassic_rudder_preliminary_solution->Forward_L.ecd_add;
		chassic_rudder_preliminary_solution->Back_L.last_ecd_add=chassic_rudder_preliminary_solution->Back_L.ecd_add;
		chassic_rudder_preliminary_solution->Back_R.last_ecd_add=chassic_rudder_preliminary_solution->Back_R.ecd_add;
		chassic_rudder_preliminary_solution->Forward_R.last_ecd_add=chassic_rudder_preliminary_solution->Forward_R.ecd_add;
		
}


/** 
  * @brief          �ֵ���ٶ�����
  * @param[in]      chassis_speed_set����������ָ��
  * @retval         none
  */
static void chassis_speed_control_set(chassis_move_t*chassis_speed_set)
{ 
	
	chassis_speed_set->motor_chassis[0].speed_set
		=chassis_speed_set->Forward_L.wheel_speed*chassis_speed_set->Forward_L.Judge_Speed_Direction;
	chassis_speed_set->motor_chassis[1].speed_set
		=chassis_speed_set->Forward_R.wheel_speed*chassis_speed_set->Forward_R.Judge_Speed_Direction;
	chassis_speed_set->motor_chassis[2].speed_set
		=chassis_speed_set->Back_L.wheel_speed*chassis_speed_set->Back_L.Judge_Speed_Direction;
	chassis_speed_set->motor_chassis[3].speed_set
		=chassis_speed_set->Back_R.wheel_speed*chassis_speed_set->Back_R.Judge_Speed_Direction;	
	
	int i;
	fp32 temp,max_vector,vector_rate;
	
	for (i = 0; i < 4; i++)
  {
      temp = fabs(chassis_speed_set->motor_chassis[i].speed_set);
      if (max_vector < temp)
      {
         max_vector = temp;
      }
  }
	//��������ٶ�
   if (max_vector > MAX_WHEEL_SPEED)
   {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_speed_set->motor_chassis[i].speed_set *= vector_rate;
        }
    }
    
}


/** 
  * @brief          �����������������
  * @param[in]      chassis_move_control_loop����������ָ��
  * @retval         none
  */

static void rudder_control_loop(chassis_move_t *rudder_move_control_loop)
{   
    Rudder_motor_relative_angle_control(&rudder_move_control_loop->Forward_L);
	  Rudder_motor_relative_angle_control(&rudder_move_control_loop->Back_L);
	  Rudder_motor_relative_angle_control(&rudder_move_control_loop->Back_R);
	  Rudder_motor_relative_angle_control(&rudder_move_control_loop->Forward_R); 
}

/** 
  * @brief          ���������߽紦��
  * @author         XQL
  * @param[in]      Rudder_ecd_judge
  * @retval         none
  */
static void Rudder_motor_ecd_judge(Rudder_Motor_t *Rudder_ecd_judge)
{
	
	  if(Rudder_ecd_judge->ecd_zero_set>8191)
		{
			Rudder_ecd_judge->ecd_set=Rudder_ecd_judge->ecd_zero_set-8191;
		}
		else if(Rudder_ecd_judge->ecd_zero_set<0)
		{
			Rudder_ecd_judge->ecd_set=Rudder_ecd_judge->ecd_zero_set+8191;
		}
	
}
	
/** 
  * @brief          ����������̨��㴦��
  * @author         LYH
  * @param[in]      Rudder_ecd_set
  * @retval         none
  */
static void Rudder_motor_zero_ecd_set(chassis_move_t *Rudder_ecd_set)
{
	  fp32 err=0.0f;
	
	  err=-1.0f*(chassis_move.gimbal_data.relative_angle_receive-chassis_move.chassis_relative_last);
	
		Rudder_ecd_set->Forward_L.ecd_zero_set= (Rudder_ecd_set->Forward_L.ecd_zero_set+err);
	  Rudder_ecd_set->Back_L.ecd_zero_set= (Rudder_ecd_set->Back_L.ecd_zero_set+err);
	  Rudder_ecd_set->Back_R.ecd_zero_set= (Rudder_ecd_set->Back_R.ecd_zero_set+err);
	  Rudder_ecd_set->Forward_R.ecd_zero_set= (Rudder_ecd_set->Forward_R.ecd_zero_set+err);
	
	  chassis_move.chassis_relative_last=chassis_move.gimbal_data.relative_angle_receive;
	
	  //��㴦��
	  Rudder_motor_ecd_judge(&Rudder_ecd_set->Forward_L);
	  Rudder_motor_ecd_judge(&Rudder_ecd_set->Back_L);
	  Rudder_motor_ecd_judge(&Rudder_ecd_set->Back_R);
	  Rudder_motor_ecd_judge(&Rudder_ecd_set->Forward_R);
}


/** 
  * @brief          ��������������
  * @author         XQL  1.0
	*                 LYH  2.0
  * @param[in]      chassis_motor����������ָ��
  * @retval         none
  */

static void Rudder_motor_relative_angle_control(Rudder_Motor_t *chassis_motor)
{   
	
	//����Ŀ��λ��
	 if(chassis_motor->ecd_add>0)
	 {
			if(chassis_motor->ecd_zero_set+chassis_motor->ecd_add>8191)
			 {
					chassis_motor->ecd_set=chassis_motor->ecd_zero_set+chassis_motor->ecd_add-8191;
			 }
			else if(chassis_motor->ecd_zero_set+chassis_motor->ecd_add<8191)
			 {
					chassis_motor->ecd_set=chassis_motor->ecd_zero_set+chassis_motor->ecd_add;
			 }
   }
	else if(chassis_motor->ecd_add<0)
	{
			if(chassis_motor->ecd_zero_set+chassis_motor->ecd_add<0)
			 {
					chassis_motor->ecd_set=chassis_motor->ecd_zero_set+chassis_motor->ecd_add+8191;
			 }
			else if(chassis_motor->ecd_zero_set+chassis_motor->ecd_add>0)
			 {
					chassis_motor->ecd_set=chassis_motor->ecd_zero_set+chassis_motor->ecd_add;
			 }
	}	

	else if(chassis_motor->ecd_add==0.0f&&fabs(chassis_move.relative_angle_Dbuf)<0.3f)
{
	chassis_motor->ecd_set=chassis_motor->last_ecd_set;
}
else if(chassis_move.chassis_motor_mode==CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW&&chassis_motor->ecd_add==0.0f)
{
	chassis_motor->ecd_set=chassis_motor->gimbal_motor_measure->ecd;
}
	else if(chassis_move.chassis_motor_mode==CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW&&fabs(chassis_move.vx_set_CANsend)<100.0f&&fabs(chassis_move.vy_set_CANsend)<100.0f)
{
	chassis_motor->ecd_set=chassis_motor->gimbal_motor_measure->ecd;
}
else if(chassis_move.chassis_motor_mode!=CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW&&chassis_motor->ecd_add==0.0f)
{  	
		chassis_motor->ecd_set=chassis_motor->ecd_zero_set;	  
} 

	chassis_motor->ecd_error=chassis_motor->ecd_set-chassis_motor->gimbal_motor_measure->ecd;
	chassis_motor->last_ecd_set = chassis_motor->ecd_set;
  //�ͽ�ԭ��

	  if(chassis_motor->ecd_error>4096)
		{
			chassis_motor->ecd_error=chassis_motor->ecd_error-8191;
		}
		else if(chassis_motor->ecd_error<-4096)
		{
			chassis_motor->ecd_error=8191+chassis_motor->ecd_error;
		}

		
	RUDDER_MOTOR_PID_CONTROL(chassis_motor);
}



/** 
  * @brief          ������������������
  * @param[in]      rudder_motor����������ָ��
  * @retval         none
  */	
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t *rudder_motor)   
{
	
	
Matlab_PID_Calc(rudder_motor->ecd_error,0,0,&rudder_motor->rudder_control);
rudder_motor->given_current=rudder_motor->rudder_control.rudder_out.Out1;

}


/** 
  * @brief          �������ʿ���
  * @author         XQL  1.0
	*                 LYH  2.0
  * @param[in]      chassis_motor����������ָ��
  * @retval         none
  */	
static void RUDDER_POWER_CONTROL(chassis_move_t *rudder_power)
{
	if(chassis_move.chassis_power_buffer<10)  //��������������ʱ��ȫ�潵�͹���
	{
		rudder_power->power_control.K=3;
		rudder_power->power_control.POWER_MAX=5;
	}
	else
	{
		rudder_power->power_control.K=35;   //35
		rudder_power->power_control.POWER_MAX=40;  //40
	}
	rudder_power->rudder_given_current[0]=rudder_power->Forward_L.given_current;
	rudder_power->rudder_given_current[2]=rudder_power->Back_L.given_current;
	rudder_power->rudder_given_current[3]=rudder_power->Back_R.given_current;
	rudder_power->rudder_given_current[1]=rudder_power->Forward_R.given_current;
	
	int i;
	 for (i = 0; i < 4; i++)
  {
    rudder_power->power_control.current[i]=rudder_power->rudder_given_current[i];
		rudder_power->power_control.totalCurrentTemp+=abs(rudder_power->power_control.current[i]);	
  }
	
	for(i=0;i<4;i++)
	{
	   rudder_power->power_control.MAX_current[i]=(rudder_power->power_control.K*rudder_power->power_control.current[i]/
		              rudder_power->power_control.totalCurrentTemp)
		*(rudder_power->power_control.POWER_MAX)/abs(rudder_power->rudder_speed[i]);	
	}
	 rudder_power->power_control.totalCurrentTemp=0;

	for(i=0;i<4;i++)
	{   
	    rudder_power->rudder_given_current[i]=(int16_t)(rudder_power->rudder_given_current[i]);  		
	 }
	 
}
/** 
  * @brief          �ֵ�����ʿ���
  * @author         XQL  1.0
	*                 LYH  2.0
  * @param[in]      chassis_motor���ֵ������ָ��
  * @retval         none
  */	
void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t *chassis_motor)
{

  chassis_power_move_control(chassis_motor);
	
//			if((chassis_move.key_C==8000)&&get_cap.capvot>16.5f)  //���ݵȼ��仯�������Ĺ���
//		{
//			if(chassis_move.chassis_level==1)
//			{
//			 chassis_move.power_control.K +=30;
//			 chassis_move.power_control.POWER_MAX +=20;
//			}
//			else if(chassis_move.chassis_level==2)
//			{
//			 chassis_move.power_control.K +=40;
//			 chassis_move.power_control.POWER_MAX +=25;
//			}
//			else if(chassis_move.chassis_level==3)
//			{
//			 chassis_move.power_control.K +=50;
//			 chassis_move.power_control.POWER_MAX +=35;
//			}
//			else
//			{
//			 chassis_move.power_control.K +=50;
//			 chassis_move.power_control.POWER_MAX +=30;
//			}
//		}

	if(chassis_move.key_C==8000)
	{
			chassis_move.power_control.K +=50;
			chassis_move.power_control.POWER_MAX +=30;
	}
	
	if(power_charge>12000)  power_charge=12000;
	CAN_CMD_cap(power_charge);
	 int i;

	 for (i = 0; i < 4; i++)
  {
    chassis_motor->power_control.current[i]=chassis_motor->motor_chassis[i].give_current;
		chassis_motor->power_control.totalCurrentTemp+=abs(chassis_motor->power_control.current[i]);	
  }
	
	chassis_motor->POWER_MAX_SET=chassis_motor->power_control.POWER_MAX;    //���p������ʹ�ù��ʼ���������
	                                                                        //ʹ�ù��ʼƺô�:��ϸ�ڵؿ�������  δʹ��ԭ��:δ���ü�ϸ��
	if(chassis_motor->POWER_MAX_SET<0) chassis_motor->POWER_MAX_SET=0;
	for(i=0;i<4;i++)
	{
	   chassis_motor->power_control.MAX_current[i]=(chassis_motor->power_control.K*chassis_motor->power_control.current[i]/chassis_motor->power_control.totalCurrentTemp)
		*(chassis_motor->POWER_MAX_SET)/(abs(chassis_motor->motor_chassis[i].speed));	
	}
	 chassis_motor->power_control.totalCurrentTemp=0;

	for(i=0;i<4;i++)
	{ 					
		if(abs(chassis_motor->motor_chassis[i].give_current)>=abs(chassis_motor->power_control.MAX_current[i]))
		{
			chassis_motor->motor_chassis[i].give_current=chassis_motor->power_control.MAX_current[i];
		}
		else
		{ 
	    chassis_motor->motor_chassis[i].give_current=(int16_t)(chassis_motor->motor_chassis[i].give_current);  		
		}
	}
//}
}


/** 
  * @brief          �ֵ����������������
	* @author         LYH
  * @param[in]      chassis_motor���ֵ������ָ��
  * @retval         none
  */	
static void CHASSIC_MOTOR_PID_CONTROL(chassis_move_t *chassis_motor)
{
	chassis_speed_control_set(chassis_motor);
	int i;


	for(i=0;i<4;i++)
	{   
	  chassis_motor->motor_chassis[i].give_current=PID_Calc(&chassis_motor->motor_chassis[i].chassis_pid,
		chassis_motor->motor_chassis[i].speed,chassis_motor->motor_chassis[i].speed_set);
		if(abs(chassis_motor->power_control.speed[i])<chassis_motor->power_control.SPEED_MIN)
		{
			chassis_motor->power_control.speed[i]=chassis_motor->power_control.SPEED_MIN;				
		}
	}

	CHASSIC_MOTOR_POWER_CONTROL(chassis_motor);
}


/** 
  * @brief          ����PID�γ�ʼ��
  * @author         XQL 1.0
                    LYH 2.0
  * @param[in]      rudder_init����������ָ��
  * @param[in]      PID[8]��PID����
  * @retval         none
  */	
static void RUDDER_PID_INIT(chassis_move_t*rudder_init,const fp32 PID[8])
{
	rudder_init->Forward_L.rudder_control.rudder_in.P_P=rudder_init->Forward_R.rudder_control.rudder_in.P_P=
	rudder_init->Back_L.rudder_control.rudder_in.P_P=rudder_init->Back_R.rudder_control.rudder_in.P_P=PID[0];
	
	rudder_init->Forward_L.rudder_control.rudder_in.P_I=rudder_init->Forward_R.rudder_control.rudder_in.P_I=
	rudder_init->Back_L.rudder_control.rudder_in.P_I=rudder_init->Back_R.rudder_control.rudder_in.P_I=PID[1];
	
	rudder_init->Forward_L.rudder_control.rudder_in.P_D=rudder_init->Forward_R.rudder_control.rudder_in.P_D=
	rudder_init->Back_L.rudder_control.rudder_in.P_D=rudder_init->Back_R.rudder_control.rudder_in.P_D=PID[2];
	
	rudder_init->Forward_L.rudder_control.rudder_in.P_N=rudder_init->Forward_R.rudder_control.rudder_in.P_N=
	rudder_init->Back_L.rudder_control.rudder_in.P_N=rudder_init->Back_R.rudder_control.rudder_in.P_N=PID[3];
	
	rudder_init->Forward_L.rudder_control.rudder_in.S_P=rudder_init->Forward_R.rudder_control.rudder_in.S_P=
	rudder_init->Back_L.rudder_control.rudder_in.S_P=rudder_init->Back_R.rudder_control.rudder_in.S_P=PID[4];
	
	rudder_init->Forward_L.rudder_control.rudder_in.S_I=rudder_init->Forward_R.rudder_control.rudder_in.S_I=
	rudder_init->Back_L.rudder_control.rudder_in.S_I=rudder_init->Back_R.rudder_control.rudder_in.S_I=PID[5];
	
	rudder_init->Forward_L.rudder_control.rudder_in.S_D=rudder_init->Forward_R.rudder_control.rudder_in.S_D=
	rudder_init->Back_L.rudder_control.rudder_in.S_D=rudder_init->Back_R.rudder_control.rudder_in.S_D=PID[6];
	
	rudder_init->Forward_L.rudder_control.rudder_in.S_N=rudder_init->Forward_R.rudder_control.rudder_in.S_N=
	rudder_init->Back_L.rudder_control.rudder_in.S_N=rudder_init->Back_R.rudder_control.rudder_in.S_N=PID[7];
}


/** 
  * @brief          �ٶ�����������
  * @author         LYH
  * @param[in]      *vx_set��x�����ٶ�������
  * @param[in]      *vy_set��y�����ٶ�������
  * @param[in]      chassis_move_rc_to_vector����������ָ��
  * @retval         none
  */	
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
		
			if(chassis_move.key_C==8000)  //���ݵȼ��仯�������Ĺ���
		{
    kx=2.5f;  
    ky=2.0f;
		kw=1.0f;	
		}
		else
		{
		  kx=1.5f;  
			ky=1.5f;
			kw=1.0f;	
		}

    *vy_set += -chassis_move_rc_to_vector->vy_set_CANsend/100;
    *vx_set += -chassis_move_rc_to_vector->vx_set_CANsend/100;
		*vx_set = kx*(*vx_set);
		*vy_set = ky*(*vy_set);

}
/** 
  * @brief          ɲ��ģʽ
  * @author         LYH 1.0
  * @retval         ɲ���ȼ��٣������˽ṹ
  */
int Rear_Brake(chassis_move_t *chassis_move_control)
{	
	if(fabs(chassis_move_control->relative_angle_Dbuf)<0.6f&&fabs(chassis_move.vx)<1.0f&&fabs(chassis_move.vy)<1.0f)
	return 1;
	else
	return 0;
}
/** 
  * @brief          �ֵ����̬���ʿ���
  * @author         LYH 1.0
  * @retval         ���ݵ��ݵ�ѹ�ͻ�������ʵ�ֶ�̬���ʿ���
  */	
void chassis_power_move_control(chassis_move_t *chassis_motor)
{
//			if(chassis_move.chassis_power_buffer<10 &&get_cap.capvot<17 ) //��������������ʱ��ȫ�潵�͹���
//		{
//			 chassis_motor->power_control.K=5;
//			 chassis_motor->power_control.POWER_MAX=20;
//		}
//			else
//		{
//			if(chassis_move.chassis_level==1)
//			{
//			 chassis_motor->power_control.K=113;
//			 chassis_motor->power_control.POWER_MAX=60;
//			 power_charge=6000;
//			}
//			else if(chassis_move.chassis_level==2)
//			{
//			 chassis_motor->power_control.K=116;
//			 chassis_motor->power_control.POWER_MAX=80;
//			 power_charge=8000;
//			}
//			else if(chassis_move.chassis_level==3)
//			{
//			 chassis_motor->power_control.K=122;    //124
//			 chassis_motor->power_control.POWER_MAX=100;
//			 power_charge=10000;
//			}
//			else
//			{
		   chassis_motor->power_control.K=122;
			 chassis_motor->power_control.POWER_MAX=100;
			 power_charge=10000;
//			}
//		}

//		if(get_cap.capvot == 0)    //�޵���ģʽ
//		{
//		if(chassis_move.chassis_power_buffer > 50.0)
//		{
//				chassis_motor->power_control.K+=25;
//				chassis_motor->power_control.POWER_MAX+=15;
//		}
//		if(chassis_move.chassis_power_buffer> 40.0)
//		{
//				chassis_motor->power_control.K+=15;
//				chassis_motor->power_control.POWER_MAX+=10;
//		}
//		else if(chassis_move.chassis_power_buffer > 25.0){
//				chassis_motor->power_control.K+=10;
//				chassis_motor->power_control.POWER_MAX+=5;
//		}
//		else if(chassis_move.chassis_power_buffer> 10.0)
//		{
//				chassis_motor->power_control.K+=1;
//		}
//	}
//		else    //�е���ģʽ 
//		{
//		if(get_cap.capvot>19 )  //����ѹ>17V�ҵ��̻��������㹻 (������̿���orǷѹ)
//		{		
//			if(chassis_move.chassis_power_buffer > 50.0)
//		 {
//				chassis_motor->power_control.K+=6;
//		 		chassis_motor->power_control.POWER_MAX+=5;
//			  power_charge*=2.1f;
//		 }
//		 else if(chassis_move.chassis_power_buffer> 30.0)
//		 {
//				chassis_motor->power_control.K+=4;
//				chassis_motor->power_control.POWER_MAX+=4;
//			  power_charge*=1.2f;
//		 }
//	  	else if(chassis_move.chassis_power_buffer > 15.0){
//		 		chassis_motor->power_control.K+=3;
//				chassis_motor->power_control.POWER_MAX+=3;
//				power_charge*=1.0f;
//		 }
//		 else if(chassis_move.chassis_power_buffer> 10.0)
//		 {
//			 chassis_motor->power_control.K+=1;
//			chassis_motor->power_control.POWER_MAX+=1;
//			 power_charge*=0.9f;
//		 }
//		 else if(chassis_move.chassis_power_buffer>8.0)
//		 {
//			 power_charge*=0.8f;
//		 }
//	}
//	else
//	{

//		if(chassis_move.chassis_power_buffer>50)
//		{
//			power_charge*=2.0f;	
//		}
//		else if(chassis_move.chassis_power_buffer>30)
//		{
//			power_charge*=1.3f;	
//		}
//		else if(chassis_move.chassis_power_buffer>15)
//		{
//			power_charge*=1.1f;	
//		}
//		else if (chassis_move.chassis_power_buffer>8)
//		{
//			power_charge*=1.0f;	
//		}
//		else
//		{
//			power_charge*=0.8f;	
//		}
//	}
//}
}
