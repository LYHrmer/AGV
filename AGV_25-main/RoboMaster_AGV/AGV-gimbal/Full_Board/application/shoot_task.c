/**********************************************************************************************************
 * @�ļ�     shoot_task.c
 * @˵��     ������������
 * @�汾  	 V3.0
 * @����     
 * @����     2024.12
**********************************************************************************************************/
#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "arm_math.h"
#include "user_lib.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "stm32.h"
#include "vision_task.h"
#include "Dji_motor.h"


/***********�ڲ���������***********/

static void shoot_init(Ammo *Shoot_Init);

static void Shoot_Ready(Ammo *Shoot);

static void Shoot_Set_Mode(Ammo *Shoot);

static void Shoot_Feedback_Update(Ammo *shoot);

static void fric_control_loop(Ammo *fric_control);

static void trigger_control_loop(Ammo *trigger_control);

static void shoot_bullet_control(Ammo *shoot);

static void heat_control(Ammo *heat,float dt);


/***********�ڲ�����***********/
static const fp32 Trigger_speed_pid[3] = {900, 0, 100};
static const fp32 Trigger_a_speed_pid[3] = {2024, 0, 200};
static const fp32 angle_pid[3]={5,0,0.0005};
static const fp32 fric_order_filter[1] = {0.1666666667f};

/***********�ṹ��***********/
Ammo Ammo_booster;


/***********�ⲿ����***********/



/****************************************
*�� �� ��: shoot_task
*����˵��: ������񣬼��1ms
*��    ��: ��
*�� �� ֵ: ��
****************************************/
void shoot_task(void const *pvParameters)
{
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	shoot_init(&Ammo_booster);
	while (1)
	{
		Shoot_Feedback_Update(&Ammo_booster);	//���ݸ���
		Shoot_Set_Mode(&Ammo_booster);				//״̬������
		
		fric_control_loop(&Ammo_booster);
		trigger_control_loop(&Ammo_booster);
		if (toe_is_error(DBUS_TOE))
		{
			// ң��������ֹͣ����
			can_cmd_fric(0, 0);
		}
		else
		{
			//Ħ���ַ���
			can_cmd_fric(-Ammo_booster.fric_motor[0].fric.give_cmd_current,Ammo_booster.fric_motor[1].fric.give_cmd_current);
			
			//�����־λ����
		  //can_cmd_shoot_flag((int16_t)Ammo_booster.shoot_flags.fric_ON,(int16_t)Ammo_booster.shoot_flags.fric_ON,(int16_t)Ammo_booster.shoot_flags.trigger_ON,Ammo_booster.trigger_motor.bullet_round);
		}
		vTaskDelay(1);
	}
}

/****************************************
*�� �� ��: shoot_init
*����˵��: �����ʼ��
*��    ��: ��
*�� �� ֵ: ��
****************************************/
void shoot_init(Ammo *Shoot_Init)
{

	//Ħ���ֵ��pid��ʼ��
	stm32_shoot_pid_init();
	// �ٶ��޷�
	Shoot_Init->max_speed = FRIC_SPEED;
	Shoot_Init->min_speed = -FRIC_SPEED;
	//�������pid��ʼ��
	PID_init(&Shoot_Init->trigger_motor.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
	PID_init(&Shoot_Init->trigger_motor.trigger_angle_pid, PID_POSITION, angle_pid,35,15);
	PID_init(&Shoot_Init->trigger_motor.trigger_motor_a_pid, PID_POSITION, Trigger_a_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
	//��ȡң����ָ������
	Shoot_Init->shoot_rc = get_remote_control_point();
	//��ȡ�������ָ������
	for(int i=0;i<2;i++)
	{
		//��ȡĦ���ֵ��ָ������
		M3508_Init(&Shoot_Init->fric_motor[i].fric,i);
		//�˲���ʼ��
		first_order_filter_init(&Shoot_Init->fric_cmd_slow_set_speed[i], SHOOT_CONTROL_TIME, fric_order_filter);
		//Ħ�����ٶ�
		Shoot_Init->fric_motor[i].fric.give_speed_set = FRIC_SPEED;
		Shoot_Init->fric_motor[i].fric.give_speed_set = fp32_constrain(Shoot_Init->fric_motor[i].fric.give_speed_set,Shoot_Init->min_speed,Shoot_Init->max_speed);
	  stm32_Y_fric.out_shoot[i] = 0;

	}
	Shoot_Feedback_Update(Shoot_Init);
	Shoot_Init->trigger_motor.give_rammer_angle = Shoot_Init->trigger_motor.get_rammer_angle;
}

/****************************************
*�� �� ��: Shoot_Feedback_Update
*����˵��: ���ݸ���
*��    ��: ��
*�� �� ֵ: ��
****************************************/
static void Shoot_Feedback_Update(Ammo *Shoot)
{
	int16_t delta_encoder;
	
  M2006_Rx_Date(&Shoot->trigger_motor.trigger);
	//���������
//	Shoot->trigger_motor.bullet_round = (int)(Shoot->trigger_motor.rammer_round * 8.0);
	Shoot->trigger_motor.sumHeat = Shoot->trigger_motor.bullet_round * BulletHeat17;//ÿ����һ��17mm���裬��������10

	//���Ȧ�����ã� ��Ϊ�������תһȦ���������ת36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
	//ͬ��,����תһȦ���ת36Ȧ
	if(Shoot->trigger_motor.trigger.encoder_round == FULL_CNT)
	{
		Shoot->trigger_motor.trigger.encoder_round = -(FULL_CNT - 1);
	}
	else if(Shoot->trigger_motor.trigger.encoder_round == -FULL_CNT)
	{
		Shoot->trigger_motor.trigger.encoder_round = FULL_CNT - 1;
	}
	Shoot->trigger_motor.get_rammer_angle = ((Shoot->trigger_motor.trigger.encoder_round * Shoot_Encoder +Shoot->trigger_motor.trigger.ecd)*Motor_ECD_TO_ANGLE) - PI; 

 // heat_control(Shoot,float dt);
	//Ħ�������ݸ���
	for(int i = 0; i < 2; i++)
	{
		M3508_Rx_Date(&Shoot->fric_motor[i].fric);
	}
	//���
	Shoot->shoot_key.last_press_l = Shoot->shoot_key.press_l;
	Shoot->shoot_key.last_press_r = Shoot->shoot_key.press_r;
	Shoot->shoot_key.press_l = Shoot->shoot_rc->mouse.press_l;
	Shoot->shoot_key.press_l = Shoot->shoot_rc->mouse.press_r;
}
/****************************************
*�� �� ��: Shoot_Ready
*����˵��: ���׼��
*��    ��: ��
*�� �� ֵ: ��
****************************************/
static void Shoot_Ready(Ammo *Shoot)
{
	if(!Shoot->shoot_flags.rc_flag && ((Shoot->shoot_rc->key.v & KEY_PRESSED_OFFSET_R) || (switch_is_down(Shoot->shoot_rc->rc.s[0]))))
	{   
		if(Shoot->shoot_flags.fric_ON == 0)
		{
			//���δ��������׼��״̬�����ҿ���Ħ����
			Shoot->shoot_flags.ready_flag = 1;
			Shoot->shoot_flags.fric_ON = !Shoot->shoot_flags.fric_ON;
		}
		else
		{
			//���������ر�׼��״̬
			Shoot->shoot_flags.ready_flag = 0;
			Shoot->shoot_flags.fric_ON = !Shoot->shoot_flags.fric_ON;
		}
	}
	if(Shoot->shoot_flags.ready_flag == 1)
	{
		Shoot->shoot_mode = SHOOT_READY;
	}
	else
	{
		Shoot->shoot_mode = SHOOT_STOP;
	}
	if(Shoot->last_shoot_mode != Shoot->shoot_mode)
	{
		// ģʽ�����л�,pid���
		stm32_step_shoot_pid_clear();
	}
	Shoot->last_shoot_mode = Shoot->shoot_mode;
	Shoot->shoot_flags.rc_flag = ((Shoot->shoot_rc->key.v & KEY_PRESSED_OFFSET_R)||(switch_is_down(Shoot->shoot_rc->rc.s[0])));

}
/****************************************
*�� �� ��: Shoot_Set_Mode
*����˵��: ���ģʽ����
*��    ��: ��
*�� �� ֵ: ��
****************************************/
static float mouse_count=0.0f;
static void Shoot_Set_Mode(Ammo *Shoot)
{
	Shoot_Ready(Shoot);
/*	if(!Shoot->shoot_key.last_G&&Shoot->shoot_rc->key.v & KEY_PRESSED_OFFSET_G)
	{
		Shoot->shoot_key.G=!Shoot->shoot_key.G;
	}
	Shoot->shoot_key.last_G = Shoot->shoot_rc->key.v & KEY_PRESSED_OFFSET_G;
	if(!Shoot->shoot_key.G)
	{
		if(Shoot->shoot_rc->rc.ch[4] < -600 && switch_is_mid(Shoot->shoot_rc->rc.s[1]))
		{
			servo_pwm_set(1000,1);
			Shoot->shoot_key.rc_key_flag = 1;
		}
		else if(Shoot->shoot_rc->rc.ch[4] > 600 && switch_is_mid(Shoot->shoot_rc->rc.s[1]))
		{
			servo_pwm_set(2300,1);
		Shoot->shoot_key.rc_key_flag = 1;
		}
		if(Shoot->shoot_key.rc_key_flag == 0 && switch_is_down(Shoot->shoot_rc->rc.s[1]))
		{
			servo_pwm_set(2300,1);
		}
	}
	else if(Shoot->shoot_key.G)
	{
		servo_pwm_set(1000,1);
	}
*/
	if((/*Shoot->shoot_flags.shootAble &&*/ Shoot->shoot_mode == SHOOT_READY ) || Shoot->shoot_rc->mouse.press_l )//|| gimbal_behaviour == GIMBAL_AUTO_ATTACK )
	{
		if(Shoot->shoot_rc->rc.ch[4]>=400)
		{
			Shoot->shoot_mode = SHOOT_BULLET;//����
			if(Shoot->last_shoot_mode != SHOOT_BULLET)
				Shoot->last_shoot_mode = SHOOT_BULLET;
		}
		else if(Shoot->shoot_rc->rc.ch[4]<-400 )//|| mouse_count<10)//	
		{
			Shoot->shoot_mode = SHOOT_ONE;//����
		}
			else if(!Shoot->shoot_flags.trigger_toto && (Shoot->shoot_rc->key.v & KEY_PRESSED_OFFSET_Q)&&Shoot->shoot_rc->mouse.press_l)
		{
			Shoot->shoot_mode = SHOOT_TOTOTO;//��ģʽ
			Shoot->shoot_flags.trigger_toto = 1;
		}
		else
		{
			Shoot->shoot_mode = SHOOT_STOP;
		}
	}

	else if(Shoot->shoot_rc->rc.ch[4]<-600 || Shoot->shoot_rc->mouse.press_r || (Shoot->trigger_motor.trigger.get_torque > 10 && Shoot->trigger_motor.trigger.get_speed < 5))
	{
		Shoot->shoot_mode  = SHOOT_STUCK;//�����ز�
	}
	else
	{
		Shoot->shoot_mode = SHOOT_STOP;
	}
}
/****************************************
*�� �� ��: trigger_speed_change
*����˵��: ����������ģʽ��ת��ѡ��
*��    ��: CoolBuff�������־
*�� �� ֵ: ��
****************************************/
static void trigger_speed_change(Trigger_Motor_t *trigger_speed,bool_t CoolBuff)
{
	switch(pos_date.robot_level)
	{
		case 1:
		{
			if(CoolBuff)
				trigger_speed->trigger.give_speed_set = LOW_SPEED + 5.0f;//������
			else
				trigger_speed->trigger.give_speed_set = LOW_SPEED;	//������
			break;
		}
		case 2:
		{
			if(CoolBuff)
				trigger_speed->trigger.give_speed_set = MID_SPEED+5.0f;//������
			else
				trigger_speed->trigger.give_speed_set = MID_SPEED;	//������
			break;
		}
		case 3:
		{
			if(CoolBuff)
				trigger_speed->trigger.give_speed_set = HIGH_SPEED + 5.0f;//������
			else
				trigger_speed->trigger.give_speed_set = HIGH_SPEED;	//������
			break;
		}
		default:
		{
			if(CoolBuff)
				trigger_speed->trigger.give_speed_set = HIGHER_SPEED + 5.0f;//������
			else
				trigger_speed->trigger.give_speed_set = HIGHER_SPEED;	//������
			break;
		}
	}
}
/****************************************
*�� �� ��: heat_control
*����˵��: ��������
*��    ��: ������ʱ�䣬��λs
*�� �� ֵ: ��
****************************************/
float HeatControlThreshold = 0.95f;   	//�����������Ƶ���ֵ
float curHeat=0.0f;
static void heat_control(Ammo *Heat,float dt)
{
	Heat->HeatLimit17 = shoot_date.shoot_17mm_heat_limit - BulletHeat17;//��������
	Heat->HeatCool17 = shoot_date.shoot_17mm_cooling_value *dt;//��ȴ����
/****************************************���µ�ǰ����****************************************/
	if(Heat->heat_flags.HeatUpDate == 1)
	{
		Heat->heat_flags.HeatUpDate = 0;
		Heat->CurHeat17 = shoot_date.shoot_17mm_heat; 
	}
	else
	{
		Heat->CurHeat17 += Heat->trigger_motor.bullet_round * BulletHeat17;//���й�������
		Heat->trigger_motor.bullet_round = 0;
	//	Heat->CurHeat17 -= robot_state.shooter_barrel_cooling_value;
		if(Heat->CurHeat17 < 0) Heat->CurHeat17 = 0;
	}
	
	if(Heat->CurHeat17 < HeatControlThreshold * Heat->HeatLimit17)
		Heat->shoot_flags.shootAble = 1;
	else
		Heat->shoot_flags.shootAble = 0;
	Heat->lastHeat17 = Heat->CurHeat17;
	
}

/**
 * @brief          ������ѭ��
 * @param[in]      void
 * @retval         ������
 */
static void trigger_control_loop(Ammo *trigger_control)
{
	if(trigger_control->shoot_mode == SHOOT_READY)
	{	
		trigger_control->trigger_motor.trigger.give_speed_set=0.0f;
		trigger_control->last_shoot_mode = SHOOT_READY;
		trigger_control->shoot_flags.trigger_ON = 1;
	}
	else if(trigger_control->shoot_mode == SHOOT_STOP)
	{
		trigger_control->trigger_motor.trigger.give_speed_set = 0.0f;
		trigger_control->trigger_motor.trigger.give_angle_set =0.0f;
		
		PID_clear(&trigger_control->trigger_motor.trigger_motor_pid);
		PID_clear(&trigger_control->trigger_motor.trigger_angle_pid);
		PID_clear(&trigger_control->trigger_motor.trigger_motor_a_pid);
		trigger_control->trigger_motor.trigger.give_cmd_current = 0;
	}
	else if(trigger_control->shoot_mode == SHOOT_STUCK)
	{
			trigger_control->trigger_motor.give_rammer_angle -= PI_Ten;
//				else
//					trigger_control->trigger_motor.give_rammer_angle -=PI_Four;//�Ե����ͻز���������
				
				if(trigger_control->trigger_motor.give_rammer_angle > PI)
					trigger_control->trigger_motor.give_rammer_angle -= PI;
				else if(trigger_control->trigger_motor.give_rammer_angle < -PI)
					trigger_control->trigger_motor.give_rammer_angle += PI;
				
				PID_calc(&trigger_control->trigger_motor.trigger_angle_pid,trigger_control->trigger_motor.get_rammer_angle,trigger_control->trigger_motor.give_rammer_angle);
				trigger_control->trigger_motor.trigger.give_speed_set = trigger_control->trigger_motor.trigger_angle_pid.out;
	}
	else
	{

//		if(trigger_control->shoot_flags.shootAble)//�������Ʒ���
//		{
			if(trigger_control->shoot_mode == SHOOT_BULLET)
			{
				trigger_speed_change(&trigger_control->trigger_motor,trigger_control->shoot_flags.cool_buff);
				trigger_control->last_shoot_mode=SHOOT_BULLET;
			}
			else if(trigger_control->shoot_mode == SHOOT_TOTOTO)
			{
				if(trigger_control->shoot_flags.cool_buff)
					trigger_control->trigger_motor.trigger.give_speed_set = 35.0f;
				else
					trigger_control->trigger_motor.trigger.give_speed_set =	30.0f;
				trigger_control->last_shoot_mode = SHOOT_TOTOTO;
			}
			else if(trigger_control->shoot_mode == SHOOT_ONE)
			{
				
					trigger_control->trigger_motor.give_rammer_angle += PI_Four;
//				else
//					trigger_control->trigger_motor.give_rammer_angle -=PI_Four;//�Ե����ͻز���������
				
				if(trigger_control->trigger_motor.give_rammer_angle > PI)
					trigger_control->trigger_motor.give_rammer_angle -= PI;
				else if(trigger_control->trigger_motor.give_rammer_angle < -PI)
					trigger_control->trigger_motor.give_rammer_angle += PI;
				
				PID_calc(&trigger_control->trigger_motor.trigger_angle_pid,trigger_control->trigger_motor.get_rammer_angle,trigger_control->trigger_motor.give_rammer_angle);
				trigger_control->trigger_motor.trigger.give_speed_set = trigger_control->trigger_motor.trigger_angle_pid.out;
				trigger_control->last_shoot_mode = SHOOT_ONE;
			}
		
	//	}
		else
		{
			PID_clear(&trigger_control->trigger_motor.trigger_motor_pid);
		}
		PID_calc(&trigger_control->trigger_motor.trigger_motor_pid,trigger_control->trigger_motor.trigger.get_speed,trigger_control->trigger_motor.trigger.give_speed_set);
		trigger_control->trigger_motor.trigger.give_cmd_current=(int16_t)(trigger_control->trigger_motor.trigger_motor_pid.out);
	}		
		
}
/**********************************************************************************************************
*�� �� ��: fric_speed_change
*����˵��: ����Ħ����ת��ѡ��
*��    ��: void
*�� �� ֵ: ��
**********************************************************************************************************/
static void fric_speed_change(Fric_Motor_t *fric_speed)
{
	switch(pos_date.robot_level)
	{
		case 1:
		{
			fric_speed->fric.give_speed_set = LOW_FRIC_SPEED;
			break;
		}
		case 2:
		{
			fric_speed->fric.give_speed_set = MID_FRIC_SPEED;
			break;
		}
		case 3:
		{
			fric_speed->fric.give_speed_set = HIGH_FRIC_SPEED;
			break;
		}
		default:
		{
			fric_speed->fric.give_speed_set = HIGHER_FRIC_SPEED;
			break;
		}
	}
	Math_Constrain(&fric_speed->fric.give_speed_set,FRIC_MIN_SPEED,FRIC_MAX_SPEED);
}
/**
  * @brief          Ħ���ֵ��ѭ��
  * @param[in]      void
  * @retval         void
  */
static void fric_control_loop(Ammo *fric_control)
{
	for(int i=0;i<2;i++)
	{
		if(fric_control->shoot_flags.fric_ON)
		{
			//Ħ�����ٶȾ���
			fric_speed_change(&fric_control->fric_motor[i]);
			//Ħ�����ٶ�pid����
			stm32_step_shoot(fric_control->fric_motor[i].fric.give_speed_set,fric_control->fric_motor[i].fric.get_speed,i);
			fric_control->fric_motor[i].fric.give_cmd_current = (int16_t)stm32_Y_fric.out_shoot[i];
		}
		else
		{
			stm32_step_shoot_pid_clear();
			fric_control->fric_motor[i].fric.give_cmd_current = 0;
		}
		fric_control->fric_motor[i].fric.give_cmd_current = int16_constrain(fric_control->fric_motor[i].fric.give_cmd_current,-8000,8000);
	}
}

/**
 * @brief   ������ƣ����Ʋ�������Ƕȣ����һ�η���
 */

static void shoot_bullet_control(Ammo *Shoot)
{
	vision_shoot_judge(&Shoot->vision_control, Shoot->vision_control.gimbal_vision_control.gimbal_yaw - Shoot->vision_control.imu_absolution_angle.yaw,Shoot->vision_control.gimbal_vision_control.gimbal_pitch - Shoot->vision_control.imu_absolution_angle.pitch, sqrt(pow(Shoot->vision_control.target_data.x, 2) + pow(Shoot->vision_control.target_data.y, 2)));
	
	if(Shoot->shoot_rc->key.v & KEY_PRESSED_OFFSET_Q)
	{
		if (shoot_date.shoot_17mm_heat_limit - shoot_date.shoot_17mm_cooling_value > 60 && Shoot->shoot_rc->mouse.press_l ) //&& (fabs(vision_control.gimbal_vision_control.gimbal_pitch - vision_control.imu_absolution_angle.pitch) <= ALLOW_ATTACK_ERROR && fabs(vision_control.gimbal_vision_control.gimbal_yaw - vision_control.imu_absolution_angle.yaw) <= ALLOW_ATTACK_ERROR))
		{
			//�Զ����
			if (gimbal_behaviour == GIMBAL_AUTO_ATTACK && Shoot->vision_control.shoot_vision_control.shoot_command == SHOOT_ATTACK)
			{
				Shoot->trigger_motor.give_rammer_angle += PI_Two;
				 if(Shoot->trigger_motor.give_rammer_angle > PI)
			  	Shoot->trigger_motor.give_rammer_angle += PI;
			  else if(Shoot->trigger_motor.trigger.give_angle_set < -PI)
			  	Shoot->trigger_motor.trigger.give_angle_set -= PI;
			}
			else if(Shoot->shoot_mode == SHOOT_BULLET && Shoot->move_flag == 1)
			{
				//�ƶ�and���
				
			}
		}
	}
	else if((shoot_date.shoot_17mm_heat_limit - shoot_date.shoot_17mm_cooling_value > 30 || Shoot->shoot_rc->key.v & KEY_PRESSED_OFFSET_Q) && (Shoot->shoot_rc->rc.ch[4] > 50||gimbal_behaviour == GIMBAL_AUTO_ATTACK) && (Shoot->vision_control.shoot_vision_control.shoot_command == SHOOT_ATTACK|| Shoot->shoot_rc->mouse.press_l )) //&& (fabs(vision_control.gimbal_vision_control.gimbal_pitch - vision_control.imu_absolution_angle.pitch) <= ALLOW_ATTACK_ERROR && fabs(vision_control.gimbal_vision_control.gimbal_yaw - vision_control.imu_absolution_angle.yaw) <= ALLOW_ATTACK_ERROR))
	{
		//�Զ����
		if (gimbal_behaviour == GIMBAL_AUTO_ATTACK && Shoot->vision_control.shoot_vision_control.shoot_command == SHOOT_ATTACK)
		{
			Shoot->trigger_motor.give_rammer_angle += PI_Four;
			 if(Shoot->trigger_motor.give_rammer_angle > PI)
			  	Shoot->trigger_motor.give_rammer_angle += PI;
			  else if(Shoot->trigger_motor.trigger.give_angle_set < -PI)
			  	Shoot->trigger_motor.trigger.give_angle_set -= PI;
		}
		else if (Shoot->shoot_mode == SHOOT_BULLET)
		{
			Shoot->trigger_motor.give_rammer_angle += PI_Four;
			 if(Shoot->trigger_motor.give_rammer_angle > PI)
			  	Shoot->trigger_motor.give_rammer_angle += PI;
			 else if(Shoot->trigger_motor.trigger.give_angle_set < -PI)
			  	Shoot->trigger_motor.trigger.give_angle_set -= PI;
		}
	}
}

/**
  * @brief          �������
  * @param[in]      void
  * @retval         void
  */
//static void pills_cover(Ammo *Shoot)
//{
//	static int count = 0;
//	if (count == 0)
//	{
//		servo_pwm_set(1700, 1);
//		count++;
//	}
//	//ң��������
//	//��
//	if (Shoot->shoot_rc->rc.ch[4] > 500 )
//	servo_pwm_set(1000, 1);
//  //servo_pwm_set(1700, 1);
//	//��
//	else if(Shoot->shoot_rc->rc.ch[4] < -500 )
//	servo_pwm_set(1700, 1);
//  //servo_pwm_set(1000, 1);
//	else
//	{

//	}
//}
