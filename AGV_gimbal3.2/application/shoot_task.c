/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      �������.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-21-2022     LYH              1. ���     
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */

#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "stm32.h"
/*----------------------------------�궨��---------------------------*/
#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)//΢������IO
#define trigger_motor(speed)				trigger_control.trigger_speed_set = speed //�����������
/*----------------------------------�ڲ�����---------------------------*/                  
/**
  * @brief   ���״̬��
  */
void Choose_Shoot_Mode(void);
static void Shoot_Set_Mode(void);
static void shoot_level(void);
/**
  * @brief   ������ݸ���
  */
static void Shoot_Feedback_Update(void);
/** @brief   Ħ���ֿ���ѭ��
  */
static void fric_control_loop(fric_move_t *fric_move_control_loop);
static void shoot_fric_off(fric_move_t *fric1_off); 
/**
  * @brief  ������ƣ����Ʋ�������Ƕȣ����һ�η���
  */
static void shoot_bullet_control(void);
/**
  * @brief  �����ʼ��
  */
void shoot_init(void);

/*----------------------------------�ڲ�����---------------------------*/
fp32 limit_V;
fp32 Max_SPEED;
fp32 fric;
int jam_flag=0;
int F_flag=0;  int laster_time=0;
fp32 angle=0; 
int16_t shoot_Current[2]; 
int laster_flag;
uint16_t ShootSpeed;  
fp32 KH=0;
fp32 speed_; 
fp32 speed_t;
int flag=0; 
int time_l=0; 
int flag1=0;
int Ready_Flag=0;  //ȷ���Ƿ������״̬
static uint8_t std_fric = 0; //һ������Ħ����
int trigger_flag=0,trigger_flag1=0; int add_t=0;
fp32 fric_max_control_speed = 2000; //���Ħ���ֿ������ֵ
/*----------------------------------�ṹ��------------------------------*/
static PidTypeDef trigger_motor_pid; 
Shoot_Motor_t trigger_motor;          //����������
fric_move_t fric_move;          //Ħ��������
shoot_mode_e shoot_mode = SHOOT_STOP;   //�˴����ģʽ
shoot_mode_e last_fric_mode= SHOOT_STOP;  //�ϴ����ģʽ
/*----------------------------------�ⲿ����---------------------------*/
extern ExtY_stm32 stm32_Y;
extern ExtU_stm32 stm32_U;
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
/*---------------------------------------------------------------------*/

/**
  * @brief          ������񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void shoot_task(void const *pvParameters)
{  
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
	  shoot_init();
	  while (1)
	  {	
			shoot_laser_on();
			shoot_level(); 	  
			Shoot_Set_Mode();        
			Shoot_Feedback_Update(); 
			shoot_control_loop();
		 if (toe_is_error(DBUS_TOE))
      {
        // ң��������ֹͣ����
        CAN_cmd_shoot(0, 0, 0, 0);
      }
      else           
			CAN_cmd_shoot( fric_move.fric_CAN_Set_Current[0],fric_move.fric_CAN_Set_Current[1],trigger_motor.given_current,0);	
			vTaskDelay(1);		
		}
}
/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
    fric_move.laster_add = 0;
    trigger_motor.move_flag = 1;
    trigger_motor.move_flag_ONE = 1;
    // ��ʼ��PID
    stm32_shoot_pid_init();
	  stm32_Y.out_shoot=0;
		stm32_Y.out_shoot1=0;
    static const fp32 Trigger_speed_pid[3] = {900, 0, 100};
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    const static fp32 motor_speed_pid[3] = {S3505_MOTOR_SPEED_PID_KP, S3505_MOTOR_SPEED_PID_KI, S3505_MOTOR_SPEED_PID_KD};
    PID_Init(&fric_move.motor_speed_pid[0], PID_POSITION, motor_speed_pid, S3505_MOTOR_SPEED_PID_MAX_OUT, S3505_MOTOR_SPEED_PID_MAX_IOUT);
    PID_Init(&fric_move.motor_speed_pid[1], PID_POSITION, motor_speed_pid, S3505_MOTOR_SPEED_PID_MAX_OUT, S3505_MOTOR_SPEED_PID_MAX_IOUT);
    fric_move.motor_speed_pid[0].mode_again = KI_SEPRATE;
    fric_move.motor_speed_pid[1].mode_again = KI_SEPRATE;
    // ����ָ���ȡ
    fric_move.shoot_rc = get_remote_control_point();
    trigger_motor.shoot_motor_measure = get_trigger_motor_measure_point();
    trigger_motor.blocking_angle_set = 0;
    fric_move.motor_fric[0].fric_motor_measure = get_shoot_motor_measure_point(0); // ��Ħ����
    fric_move.motor_fric[1].fric_motor_measure = get_shoot_motor_measure_point(1); // ��Ħ����
    // �˲���ʼ��
    const static fp32 fric_1_order_filter[1] = {0.1666666667f};
    const static fp32 fric_2_order_filter[1] = {0.1666666667f};
    first_order_filter_init(&fric_move.fric1_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_1_order_filter);
    first_order_filter_init(&fric_move.fric2_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_2_order_filter);
    // �ٶ��޷�
    fric_move.max_speed = 4.75f;
    fric_move.min_speed = -4.75f;
    Shoot_Feedback_Update();
    trigger_motor.set_angle = trigger_motor.angle;
}
/**
  * @brief          ����ȼ�����
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_level(void)
{
			if(robot_state.shooter_id1_17mm_speed_limit==15)  
			{ 
				KH=0.4;
				ShootSpeed=15;
//			 fric=1.90f;
				fric=1.90f;
//			if(robot_state.shooter_id1_17mm_cooling_limit<0.5*robot_state.shooter_id1_17mm_cooling_limit)
//			 trigger_motor.speed_set=10.0;				
							 trigger_motor.speed_set=11.0;		
			}
			else if(robot_state.shooter_id1_17mm_speed_limit==18)
			{ 
				KH=0.4;
				ShootSpeed=18;
				fric=2.13f; 
				trigger_motor.speed_set=11.0;
			}			  
			else if(robot_state.shooter_id1_17mm_speed_limit==30)
			{ 
				KH=1.1;
				ShootSpeed=30;
				fric=2.95f; 
				trigger_motor.speed_set=10.0;
			}
			else
			{
				KH=0.32;
				ShootSpeed=15;
//			 fric=1.95f;
				fric=1.90f;
			 trigger_motor.speed_set=10.0;	
			}
}

/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{  
		uint8_t i;
	//������ʱ�����±�־λ�����Ƶ��㵥��
	if(shoot_mode !=SHOOT_STOP&&(fric_move.shoot_rc->rc.ch[4]>50||fric_move.shoot_rc->mouse.press_l))
   {
	  shoot_mode = SHOOT_BULLET;
	  if(last_fric_mode!=SHOOT_BULLET)
	   {
		   last_fric_mode=SHOOT_BULLET;    
	   }
    time_l++;       	   
   }
   else
    {
	    time_l=0,flag1=0;   flag=0; 
    }
   if(time_l>150)
   {
	  flag=0; 
	  flag1=0;
   }
   else if(shoot_mode == SHOOT_BULLET)
   {
	 flag=1;  
   }
   if(flag==1&&flag1==0)
   {
	  trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
    flag1=1;  
   }
    //�˲���������>������
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;	
		//���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
		if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
		{
			trigger_motor.ecd_count--;
		}
		else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
		{
			trigger_motor.ecd_count++;
		}
		if (trigger_motor.ecd_count == FULL_COUNT)
		{
			trigger_motor.ecd_count = -(FULL_COUNT - 1);
		}
		else if (trigger_motor.ecd_count == -FULL_COUNT)
		{
			trigger_motor.ecd_count = FULL_COUNT - 1;
		}
		//���������Ƕ�
		trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
		//Ħ���֡���>�ٶȴ���
    for (i = 0; i < 2; i++)
    {
      fric_move.motor_fric[i].speed = 0.000415809748903494517209f * fric_move.motor_fric[i].fric_motor_measure->speed_rpm;
      fric_move.motor_fric[i].accel = fric_move.motor_speed_pid[i].Dbuf[0] * 500.0f;
    }
	  speed_t= fric_move.motor_fric[0].fric_motor_measure->speed_rpm;
	  speed_= fric_move.motor_fric[1].fric_motor_measure->speed_rpm;
}
/**
	* @brief          ���׼���������Ƿ���Ħ���ַ�������
  * @param[in]      void
  * @retval         ������
  */
static void shoot_ready(void)
{
	static int flag_shoot;	
	if(!flag_shoot && ((fric_move.shoot_rc->key.v&KEY_PRESSED_OFFSET_Z)||(switch_is_down(fric_move.shoot_rc->rc.s[0]))))
	{		if(std_fric == 0)
		{
			Ready_Flag=1;
			std_fric=!std_fric;
		}
		else
		{
			std_fric=!std_fric;
			Ready_Flag=0;
		}
	}
	flag_shoot= ((fric_move.shoot_rc->key.v&KEY_PRESSED_OFFSET_Z)||(switch_is_down(fric_move.shoot_rc->rc.s[0])));
} 
/**
	* @brief          ���ģʽ����
  * @param[in]      void
  * @retval         ������
  */
static void Shoot_Set_Mode(void)
{ 
	shoot_ready();
	//�Ƿ�������
	if(Ready_Flag==1)
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		 shoot_mode = SHOOT_READY;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		shoot_mode = SHOOT_STOP;
			 last_fric_mode = SHOOT_STOP;
	}
	
		if (last_fric_mode != shoot_mode)
   {
        //ģʽ�����л�,pid���
        stm32_step_shoot_pid_clear();
   }
	 last_fric_mode = shoot_mode;
}
/**
  * @brief          ������ѭ��
  * @param[in]      void
  * @retval         ������
  */
void shoot_control_loop(void)
{
    if (shoot_mode == SHOOT_BULLET)
    {                                
       trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
       trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
			if (trigger_flag1==0)
			{
				shoot_bullet_control();
			}
		  fric_control_loop(&fric_move);
		  if(trigger_motor.speed>2)
		  {	
				trigger_flag=1;	
				jam_flag=0;
		  }
      if (trigger_motor.speed<0.1f&&trigger_flag==1)
		  {
				jam_flag++;
				if(jam_flag>20)
				{
					trigger_flag1=1;
					trigger_flag=0;
					jam_flag=0;
				}
		}
    }		
	else if (shoot_mode == SHOOT_READY)
	{                 
		trigger_motor.speed_set = 0;
		fric_control_loop(&fric_move);
		trigger_motor.move_flag =1;
		trigger_motor.move_flag_ONE =1;
	}
	else if(shoot_mode == SHOOT_STOP)	
	{   
    shoot_fric_off(&fric_move);
    trigger_motor.speed_set = 0;
		fric=0.0f;
		fric_control_loop(&fric_move);
    }
	
		if(trigger_flag1==0)
		{		
		 PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
		}
		else if(trigger_flag1==1)
		{
		 PID_Calc(&trigger_motor_pid, trigger_motor.speed, -4.0);	
		 add_t++;
		 if(add_t>190)
			{
			 add_t=0;	
			 trigger_flag1=0;	
			}
		 trigger_motor.set_angle=trigger_motor.angle;
		}
		trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
}
/**
  * @brief   ������ƣ����Ʋ�������Ƕȣ����һ�η���
  */
static void shoot_bullet_control(void)
{  

	if((robot_state.shooter_id1_17mm_cooling_limit-power_heat_data_t.shooter_id1_17mm_cooling_heat>KH*robot_state.shooter_id1_17mm_cooling_rate)||fric_move.shoot_rc->rc.ch[4]>50) 
	{
		 if ( shoot_mode == SHOOT_BULLET&&trigger_motor.move_flag ==1&&flag==0)
		 {
			trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
			trigger_motor.cmd_time = xTaskGetTickCount();
			trigger_motor.move_flag =0;
		 }
	}
		else
	{
	   trigger_motor.speed_set = 0;	
	}
	
	 if (rad_format(trigger_motor.set_angle - trigger_motor.angle)>0.05f)
    {
           
    }
    else
    {  
		if( shoot_mode == SHOOT_BULLET)
		 {
				trigger_motor.move_flag =1;
		 }
    }
}
/**
  * @brief  Ħ���ֹر�
  */
static void shoot_fric_off(fric_move_t *fric_off)
{
    fric_off->speed_set[0]=0.0f;
    fric_off->speed_set[1]=0.0f;
}
/**
  * @brief  Ħ���ֿ���ѭ��
  */
static void fric_control_loop(fric_move_t *fric_move_control_loop)
{
    uint8_t i = 0;
		//PID����޷�
	  for (i = 0; i < 2; i++)
    {
			 fric_move_control_loop->motor_speed_pid[i].max_out = TRIGGER_BULLET_PID_MAX_OUT;
			 fric_move_control_loop->motor_speed_pid[i].max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
	  }
		//�ٶ�����	
	  fric_move_control_loop->speed_set[0] =-fric; 
    fric_move_control_loop->speed_set[1] =fric;   
    for (i = 0; i < 2; i++)
    {
		fric_move_control_loop->motor_fric[i].speed_set = fric_move.speed_set[i];
    PID_Calc(&fric_move_control_loop->motor_speed_pid[i], fric_move_control_loop->motor_fric[i].speed, fric_move_control_loop->motor_fric[i].speed_set);
    }
    stm32_step_shoot_0(fric_move_control_loop->speed_set[0],fric_move_control_loop->motor_fric[0].speed);
	  stm32_step_shoot_1(fric_move_control_loop->speed_set[1],fric_move_control_loop->motor_fric[1].speed);    
		
    if (fric_move.fric_CAN_Set_Current[0] <= -fric_max_control_speed)
    {
        fric_move.fric_CAN_Set_Current[0] = -fric_max_control_speed;
    }
    if (fric_move.fric_CAN_Set_Current[1] >= fric_max_control_speed)
    {
        fric_move.fric_CAN_Set_Current[1] = fric_max_control_speed;
    }		
		
    fric_move.fric_CAN_Set_Current[0]=stm32_Y.out_shoot;
	  fric_move.fric_CAN_Set_Current[1]=stm32_Y.out_shoot1;
}
