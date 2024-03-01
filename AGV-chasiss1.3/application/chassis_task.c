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
#include "detect_task.h"
extern cap_measure_t get_cap;

#define pi 3.1415926
#define half_pi 1.5707963

/**
 * @brief          死区限制
 * @author         XQL
 * @param[in]      input：  输入值
 * @param[in]      output： 输出值
 * @param[in]      dealine：死区值
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
 * @brief         取绝对值
 * @author        XQL
 * @param[in]     输入
 * @retval        none
 */
#define abs(x) ((x) > 0 ? (x) : (-x))

/**
 * @brief          电机编码值规整 0—8191
 * @author         XQL
 * @param[in]      电机编码值
 * @retval         none
 */
#define ECD_Format(ecd)         \
	{                           \
		if ((ecd) > ecd_range)  \
			(ecd) -= ecd_range; \
		else if ((ecd) < 0)     \
			(ecd) += ecd_range; \
	}

/**
 * @brief 	        编码转弧度
 * @author         XQL
 * @param[in]      angle：编码值
 * @retval         none
 */
#define rand(angle) angle = angle / 8191 * 3.1415926f * 2

/**
 * @brief 	        取较小值
 * @author         XQL
 * @param[in]      angle1：输入角度1
 * @param[in]      angle2：输入角度2
 * @param[in]      ture：  输出角度
 * @retval         none
 */
#define compare(angle1, angle2, ture) \
	{                                 \
		if (angle1 > angle2)          \
		{                             \
			ture = angle2;            \
		}                             \
		else                          \
		{                             \
			ture = angle1;            \
		}                             \
	}

/**
 * @brief 	        角度转弧度
 * @author         XQL
 * @param[in]      angle：输入角度
 * @retval         none
 */
#define rad(angle) angle = angle / 180 * 3.1415926f

// 底盘控制所有相关数据
chassis_move_t chassis_move;
fp32 kx = 1.f, ky = 1.f, kw = 1.f; // 速度转换的几个系数
int linkState_2 = 0;
int linkState_1 = 0; // 判断双板通信是否正常
/**
 * @brief          返回舵电机 6020电机数据指针
 * @author         XQL
 * @param[in]      none
 * @retval         电机数据指针
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

// 舵电机PID初始化
static void RUDDER_PID_INIT(chassis_move_t *rudder_init, const fp32 PID[8]);
// 轮电机速度设置
static void chassis_speed_control_set(chassis_move_t *chassis_speed_set);
// 底盘初始化
static void chassis_init(chassis_move_t *chassis_move_init);
// 底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
// 底盘模式设置
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
// 底盘切换模式状态保存
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
// 底盘控制量设置
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
// 轮，舵角度，速度初步解算
static void chassic_rudder_preliminary_A_S_solution(chassis_move_t *chassic_rudder_preliminary_solution);
// 舵控制输入
static void rudder_control_loop(chassis_move_t *rudder_move_control_loop);
// 轮电机控制输出
static void CHASSIC_MOTOR_PID_CONTROL(chassis_move_t *chassis_motor);
// 舵电机控制输出
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t *rudder_motor);
// 舵电机输出角
static void Rudder_motor_relative_angle_control(Rudder_Motor_t *chassis_motor);
// 遥控器、键盘输入量设置
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
// 舵电机零点过边界处理
static void Rudder_motor_ecd_judge(Rudder_Motor_t *Rudder_ecd_judge);
// 舵电机跟随云台零点处理
static void Rudder_motor_zero_ecd_set(chassis_move_t *Rudder_ecd_set);
// 舵角度最优解控制量设置
static void rudder_optimal_angle_solution(Rudder_Motor_t *rudder_optimal_angle);
// 舵电机功率控制
static void RUDDER_POWER_CONTROL(chassis_move_t *rudder_power);
// 轮电机动态功率控制
void chassis_power_move_control(chassis_move_t *chassis_motor);
// 超级电容充电
void Power_Charge(fp32 power);
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
/**
 * @brief          底盘任务
 * @author         XQL 1.0
 *                 LYH 2.0
 * @param[in]      chassis_move_init：底盘数据指针
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
	chassis_init(&chassis_move);
	while (1)
	{
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_contorl(&chassis_move);
		rudder_control_loop(&chassis_move);
		RUDDER_POWER_CONTROL(&chassis_move);
		CHASSIC_MOTOR_PID_CONTROL(&chassis_move);

		linkState_1++;		linkState_2++;
		
		if (toe_is_error(0)) // 若双板通信没接收到数据，则舵和轮都不动
		{
			CAN_cmd_chassis(0, 0, 0, 0);
			CAN_cmd_rudder(0, 0, 0, 0);
		}
		else
		{
			CAN_cmd_rudder(chassis_move.rudder_given_current[0],chassis_move.rudder_given_current[1],chassis_move.rudder_given_current[2],chassis_move.rudder_given_current[3]);

			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current,chassis_move.motor_chassis[1].give_current,chassis_move.motor_chassis[2].give_current,chassis_move.motor_chassis[3].give_current);
		}
		vTaskDelay(2); // 若为1可能会出现疯车，因为通信频率
	}
}

fp32 motor_speed_pid[3] = {CHASSIS_KP, CHASSIS_KI, CHASSIS_KD};
fp32 power_buffer_pid[3] = {M3505_MOTOR_POWER_PID_KP, M3505_MOTOR_POWER_PID_KI, M3505_MOTOR_POWER_PID_KD }; //功率环PID参数
const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
fp32 rudder_pid[8] = {RUDDER_P_P, RUDDER_P_I, RUDDER_P_D, RUDDER_P_N, RUDDER_S_P, RUDDER_S_I, RUDDER_S_D, RUDDER_S_N};

/**
 * @brief          初始化底盘数据
 * @author         XQL 1.0
 *                 LYH 2.0
 * @param[in]      chassis_move_init：底盘数据指针
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{

	if (chassis_move_init == NULL)
	{
		return;
	}
	chassis_move_init->chassis_rc_ctrl = get_remote_control_point();

	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

	chassis_move_init->chassis_rc_ctrl = get_remote_control_point();
	// 舵电机数据指针获取
	chassis_move_init->Forward_L.gimbal_motor_measure = get_Forward_L_motor_measure_point();
	chassis_move_init->Forward_R.gimbal_motor_measure = get_Forward_R_motor_measure_point();
	chassis_move_init->Back_R.gimbal_motor_measure = get_Back_R_motor_measure_point();
	chassis_move_init->Back_L.gimbal_motor_measure = get_Back_L_motor_measure_point();

	chassis_move_init->chassis_motor_mode = CHASSIS_VECTOR_RAW;

	// 轮电机数据指针获取，PID初始化
	int i;
	for (i = 0; i < 4; i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_Init(&chassis_move_init->motor_chassis[i].chassis_pid, PID_POSITION, motor_speed_pid, CHASSIS_MAX_OUT, CHASSIS_MAX_IOUT);
	}

	// 舵电机PID初始化
	RUDDER_PID_INIT(chassis_move_init, rudder_pid);
	// 跟随云台PID
	PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	//功率环PID
	PID_Init(&chassis_move_init->buffer_pid, PID_POSITION, power_buffer_pid, M3505_MOTOR_POWER_PID_MAX_OUT, M3505_MOTOR_POWER_PID_MAX_IOUT);
	// 遥控器数据指针获取
	chassis_move_init->chassis_rc_ctrl = get_remote_control_point();
	// 一阶低通滤波初始化
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
	// 斜坡函数初始化
	 ramp_init(&chassis_move_init->vx_ramp, 0.01f, 20, -20);
   ramp_init(&chassis_move_init->vy_ramp, 0.01f, 10, -10);
	// 轮电机转动方向初始化
	chassis_move_init->Forward_L.Judge_Speed_Direction = chassis_move_init->Forward_R.Judge_Speed_Direction =
		chassis_move_init->Back_L.Judge_Speed_Direction = chassis_move_init->Back_R.Judge_Speed_Direction = 1.0f;
	// 底盘数据初始化
	chassis_move_init->chassis_relative_last = 0.0f;
	chassis_feedback_update(chassis_move_init);
	// 舵电机编码值初始化
	chassis_move.Forward_L.ecd_zero_set = Forward_L_ecd;
	chassis_move.Forward_R.ecd_zero_set = Forward_R_ecd;
	chassis_move.Back_R.ecd_zero_set = Back_R_ecd;
	chassis_move.Back_L.ecd_zero_set = Back_L_ecd;
	chassis_move.power_control.SPEED_MIN = 0.1f;
}

/**
 * @brief          更新底盘数据
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      chassis_move_update：底盘数据指针
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

	chassis_move_update->rudder_speed[0] = chassis_move_update->Forward_L.gimbal_motor_measure->speed_rpm * GM6020_RPM_TO_VECTOR;
	chassis_move_update->rudder_speed[1] = chassis_move_update->Back_L.gimbal_motor_measure->speed_rpm * GM6020_RPM_TO_VECTOR;
	chassis_move_update->rudder_speed[2] = chassis_move_update->Back_R.gimbal_motor_measure->speed_rpm * GM6020_RPM_TO_VECTOR;
	chassis_move_update->rudder_speed[3] = chassis_move_update->Forward_R.gimbal_motor_measure->speed_rpm * GM6020_RPM_TO_VECTOR;

	// 云台的相对角度
	chassis_move.gimbal_data.relative_angle = ((fp32)(chassis_move.gimbal_data.relative_angle_receive)) * Motor_Ecd_to_Rad;
}

/**
 * @brief          底盘控制模式设置
 * @author         XQL
 * @param[in]      chassis_move_mode：底盘数据指针
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
 * @brief          底盘切换模式数据缓存
 * @author         XQL 1.0
 *                 LYH 2.0
 * @param[in]      chassis_move_transit：底盘数据指针
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if (chassis_move_transit == NULL || chassis_move_transit->last_chassis_motor_mode == chassis_move_transit->chassis_motor_mode)
	{
		return;
	}

	if ((chassis_move_transit->last_chassis_motor_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && // 切换到底盘跟随云台
		chassis_move_transit->chassis_motor_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		// 更新舵电机零点
		chassis_move_transit->Forward_L.ecd_zero_set = Forward_L_ecd;
		chassis_move_transit->Forward_R.ecd_zero_set = Forward_R_ecd;
		chassis_move_transit->Back_R.ecd_zero_set = Back_R_ecd;
		chassis_move_transit->Back_L.ecd_zero_set = Back_L_ecd;
		// 更正轮电机转向
		chassis_move_transit->Forward_L.Judge_Speed_Direction = chassis_move_transit->Forward_R.Judge_Speed_Direction =
			chassis_move_transit->Back_L.Judge_Speed_Direction = chassis_move_transit->Back_R.Judge_Speed_Direction = 1.0f;
	}
	else if ((chassis_move_transit->last_chassis_motor_mode != CHASSIS_VECTOR_SPIN) && // 切换到小陀螺模式
			 chassis_move_transit->chassis_motor_mode == CHASSIS_VECTOR_SPIN)  
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		// 更新舵电机零点
		chassis_move_transit->Forward_L.ecd_zero_set = Forward_L_ecd;
		chassis_move_transit->Forward_R.ecd_zero_set = Forward_R_ecd;
		chassis_move_transit->Back_R.ecd_zero_set = Back_R_ecd;
		chassis_move_transit->Back_L.ecd_zero_set = Back_L_ecd;
		// 更正轮电机转向
		chassis_move_transit->Forward_L.Judge_Speed_Direction = chassis_move_transit->Forward_R.Judge_Speed_Direction =
			chassis_move_transit->Back_L.Judge_Speed_Direction = chassis_move_transit->Back_R.Judge_Speed_Direction = 1.0f;
	}
	else if ((chassis_move_transit->last_chassis_motor_mode != RUDDER_VECTOR_FOLLOW_GIMBAL_YAW) && // 切换到舵跟随云台模式
			 chassis_move_transit->chassis_motor_mode == RUDDER_VECTOR_FOLLOW_GIMBAL_YAW)
	{
		chassis_move_transit->chassis_relative_angle_set = 0.0f;
		// 速度值清零
		chassis_move_transit->wz_set = 0.0f;
		chassis_move_transit->vx_set = 0.0f;
		chassis_move_transit->vy_set = 0.0f;
		// 更正轮电机转向
		chassis_move_transit->Forward_L.Judge_Speed_Direction = chassis_move_transit->Forward_R.Judge_Speed_Direction =
			chassis_move_transit->Back_L.Judge_Speed_Direction = chassis_move_transit->Back_R.Judge_Speed_Direction = -1.0f;
	}
	chassis_move_transit->last_chassis_motor_mode = chassis_move_transit->chassis_motor_mode;
}

/**
 * @brief          底盘输入控制量设置及转换
 * @author         LYH 1.0
 * @param[in]      chassis_move_control：底盘数据指针
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	if (chassis_move_control == NULL)
	{
		return;
	}
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	fp32 relative_angle = 0.0f;
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

	if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) // 底盘跟随云台模式
	{
		fp32 sin_yaw, cos_yaw = 0.0f;
		fp32 relative_angle = 0.0f;
		chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
		relative_angle = chassis_move_control->gimbal_data.relative_angle;
		if (relative_angle > PI)
			relative_angle = -2 * PI + relative_angle;

		sin_yaw = arm_sin_f32((relative_angle));
		cos_yaw = arm_cos_f32((relative_angle));

		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
		chassis_move_control->vy_set = -1.0f * sin_yaw * vx_set + cos_yaw * vy_set;

		if (fabs(relative_angle) < 0.10 && (chassis_move_control->vx_set_CANsend == 0.0f && chassis_move_control->vy_set_CANsend == 0.0f)) // 一般情况下不抱圆，起步速度更快
		{
			chassis_move_control->wz_set = 0;
		}
		else
		{
			chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, relative_angle, chassis_move_control->chassis_relative_angle_set);
		}
	}
	
	
	
	else if (chassis_move_control->chassis_motor_mode == RUDDER_VECTOR_FOLLOW_GIMBAL_YAW) // 舵跟随云台模式
	{

		fp32 sin_yaw, cos_yaw = 0.0f;
		fp32 relative_angle = 0.0f;
		chassis_move_control->chassis_relative_angle_set = rad_format(0.0f);
		relative_angle = chassis_move_control->gimbal_data.relative_angle;
		if (relative_angle > PI)
			relative_angle = -2 * PI + relative_angle;

		sin_yaw = arm_sin_f32((relative_angle));
		cos_yaw = arm_cos_f32((relative_angle));

		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
		chassis_move_control->vy_set = -1.0f * sin_yaw * vx_set + cos_yaw * vy_set;

		chassis_move_control->wz_set = 0;
	}
	else if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_SPIN) // 小陀螺模式
	{
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		relative_angle = chassis_move_control->gimbal_data.relative_angle;

		if (relative_angle > PI)
			relative_angle = -2 * PI + relative_angle;
		else if (relative_angle < -PI)
			relative_angle = 2 * PI + relative_angle;
		else
			relative_angle = relative_angle;

		sin_yaw = arm_sin_f32((relative_angle));
		cos_yaw = arm_cos_f32((relative_angle));

		chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
		chassis_move_control->vy_set = -1.0f * sin_yaw * vx_set + cos_yaw * vy_set;
		chassis_move_control->chassis_relative_angle_set = rad_format(0.0);
		
		chassis_move_control->wz_set = 5.0f;

		if (fabs(chassis_move_control->vx_set_CANsend) > 50 || fabs(chassis_move_control->vy_set_CANsend) > 50)
		{
			chassis_move_control->wz_set *= 0.4f;
			chassis_move_control->vx_set *= 1.4f;
			chassis_move_control->vy_set *= 1.4f;
		}
		else // 当原地时，加大转速
		{
			chassis_move_control->wz_set = 15.0f;
		}
	}
	else if (chassis_move_control->chassis_motor_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
	{
		chassis_move_control->vx_set = vx_set;
		chassis_move_control->vy_set = vy_set;
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
 * @brief          舵角度最优解控制量设置
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      rudder_optimal_angle：舵电机数据指针
 * @retval         none
 */
static void rudder_optimal_angle_solution(Rudder_Motor_t *rudder_optimal_angle)
{
	rudder_optimal_angle->ecd_temp_error = rudder_optimal_angle->ecd_add - rudder_optimal_angle->last_ecd_add;

	
	if (rudder_optimal_angle->ecd_temp_error > 2048 )
	{
		rudder_optimal_angle->ecd_add -= 4096;
		rudder_optimal_angle->Judge_Speed_Direction = -1;
	}
	else if (rudder_optimal_angle->ecd_temp_error < -2048 )
	{
		rudder_optimal_angle->ecd_add += 4096;
		rudder_optimal_angle->Judge_Speed_Direction = -1;
	}
	else
	{
		rudder_optimal_angle->Judge_Speed_Direction = 1;
	}
}

/**
 * @brief          轮，舵初步角度速度控制量解算
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      chassic_rudder_preliminary_solution：底盘数据指针
 * @retval         进行了略微的重构
 */
static void chassic_rudder_preliminary_A_S_solution(chassis_move_t *chassic_rudder_preliminary_solution)
{
	fp32 vx_set, vy_set, vw_set = 0.0f;
	vx_set = chassic_rudder_preliminary_solution->vx_set;
	vy_set = chassic_rudder_preliminary_solution->vy_set;
	vw_set = chassic_rudder_preliminary_solution->wz_set;
	// 更新舵电机角度
	chassic_rudder_preliminary_solution->Forward_L.last_rudder_angle = chassic_rudder_preliminary_solution->Forward_L.rudder_angle;
	chassic_rudder_preliminary_solution->Back_L.last_rudder_angle = chassic_rudder_preliminary_solution->Back_L.rudder_angle;
	chassic_rudder_preliminary_solution->Back_R.last_rudder_angle = chassic_rudder_preliminary_solution->Back_R.rudder_angle;
	chassic_rudder_preliminary_solution->Forward_R.last_rudder_angle = chassic_rudder_preliminary_solution->Forward_R.rudder_angle;

	// 根据设置速度求轮电机转速
	chassic_rudder_preliminary_solution->Forward_L.wheel_speed = -sqrt((pow((vy_set + vw_set * arm_cos_f32(45)), 2) + pow((vx_set + vw_set * arm_sin_f32(45)), 2)));
	chassic_rudder_preliminary_solution->Back_L.wheel_speed = sqrt((pow((vy_set - vw_set * arm_cos_f32(45)), 2) + pow((vx_set + vw_set * arm_sin_f32(45)), 2)));
	chassic_rudder_preliminary_solution->Back_R.wheel_speed = sqrt((pow((vy_set - vw_set * arm_cos_f32(45)), 2) + pow((vx_set - vw_set * arm_sin_f32(45)), 2)));
	chassic_rudder_preliminary_solution->Forward_R.wheel_speed = -sqrt((pow((vy_set + vw_set * arm_cos_f32(45)), 2) + pow((vx_set - vw_set * arm_sin_f32(45)), 2))); // 10.19

	// 根据速度反三角函数求角度                                                                                                                              //逆时针旋转
	chassic_rudder_preliminary_solution->Forward_L.rudder_angle = atan2((vy_set + vw_set * arm_cos_f32(45)), (vx_set + vw_set * arm_sin_f32(45))); // 0  3
	chassic_rudder_preliminary_solution->Back_L.rudder_angle = atan2((vy_set - vw_set * arm_cos_f32(45)), (vx_set + vw_set * arm_sin_f32(45)));	   // 1  2
	chassic_rudder_preliminary_solution->Back_R.rudder_angle = atan2((vy_set - vw_set * arm_cos_f32(45)), (vx_set - vw_set * arm_sin_f32(45)));
	chassic_rudder_preliminary_solution->Forward_R.rudder_angle = atan2((vy_set + vw_set * arm_cos_f32(45)), (vx_set - vw_set * arm_sin_f32(45)));

	// 求编码值变量
	chassic_rudder_preliminary_solution->Forward_L.ecd_add = chassic_rudder_preliminary_solution->Forward_L.rudder_angle / Motor_Ecd_to_Rad;
	chassic_rudder_preliminary_solution->Back_L.ecd_add = chassic_rudder_preliminary_solution->Back_L.rudder_angle / Motor_Ecd_to_Rad;
	chassic_rudder_preliminary_solution->Back_R.ecd_add = chassic_rudder_preliminary_solution->Back_R.rudder_angle / Motor_Ecd_to_Rad;
	chassic_rudder_preliminary_solution->Forward_R.ecd_add = chassic_rudder_preliminary_solution->Forward_R.rudder_angle / Motor_Ecd_to_Rad;

	// 数据更新
	chassic_rudder_preliminary_solution->Forward_L.last_ecd_add = chassic_rudder_preliminary_solution->Forward_L.ecd_add;
	chassic_rudder_preliminary_solution->Back_L.last_ecd_add = chassic_rudder_preliminary_solution->Back_L.ecd_add;
	chassic_rudder_preliminary_solution->Back_R.last_ecd_add = chassic_rudder_preliminary_solution->Back_R.ecd_add;
	chassic_rudder_preliminary_solution->Forward_R.last_ecd_add = chassic_rudder_preliminary_solution->Forward_R.ecd_add;
}

/**
 * @brief          轮电机速度设置
 * @param[in]      chassis_speed_set：底盘数据指针
 * @retval         none
 */
static void chassis_speed_control_set(chassis_move_t *chassis_speed_set)
{


	chassis_speed_set->motor_chassis[0].speed_set = chassis_speed_set->Forward_L.wheel_speed * chassis_speed_set->Forward_L.Judge_Speed_Direction * chassis_speed_set->Forward_L.Judge_Speed_cosk;
	chassis_speed_set->motor_chassis[1].speed_set = chassis_speed_set->Forward_R.wheel_speed * chassis_speed_set->Forward_R.Judge_Speed_Direction* chassis_speed_set->Forward_R.Judge_Speed_cosk;
	chassis_speed_set->motor_chassis[2].speed_set = chassis_speed_set->Back_L.wheel_speed * chassis_speed_set->Back_L.Judge_Speed_Direction* chassis_speed_set->Back_L.Judge_Speed_cosk;
	chassis_speed_set->motor_chassis[3].speed_set = chassis_speed_set->Back_R.wheel_speed * chassis_speed_set->Back_R.Judge_Speed_Direction* chassis_speed_set->Back_R.Judge_Speed_cosk;;

	int i;
	fp32 temp, max_vector, vector_rate;

	for (i = 0; i < 4; i++)
	{
		temp = fabs(chassis_speed_set->motor_chassis[i].speed_set);
		if (max_vector < temp)
		{
			max_vector = temp;
		}
	}
	// 限制最大速度
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
 * @brief          舵电机输出控制量设置
 * @param[in]      chassis_move_control_loop：底盘数据指针
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
 * @brief          舵电机零点过边界处理
 * @author         XQL
 * @param[in]      Rudder_ecd_judge
 * @retval         none
 */
static void Rudder_motor_ecd_judge(Rudder_Motor_t *Rudder_ecd_judge)
{

	if (Rudder_ecd_judge->ecd_zero_set > 8191)
	{
		Rudder_ecd_judge->ecd_set = Rudder_ecd_judge->ecd_zero_set - 8191;
	}
	else if (Rudder_ecd_judge->ecd_zero_set < 0)
	{
		Rudder_ecd_judge->ecd_set = Rudder_ecd_judge->ecd_zero_set + 8191;
	}
}

/**
 * @brief          舵电机跟随云台零点处理
 * @author         LYH
 * @param[in]      Rudder_ecd_set
 * @retval         none
 */
static void Rudder_motor_zero_ecd_set(chassis_move_t *Rudder_ecd_set)
{
	fp32 err = 0.0f;

	err = -1.0f * (chassis_move.gimbal_data.relative_angle_receive - chassis_move.chassis_relative_last);

	Rudder_ecd_set->Forward_L.ecd_zero_set = (Rudder_ecd_set->Forward_L.ecd_zero_set + err);
	Rudder_ecd_set->Back_L.ecd_zero_set = (Rudder_ecd_set->Back_L.ecd_zero_set + err);
	Rudder_ecd_set->Back_R.ecd_zero_set = (Rudder_ecd_set->Back_R.ecd_zero_set + err);
	Rudder_ecd_set->Forward_R.ecd_zero_set = (Rudder_ecd_set->Forward_R.ecd_zero_set + err);

	chassis_move.chassis_relative_last = chassis_move.gimbal_data.relative_angle_receive;

	// 零点处理
	Rudder_motor_ecd_judge(&Rudder_ecd_set->Forward_L);
	Rudder_motor_ecd_judge(&Rudder_ecd_set->Back_L);
	Rudder_motor_ecd_judge(&Rudder_ecd_set->Back_R);
	Rudder_motor_ecd_judge(&Rudder_ecd_set->Forward_R);
}

/**
 * @brief          舵电机控制量设置
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      chassis_motor：舵电机数据指针
 * @retval         none
 */

static void Rudder_motor_relative_angle_control(Rudder_Motor_t *chassis_motor)
{
 float angle;
	// 计算目标位置
	if (chassis_motor->ecd_add > 0)
	{
		if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add > 8191)
		{
			chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add - 8191;
		}
		else if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add < 8191)
		{
			chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add;
		}
	}
	else if (chassis_motor->ecd_add < 0)
	{
		if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add < 0)
		{
			chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add + 8191;
		}
		else if (chassis_motor->ecd_zero_set + chassis_motor->ecd_add > 0)
		{
			chassis_motor->ecd_set = chassis_motor->ecd_zero_set + chassis_motor->ecd_add;
		}
	}

	else if (chassis_move.chassis_motor_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW && chassis_motor->ecd_add == 0.0f)
	{
		chassis_motor->ecd_set = chassis_motor->gimbal_motor_measure->ecd;
	}
	else
	{
		chassis_motor->ecd_set=chassis_motor->gimbal_motor_measure->ecd;
	}

	chassis_motor->ecd_error = chassis_motor->ecd_set - chassis_motor->gimbal_motor_measure->ecd;

	// 就近原则
	if (chassis_motor->ecd_error > 4096)
	{
		chassis_motor->ecd_error = chassis_motor->ecd_error - 8191;
	}
	else if (chassis_motor->ecd_error < -4096)
	{
		chassis_motor->ecd_error = 8191 + chassis_motor->ecd_error;
	}
//最优角
		if (chassis_motor->ecd_error  > 2048 )
	{
		chassis_motor->ecd_error -= 4096;
		chassis_motor->Judge_Speed_Direction = -1;
	}
	else if (chassis_motor->ecd_error  < -2048 )
	{
		chassis_motor->ecd_error += 4096;
		chassis_motor->Judge_Speed_Direction = -1;
	}
	else
	{
		chassis_motor->Judge_Speed_Direction = 1;
	}
	
	// 计算根据舵角误差，计算cosK,以削减轮电机转速
		angle = (chassis_motor->ecd_error)*Motor_Ecd_to_Rad;
	if(fabs(angle)>90) angle = 90;
	else if(fabs(angle)<0.5) angle = 0;  //防止因极小的误差导致四个轮子转速不一样
  chassis_motor->Judge_Speed_cosk  = arm_cos_f32(angle)*arm_cos_f32(angle)*arm_cos_f32(angle);

	
	RUDDER_MOTOR_PID_CONTROL(chassis_motor);
}

/**
 * @brief          舵电机电流控制量计算
 * @param[in]      rudder_motor：舵电机数据指针
 * @retval         none
 */
static void RUDDER_MOTOR_PID_CONTROL(Rudder_Motor_t *rudder_motor)
{
	Matlab_PID_Calc(rudder_motor->ecd_error, 0, 0, &rudder_motor->rudder_control);
	rudder_motor->given_current = rudder_motor->rudder_control.rudder_out.Out1;
}

/**
 * @brief          舵电机功率控制
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      chassis_motor：舵电机数据指针
 * @retval         none
 */
static void RUDDER_POWER_CONTROL(chassis_move_t *rudder_power)
{
	if (chassis_move.chassis_power_buffer < 10) // 当缓冲能量过低时，全面降低功率
	{
		rudder_power->power_control.K = 3;
		rudder_power->power_control.POWER_MAX = 5;
	}
	else
	{
		rudder_power->power_control.K = 100;			// 35
		rudder_power->power_control.POWER_MAX = 40; // 40
	}
	rudder_power->rudder_given_current[0] = rudder_power->Forward_L.given_current;
	rudder_power->rudder_given_current[2] = rudder_power->Back_L.given_current;
	rudder_power->rudder_given_current[3] = rudder_power->Back_R.given_current;
	rudder_power->rudder_given_current[1] = rudder_power->Forward_R.given_current;

	int i;
	for (i = 0; i < 4; i++)
	{
		rudder_power->power_control.current[i] = rudder_power->rudder_given_current[i];
		rudder_power->power_control.totalCurrentTemp += abs(rudder_power->power_control.current[i]);
	}

	for (i = 0; i < 4; i++)
	{
		rudder_power->power_control.MAX_current[i] = (rudder_power->power_control.K * rudder_power->power_control.current[i] /
													  rudder_power->power_control.totalCurrentTemp) *
													 (rudder_power->power_control.POWER_MAX) / abs(rudder_power->rudder_speed[i]);
	}
	rudder_power->power_control.totalCurrentTemp = 0;

	for (i = 0; i < 4; i++)
	{
		rudder_power->rudder_given_current[i] = (int16_t)(rudder_power->rudder_given_current[i]);	
	}
	
	
}
/**
 * @brief          轮电机功率控制
 * @author         XQL  1.0
 *                 LYH  2.0
 * @param[in]      chassis_motor：轮电机数据指针
 * @retval         none
 */
void CHASSIC_MOTOR_POWER_CONTROL(chassis_move_t *chassis_motor)
{
	uint16_t max_power_limit = 40;
	fp32 input_power = 0;		 // 输入功率(缓冲能量环)
	fp32 scaled_motor_power[4];

	chassis_motor->power_control.POWER_MAX = 0; //最终底盘的最大功率初始化
	chassis_motor->power_control.forecast_total_power = 0; // 预测总功率初始化
	
	PID_Calc(&chassis_motor->buffer_pid, chassis_motor->chassis_power_buffer, 30); //使缓冲能量维持在一个稳定的范围,这里的PID没必要移植我的，用任意一个就行

	max_power_limit = chassis_motor->chassis_power_MAX;  //获得裁判系统的功率限制数值
	
	input_power = max_power_limit - chassis_motor->buffer_pid.out; //通过裁判系统的最大功率
	
	chassis_motor->power_control.power_charge = input_power*100; //超级电容的最大充电功率
	
	if(chassis_motor->power_control.power_charge>13000)		chassis_motor->power_control.power_charge =13000; //参考超电控制板允许的最大充电功率，溪地板子的新老不一样
	
	CAN_CMD_cap(chassis_motor->power_control.power_charge); // 设置超电的充电功率

	if (get_cap.capvot > 15) //当超电电压大于某个值(防止C620掉电)
	{
		if (chassis_move.key_C == 8000)   //主动超电，一般用于起步加速or冲刺or飞坡or上坡，chassis_move.key_C为此代码中超电开启按键
		{
			chassis_motor->power_control.POWER_MAX = 150;		
		}
		else
		{
			chassis_motor->power_control.POWER_MAX = input_power + 10;  //被动超电，相对以往能跑得更快点
			/***后续内容自补充
			if(get_cap.capvot > 19&&chassis_motor->chassis_power_buffer>20)
			chassis_motor->power_control.POWER_MAX = input_power + 20;
			else
			chassis_motor->power_control.POWER_MAX = input_power + 10; 
			***/
		}
	}
	else
	{
		chassis_motor->power_control.POWER_MAX = input_power-5;
	}

	for (uint8_t i = 0; i < 4; i++) // 获得所有3508电机的功率和总功率
	{
		chassis_motor->power_control.forecast_motor_power[i] =
    		chassis_motor->motor_chassis[i].give_current * toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm +
				k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm +
				k2* chassis_motor->motor_chassis[i].give_current *chassis_motor->motor_chassis[i].give_current + constant_3508;

		if (chassis_motor->power_control.forecast_motor_power < 0)  	continue; // 忽略负电
		
		chassis_motor->power_control.forecast_total_power += chassis_motor->power_control.forecast_motor_power[i];
	}
	
	if (chassis_motor->power_control.forecast_total_power > chassis_motor->power_control.POWER_MAX) // 超功率模型衰减
	{
		fp32 power_scale = chassis_motor->power_control.POWER_MAX / chassis_motor->power_control.forecast_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_motor_power[i] = chassis_motor->power_control.forecast_motor_power[i] * power_scale; // 获得衰减后的功率
			
			if (scaled_motor_power[i] < 0)		continue;

			fp32 b = toque_coefficient * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm;
			fp32 c = k1 * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_motor->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_motor_power[i] + constant_3508;

			if (chassis_motor->motor_chassis[i].give_current> 0)  //避免超过最大电流
			{					
				chassis_motor->power_control.MAX_current[i] = (-b + sqrt(b * b - 4 * k2 * c)) / (2 * k2);  
				if (chassis_motor->power_control.MAX_current[i] > 16000)
				{
					chassis_motor->motor_chassis[i].give_current = 16000;
				}
				else
					chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
			}
			else
			{
				chassis_motor->power_control.MAX_current[i] = (-b - sqrt(b * b - 4 * k2 * c)) / (2 * k2);
				if (chassis_motor->power_control.MAX_current[i] < -16000)
				{
					chassis_motor->motor_chassis[i].give_current = -16000;
				}
				else
					chassis_motor->motor_chassis[i].give_current = chassis_motor->power_control.MAX_current[i];
			}
		}
	}
}

/**
 * @brief          轮电机电流控制量计算
 * @author         LYH
 * @param[in]      chassis_motor：轮电机数据指针
 * @retval         none
 */
static void CHASSIC_MOTOR_PID_CONTROL(chassis_move_t *chassis_motor)
{
	chassis_speed_control_set(chassis_motor);
	int i;

	for (i = 0; i < 4; i++)
	{
		chassis_motor->motor_chassis[i].give_current = PID_Calc(&chassis_motor->motor_chassis[i].chassis_pid,
																chassis_motor->motor_chassis[i].speed, chassis_motor->motor_chassis[i].speed_set);
		if (abs(chassis_motor->power_control.speed[i]) < chassis_motor->power_control.SPEED_MIN)
		{
			chassis_motor->power_control.speed[i] = chassis_motor->power_control.SPEED_MIN;
		}
	}

	CHASSIC_MOTOR_POWER_CONTROL(chassis_motor);
}

/**
  * @brief          舵电机PID参初始化
  * @author         XQL 1.0
				           	LYH 2.0
  * @param[in]      rudder_init：底盘数据指针
  * @param[in]      PID[8]：PID参数
  * @retval         none
  */
static void RUDDER_PID_INIT(chassis_move_t *rudder_init, const fp32 PID[8])
{
	rudder_init->Forward_L.rudder_control.rudder_in.P_P = rudder_init->Forward_R.rudder_control.rudder_in.P_P =rudder_init->Back_L.rudder_control.rudder_in.P_P = rudder_init->Back_R.rudder_control.rudder_in.P_P = PID[0];

	rudder_init->Forward_L.rudder_control.rudder_in.P_I = rudder_init->Forward_R.rudder_control.rudder_in.P_I =rudder_init->Back_L.rudder_control.rudder_in.P_I = rudder_init->Back_R.rudder_control.rudder_in.P_I = PID[1];

	rudder_init->Forward_L.rudder_control.rudder_in.P_D = rudder_init->Forward_R.rudder_control.rudder_in.P_D =rudder_init->Back_L.rudder_control.rudder_in.P_D = rudder_init->Back_R.rudder_control.rudder_in.P_D = PID[2];

	rudder_init->Forward_L.rudder_control.rudder_in.P_N = rudder_init->Forward_R.rudder_control.rudder_in.P_N =rudder_init->Back_L.rudder_control.rudder_in.P_N = rudder_init->Back_R.rudder_control.rudder_in.P_N = PID[3];

	rudder_init->Forward_L.rudder_control.rudder_in.S_P = rudder_init->Forward_R.rudder_control.rudder_in.S_P =rudder_init->Back_L.rudder_control.rudder_in.S_P = rudder_init->Back_R.rudder_control.rudder_in.S_P = PID[4];

	rudder_init->Forward_L.rudder_control.rudder_in.S_I = rudder_init->Forward_R.rudder_control.rudder_in.S_I =rudder_init->Back_L.rudder_control.rudder_in.S_I = rudder_init->Back_R.rudder_control.rudder_in.S_I = PID[5];

	rudder_init->Forward_L.rudder_control.rudder_in.S_D = rudder_init->Forward_R.rudder_control.rudder_in.S_D =rudder_init->Back_L.rudder_control.rudder_in.S_D = rudder_init->Back_R.rudder_control.rudder_in.S_D = PID[6];
	
	rudder_init->Forward_L.rudder_control.rudder_in.S_N = rudder_init->Forward_R.rudder_control.rudder_in.S_N =rudder_init->Back_L.rudder_control.rudder_in.S_N = rudder_init->Back_R.rudder_control.rudder_in.S_N = PID[7];
}

/**
 * @brief          速度输入量设置
 * @author         LYH
 * @param[in]      *vx_set：x方向速度设置量
 * @param[in]      *vy_set：y方向速度设置量
 * @param[in]      chassis_move_rc_to_vector：底盘数据指针
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
		return;
	}

	if (chassis_move.key_C == 8000) // 根据等级变化开超电后的功率
	{
		kx = 2.0f;
		ky = 2.0f;
		kw = 1.0f;
	}
	else
	{
		kx = 1.5f;
		ky = 1.5f;
		kw = 1.0f;
	}

	// 一阶低通滤波作为底盘速度输入
    ramp_calc(&chassis_move_rc_to_vector->vx_ramp, -chassis_move_rc_to_vector->vx_set_CANsend/100);
    ramp_calc(&chassis_move_rc_to_vector->vy_ramp, -chassis_move_rc_to_vector->vy_set_CANsend/100);
	
	*vx_set += chassis_move_rc_to_vector->vx_ramp.out;
	*vy_set += chassis_move_rc_to_vector->vy_ramp.out;
	
	*vx_set = kx * (*vx_set);
	*vy_set = ky * (*vy_set);
}

