
#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "mathh.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 微分先行
 *
 */
typedef enum {
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
} Enum_PID_D_First;

/**
 * @brief Reusable, PID算法
 *
 */
typedef struct {
    // PID计时器周期, s
    float D_T;
    // 死区, Error在其绝对值内不输出
    float Dead_Zone;
    // 微分先行
    Enum_PID_D_First D_First;

    // 常量
    // PID的P
    float K_P;
    // PID的I
    float K_I;
    // PID的D
    float K_D;
    // 前馈
    float K_F;
	
	//输出值
    float Out;

    // 积分限幅, 0为不限制
    float I_Out_Max;
    // 输出限幅, 0为不限制
    float Out_Max;

    // 变速积分定速内段阈值, 0为不限制
    float I_Variable_Speed_A;
    // 变速积分变速区间, 0为不限制
    float I_Variable_Speed_B;
    // 积分分离阈值，需为正数, 0为不限制
    float I_Separate_Threshold;

    // 目标值
    float Target;
    // 当前值
    float Now;

    // 积分值
    float Integral_Error;
	
	//
	float D_Buf;
    // 之前的当前值
    float Pre_Now;
    // 之前的目标值
    float Pre_Target;
    // 之前的输出值
    float Pre_Out;
    // 前向误差
    float Pre_Error;
} PID_control;

/* Exported function declarations --------------------------------------------*/

void PID_UP_Init(PID_control* pid, float K_P, float K_I, float K_D, float K_F, float I_Out_Max, float Out_Max, float D_T, float Dead_Zone, float I_Variable_Speed_A, float I_Variable_Speed_B, float I_Separate_Threshold, Enum_PID_D_First D_First);
float PID_Get_Integral_Error(PID_control* pid);
float PID_Get_Out(PID_control* pid);
void PID_Set_K_P_I_D_F(PID_control* pid, float K_P,float K_I,float K_D,float K_F);
void PID_Set_I_Out_Max(PID_control* pid, float I_Out_Max);
void PID_Set_Out_Max(PID_control* pid, float Out_Max);
void PID_Set_I_Variable_Speed_A(PID_control* pid, float Variable_Speed_I_A);
void PID_Set_I_Variable_Speed_B(PID_control* pid, float Variable_Speed_I_B);
void PID_Set_I_Separate_Threshold(PID_control* pid, float I_Separate_Threshold);
void PID_Set_Target(PID_control* pid, float Target);
void PID_Set_Now(PID_control* pid, float Now);
void PID_Set_Integral_Error(PID_control* pid, float Integral_Error);
void PID_TIM_Adjust_PeriodElapsedCallback(PID_control* pid);

#endif
