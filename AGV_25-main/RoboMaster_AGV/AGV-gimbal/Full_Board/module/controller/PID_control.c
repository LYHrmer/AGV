#include "PID_Control.h"


/**
 * @brief PID初始化
 *
 * @param pid PID控制器指针
 * @param K_P P值
 * @param K_I I值
 * @param K_D D值
 * @param K_F 前馈
 * @param I_Out_Max 积分限幅
 * @param Out_Max 输出限幅
 * @param D_T 时间片长度
 */
void PID_UP_Init(PID_control* pid, float K_P, float K_I, float K_D, float K_F, float I_Out_Max, float Out_Max, float D_T, float Dead_Zone, float I_Variable_Speed_A, float I_Variable_Speed_B, float I_Separate_Threshold, Enum_PID_D_First D_First)
{
    pid->K_P = K_P;
    pid->K_I = K_I;
    pid->K_D = K_D;
    pid->K_F = K_F;
    pid->I_Out_Max = I_Out_Max;
    pid->Out_Max = Out_Max;
    pid->D_T = D_T;
    pid->Dead_Zone = Dead_Zone;
    pid->I_Variable_Speed_A = I_Variable_Speed_A;
    pid->I_Variable_Speed_B = I_Variable_Speed_B;
    pid->I_Separate_Threshold = I_Separate_Threshold;
    pid->D_First = D_First;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param pid PID控制器指针
 * @param Integral_Error 积分值
 */
void PID_Set_Integral_Error(PID_control* pid, float Integral_Error)
{
    pid->Integral_Error = Integral_Error;
}

/**
 * @brief PID调整值
 *
 * @param pid PID控制器指针
 */
void PID_TIM_Adjust_PeriodElapsedCallback(PID_control* pid)
{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    // 误差
    float error;
    // 绝对值误差
    float abs_error;
    // 线性变速积分
    float speed_ratio;

    error = pid->Target - pid->Now;
    abs_error = Float_Math_Abs(error);
    // 判断死区
    if (abs_error < pid->Dead_Zone)
    {
        pid->Target = pid->Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    // 计算p项
    p_out = pid->K_P * error;

    // 计算i项
    if (pid->I_Variable_Speed_A == 0.0f && pid->I_Variable_Speed_B == 0.0f)
    {
        // 非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        // 变速积分
        if (abs_error <= pid->I_Variable_Speed_B)
        {
            speed_ratio = 1.0f;
        }
        else if (pid->I_Variable_Speed_B < abs_error && abs_error < pid->I_Variable_Speed_A + pid->I_Variable_Speed_B)
        {
            speed_ratio = (pid->I_Variable_Speed_A + pid->I_Variable_Speed_B - abs_error) / pid->I_Variable_Speed_A;
        }
        else
        {
            speed_ratio = 0.0f;
        }
    }

    // 积分限幅
    if (pid->I_Out_Max != 0.0f)
    {
        Math_Constrain(&pid->Integral_Error, -pid->I_Out_Max / pid->K_I, pid->I_Out_Max / pid->K_I);
    }

    if (pid->I_Separate_Threshold == 0.0f)
    {
        // 没有积分分离
        pid->Integral_Error += speed_ratio * pid->D_T * error;
        i_out = pid->K_I * pid->Integral_Error;
    }
    else
    {
        // 积分分离使能
        if (abs_error < pid->I_Separate_Threshold)
        {
            pid->Integral_Error += speed_ratio * pid->D_T * error;
            i_out = pid->K_I * pid->Integral_Error;
        }
        else
        {
            pid->Integral_Error = 0.0f;
            i_out = 0.0f;
        }
    }

    // 计算d项
    if (pid->D_First == PID_D_First_DISABLE)
    {
        // 没有微分先行
		pid->D_Buf = error - pid->Pre_Error;
        d_out = pid->K_D * (error - pid->Pre_Error) / pid->D_T;
    }
    else
    {
        // 微分先行使能
		pid->D_Buf = error - pid->Pre_Error;
        d_out = pid->K_D * (pid->Out - pid->Pre_Out) / pid->D_T;
    }

    // 计算前馈
    f_out = (pid->Target - pid->Pre_Target) * pid->K_F;

    // 计算总共的输出
    pid->Out = p_out + i_out + d_out + f_out;

    // 输出限幅
    if (pid->Out_Max != 0.0f)
    {
        Math_Constrain(&pid->Out, -pid->Out_Max, pid->Out_Max);
    }

    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Out = pid->Out;
    pid->Pre_Error = error;
}

