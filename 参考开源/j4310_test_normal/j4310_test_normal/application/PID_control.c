#include "PID_Control.h"


/**
 * @brief PID��ʼ��
 *
 * @param pid PID������ָ��
 * @param K_P Pֵ
 * @param K_I Iֵ
 * @param K_D Dֵ
 * @param K_F ǰ��
 * @param I_Out_Max �����޷�
 * @param Out_Max ����޷�
 * @param D_T ʱ��Ƭ����
 */
void PID_Init(PID_control* pid, float K_P, float K_I, float K_D, float K_F, float I_Out_Max, float Out_Max, float D_T, float Dead_Zone, float I_Variable_Speed_A, float I_Variable_Speed_B, float I_Separate_Threshold, Enum_PID_D_First D_First)
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
 * @brief ��ȡ�������
 *
 * @param pid PID������ָ��
 * @return float �������
 */
float PID_Get_Integral_Error(PID_control* pid)
{
    return pid->Integral_Error;
}

/**
 * @brief ��ȡ���ֵ
 *
 * @param pid PID������ָ��
 * @return float ���ֵ
 */
float PID_Get_Out(PID_control* pid)
{
    return pid->Out;
}

/**
 * @brief �趨PID��P
 *
 * @param pid PID������ָ��
 * @param K_P PID��P
 * @param K_I PID��I
 * @param K_D PID��D
 * @param K_F PID��F
 */
void PID_Set_K_P_I_D_F(PID_control* pid, float K_P,float K_I,float K_D,float K_F)
{
    pid->K_P = K_P;
	pid->K_P = K_I;
	pid->K_P = K_D;
	pid->K_P = K_F;
}

/**
 * @brief �趨�����޷�, 0Ϊ������
 *
 * @param pid PID������ָ��
 * @param I_Out_Max �����޷�, 0Ϊ������
 */
void PID_Set_I_Out_Max(PID_control* pid, float I_Out_Max)
{
    pid->I_Out_Max = I_Out_Max;
}

/**
 * @brief �趨����޷�, 0Ϊ������
 *
 * @param pid PID������ָ��
 * @param Out_Max ����޷�, 0Ϊ������
 */
void PID_Set_Out_Max(PID_control* pid, float Out_Max)
{
    pid->Out_Max = Out_Max;
}

/**
 * @brief �趨�����ڶ���ֵ, 0Ϊ������
 *
 * @param pid PID������ָ��
 * @param I_Variable_Speed_A �����ڶ���ֵ, 0Ϊ������
 */
void PID_Set_I_Variable_Speed_A(PID_control* pid, float I_Variable_Speed_A)
{
    pid->I_Variable_Speed_A = I_Variable_Speed_A;
}

/**
 * @brief �趨��������, 0Ϊ������
 *
 * @param pid PID������ָ��
 * @param I_Variable_Speed_B ��������, 0Ϊ������
 */
void PID_Set_I_Variable_Speed_B(PID_control* pid, float I_Variable_Speed_B)
{
    pid->I_Variable_Speed_B = I_Variable_Speed_B;
}

/**
 * @brief �趨���ַ�����ֵ����Ϊ����, 0Ϊ������
 *
 * @param pid PID������ָ��
 * @param I_Separate_Threshold ���ַ�����ֵ����Ϊ����, 0Ϊ������
 */
void PID_Set_I_Separate_Threshold(PID_control* pid, float I_Separate_Threshold)
{
    pid->I_Separate_Threshold = I_Separate_Threshold;
}

/**
 * @brief �趨Ŀ��ֵ
 *
 * @param pid PID������ָ��
 * @param Target Ŀ��ֵ
 */
void PID_Set_Target(PID_control* pid, float Target)
{
    pid->Target = Target;
}

/**
 * @brief �趨��ǰֵ
 *
 * @param pid PID������ָ��
 * @param Now ��ǰֵ
 */
void PID_Set_Now(PID_control* pid, float Now)
{
    pid->Now = Now;
}

/**
 * @brief �趨����, һ�����ڻ�������
 *
 * @param pid PID������ָ��
 * @param Integral_Error ����ֵ
 */
void PID_Set_Integral_Error(PID_control* pid, float Integral_Error)
{
    pid->Integral_Error = Integral_Error;
}

/**
 * @brief PID����ֵ
 *
 * @param pid PID������ָ��
 */
void PID_TIM_Adjust_PeriodElapsedCallback(PID_control* pid)
{
    // P���
    float p_out = 0.0f;
    // I���
    float i_out = 0.0f;
    // D���
    float d_out = 0.0f;
    // F���
    float f_out = 0.0f;
    // ���
    float error;
    // ����ֵ���
    float abs_error;
    // ���Ա��ٻ���
    float speed_ratio;

    error = pid->Target - pid->Now;
    abs_error = Float_Math_Abs(error);

    // �ж�����
    if (abs_error < pid->Dead_Zone)
    {
        pid->Target = pid->Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    // ����p��
    p_out = pid->K_P * error;

    // ����i��
    if (pid->I_Variable_Speed_A == 0.0f && pid->I_Variable_Speed_B == 0.0f)
    {
        // �Ǳ��ٻ���
        speed_ratio = 1.0f;
    }
    else
    {
        // ���ٻ���
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

    // �����޷�
    if (pid->I_Out_Max != 0.0f)
    {
        Math_Constrain(&pid->Integral_Error, -pid->I_Out_Max / pid->K_I, pid->I_Out_Max / pid->K_I);
    }

    if (pid->I_Separate_Threshold == 0.0f)
    {
        // û�л��ַ���
        pid->Integral_Error += speed_ratio * pid->D_T * error;
        i_out = pid->K_I * pid->Integral_Error;
    }
    else
    {
        // ���ַ���ʹ��
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

    // ����d��
    if (pid->D_First == PID_D_First_DISABLE)
    {
        // û��΢������
        d_out = pid->K_D * (error - pid->Pre_Error) / pid->D_T;
    }
    else
    {
        // ΢������ʹ��
        d_out = pid->K_D * (pid->Out - pid->Pre_Out) / pid->D_T;
    }

    // ����ǰ��
    f_out = (pid->Target - pid->Pre_Target) * pid->K_F;

    // �����ܹ������
    pid->Out = p_out + i_out + d_out + f_out;

    // ����޷�
    if (pid->Out_Max != 0.0f)
    {
        Math_Constrain(&pid->Out, -pid->Out_Max, pid->Out_Max);
    }

    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Out = pid->Out;
    pid->Pre_Error = error;
}

