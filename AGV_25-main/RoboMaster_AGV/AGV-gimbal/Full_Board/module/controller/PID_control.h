
#ifndef ALG_PID_H
#define ALG_PID_H

/* Includes ------------------------------------------------------------------*/

#include "mathh.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief ΢������
 *
 */
typedef enum {
    PID_D_First_DISABLE = 0,
    PID_D_First_ENABLE,
} Enum_PID_D_First;

/**
 * @brief Reusable, PID�㷨
 *
 */
typedef struct {
    // PID��ʱ������, s
    float D_T;
    // ����, Error�������ֵ�ڲ����
    float Dead_Zone;
    // ΢������
    Enum_PID_D_First D_First;

    // ����
    // PID��P
    float K_P;
    // PID��I
    float K_I;
    // PID��D
    float K_D;
    // ǰ��
    float K_F;
	
	//���ֵ
    float Out;

    // �����޷�, 0Ϊ������
    float I_Out_Max;
    // ����޷�, 0Ϊ������
    float Out_Max;

    // ���ٻ��ֶ����ڶ���ֵ, 0Ϊ������
    float I_Variable_Speed_A;
    // ���ٻ��ֱ�������, 0Ϊ������
    float I_Variable_Speed_B;
    // ���ַ�����ֵ����Ϊ����, 0Ϊ������
    float I_Separate_Threshold;

    // Ŀ��ֵ
    float Target;
    // ��ǰֵ
    float Now;

    // ����ֵ
    float Integral_Error;
	
	//
	float D_Buf;
    // ֮ǰ�ĵ�ǰֵ
    float Pre_Now;
    // ֮ǰ��Ŀ��ֵ
    float Pre_Target;
    // ֮ǰ�����ֵ
    float Pre_Out;
    // ǰ�����
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
