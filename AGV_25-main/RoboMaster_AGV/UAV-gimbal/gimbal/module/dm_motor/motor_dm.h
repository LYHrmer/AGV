/**
 * @file motor_dm.h
 * @author CHR
 * @brief ����CAN������������
 * @version 0.1
 * @date 2024-11-04
 *
 *
 */
#ifndef DVC_MOTOR_DM_H
#define DVC_MOTOR_DM_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CAN_receive.h"
#include "Mathh.h"
#include "arm_math.h"
#include "can.h"
#include "PID_Control.h"

/**
 * @brief ������״̬
 *
 */
typedef enum
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
}Enum_Motor_DM_Status;

/**
 * @brief ��������IDö������, һ����ģʽ��
 *
 */
typedef enum 
{
    Motor_DM_ID_0x301 = 1,
    Motor_DM_ID_0x302,
    Motor_DM_ID_0x303,
    Motor_DM_ID_0x304,
    Motor_DM_ID_0x305,
    Motor_DM_ID_0x306,
    Motor_DM_ID_0x307,
    Motor_DM_ID_0x308,
}Enum_Motor_DM_Motor_ID_1_To_4;

/**
 * @brief ����������״̬, ��ͳģʽ��Ч
 *
 */
typedef enum 
{
    Motor_DM_Control_Status_DISABLE = 0x0,
    Motor_DM_Control_Status_ENABLE,
    Motor_DM_Control_Status_OVERVOLTAGE = 0x8,
    Motor_DM_Control_Status_UNDERVOLTAGE,
    Motor_DM_Control_Status_OVERCURRENT,
    Motor_DM_Control_Status_MOS_OVERTEMPERATURE,
    Motor_DM_Control_Status_ROTOR_OVERTEMPERATURE,
    Motor_DM_Control_Status_LOSE_CONNECTION,
    Motor_DM_Control_Status_MOS_OVERLOAD,
}Enum_Motor_DM_Control_Status_Normal;

/**
 * @brief ���������Ʒ�ʽ
 *
 */
typedef enum  
{
    Motor_DM_Control_Method_NORMAL_MIT = 0,
    Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,
    Motor_DM_Control_Method_NORMAL_OMEGA,
    Motor_DM_Control_Method_NORMAL_EMIT,
    Motor_DM_Control_Method_1_TO_4_CURRENT,
    Motor_DM_Control_Method_1_TO_4_OMEGA,
    Motor_DM_Control_Method_1_TO_4_ANGLE,
}Enum_Motor_DM_Control_Method;

/**
 * @brief ��������ͳģʽԴ����
 *
 */
typedef struct 
{
    uint8_t CAN_ID : 4;
    uint8_t Control_Status_Enum : 4;
    uint16_t Angle_Reverse;
    uint8_t Omega_11_4;
    uint8_t Omega_3_0_Torque_11_8;
    uint8_t Torque_7_0;
    uint8_t MOS_Temperature;
    uint8_t Rotor_Temperature;
} __attribute__((packed)) Struct_Motor_DM_CAN_Rx_Data_Normal;
											 
/**
 * @brief ������һ����ģʽԴ����
 *
 */
typedef struct 
{
    uint16_t Encoder_Reverse;
    // ���ٶ�100��
    int16_t Omega_Reverse;
    // ����ֵ, mA
    int16_t Current_Reverse;
    uint8_t Rotor_Temperature;
    uint8_t MOS_Temperature;
} __attribute__((packed)) Struct_Motor_DM_CAN_Rx_Data_1_To_4;

/**
 * @brief ����������Դ����, MIT���Ʊ���
 *
 */
typedef struct {
    uint16_t Control_Angle_Reverse;
    uint8_t Control_Omega_11_4;
    uint8_t Control_Omega_3_0_K_P_11_8;
    uint8_t K_P_7_0;
    uint8_t K_D_11_4;
    uint8_t K_D_3_0_Control_Torque_11_8;
    uint8_t Control_Torque_7_0;
} __attribute__((packed)) Struct_Motor_DM_CAN_Tx_Data_Normal_MIT;

/**
 * @brief ����������Դ����, λ���ٶȿ��Ʊ���
 *
 */
typedef struct {
    float Control_Angle;
    float Control_Omega;
} __attribute__((packed)) Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega;

/**
 * @brief ����������Դ����, �ٶȿ��Ʊ���
 *
 */
typedef struct {
    float Control_Omega;
} __attribute__((packed)) Struct_Motor_DM_CAN_Tx_Data_Normal_Omega;

/**
 * @brief ����������Դ����, EMIT���Ʊ���
 *
 */
typedef struct {
    float Control_Angle;
    uint16_t Control_Omega; // �޶��ٶ���, rad/s��100��
    uint16_t Control_Current; // �޶�������, �������ֵ��10000��
} __attribute__((packed)) Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT;

/**
 * @brief �������������������, ��ͳģʽ��Ч
 *
 */
typedef struct {
    Enum_Motor_DM_Control_Status_Normal Control_Status;
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
} Struct_Motor_DM_Rx_Data_Normal;

/**
 * @brief �������������������, һ����ģʽ��Ч
 *
 */
typedef struct {
    float Now_Angle;
    float Now_Omega;
    float Now_Current;
    float Now_MOS_Temperature;
    float Now_Rotor_Temperature;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
} Struct_Motor_DM_Rx_Data_1_To_4;

/**
 * @brief Reusable, ������, ��ͳģʽ
 * û�����, ������λ�������
 * ��ʼ���ĽǶ�, ���ٶ�, Ť��, �����Ȳ�����J4310���Ĭ��ֵ
 *
 */
typedef struct {
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    uint16_t CAN_Rx_ID;
    uint16_t CAN_Tx_ID;

    float Angle_Max;
    float Omega_Max;
    float Torque_Max;
    float Current_Max;

    uint32_t Flag;
    uint32_t Pre_Flag;
    uint8_t Tx_Data[8];
    Enum_Motor_DM_Status Motor_DM_Status;
    Struct_Motor_DM_Rx_Data_Normal Rx_Data;
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method;

    float Control_Angle;
    float Control_Omega;
    float Control_Torque;
    float Control_Current;
    float K_P;
    float K_D;
} Motor_DM_Normal;

/**
 * @brief Reusable, ������, һ����ģʽ
 * û�����, ������λ�������
 *
 */
typedef struct {
	
	PID_control Angle_PID;
	PID_control Omega_PID;
	
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    Enum_Motor_DM_Motor_ID_1_To_4 CAN_Rx_ID;
    Enum_Motor_DM_Status Motor_DM_Status;
    Struct_Motor_DM_Rx_Data_1_To_4 Rx_Data;
    Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
	
	uint8_t *Tx_Data;
	uint8_t Now_Temperature;
	uint16_t Encoder_Num_Per_Round;
	uint32_t Flag;
    uint32_t Pre_Flag;
	int32_t Encoder_Offset;

    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    float Current_Max;
	float Current_To_Out;
    float Theoretical_Output_Current_Max;
	float Out;
    float Target_Angle;
    float Target_Omega;
    float Target_Current;
    float Feedforward_Omega;
    float Feedforward_Current;
} Motor_DM_1_To_4;

extern uint8_t DM_Motor_CAN_Message_Enter[8];

void Motor_DM_Normal_Init(Motor_DM_Normal *motor, CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Current_Max);
void Motor_DM_Normal_CAN_RxCpltCallback(Motor_DM_Normal *motor, uint8_t *Rx_Data);
void Motor_DM_Normal_CAN_Send_Clear_Error(Motor_DM_Normal *motor);
void Motor_DM_Normal_CAN_Send_Enable(Motor_DM_Normal *motor);
void Motor_DM_Normal_CAN_Send_Disable(Motor_DM_Normal *motor);
void Motor_DM_Normal_CAN_Send_Save_Zero(Motor_DM_Normal *motor);
void Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(Motor_DM_Normal *motor);
void Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(Motor_DM_Normal *motor);
void Motor_DM_Normal_Data_Process(Motor_DM_Normal *motor);
void Motor_DM_Normal_Output(Motor_DM_Normal *motor);
void Motor_DM_1_To_4_Init(Motor_DM_1_To_4 *motor, CAN_HandleTypeDef *hcan, Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID, Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, int32_t __Encoder_Offset, float __Current_Max);
void Motor_DM_1_To_4_CAN_RxCpltCallback(Motor_DM_1_To_4 *motor, uint8_t *Rx_Data);
void Motor_DM_1_To_4_TIM_100ms_Alive_PeriodElapsedCallback(Motor_DM_1_To_4 *motor);
void Motor_DM_1_To_4_TIM_1ms_Calculate_PeriodElapsedCallback(Motor_DM_1_To_4 *motor);
void Motor_DM_1_To_4_Data_Process(Motor_DM_1_To_4 *motor);
void Motor_DM_1_To_4_PID_Calculate(Motor_DM_1_To_4 *motor);
void Motor_DM_1_To_4_Output(Motor_DM_1_To_4 *motor);

#endif
