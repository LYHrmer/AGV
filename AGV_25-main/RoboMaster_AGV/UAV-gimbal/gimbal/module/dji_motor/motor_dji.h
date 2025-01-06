/**
 * @file motor_dji.h
 * @author CHR
 * @brief ��CAN������������
 * @version 0.1
 * @date 2024-11-04
 *
 *
 */

#ifndef MOTOR_DJI_H
#define MOTOR_DJI_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "CAN_receive.h"
#include "Mathh.h"
#include "arm_math.h"
#include "fdcan.h"
#include "PID_Control.h"

// RPM���㵽rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)

// ��ѹ�������ת��ϵ��
#define GM6020_Voltage_To_Out (25000.0f / 24.0f)
// �����������ת��ϵ��
#define GM6020_Current_To_Out (16384.0f / 3.0f)
// ������������ѹ
#define GM6020_Theoretical_Output_Voltage_Max 24.0f;
// ��������������
#define GM6020_Theoretical_Output_Current_Max 3.0f;

// �����������ת��ϵ��
#define C610_Current_To_Out (10000.0f / 10.0f)

// �����������ת��ϵ��
#define C620_Current_To_Out (16384.0f / 20.0f)
// ��������������
#define C620_Theoretical_Output_Current_Max = 20.0f;


/* Exported types ------------------------------------------------------------*/

/**
 * @brief ���״̬
 *
 */
typedef enum {
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE,
} Enum_CAN_Motor_Status;

/**
 * @brief �Ƿ������ʿ���, ��ʱ����������Ϊ���ģʽ, ���ɵ�ѹ����
 *
 */
typedef enum
{
    Motor_DJI_Power_Limit_Status_DISABLE = 0,
    Motor_DJI_Power_Limit_Status_ENABLE,
}Enum_Motor_DJI_Power_Limit_Status;

/**
 * @brief �󽮵�������汾, Ӱ��GM6020���������ʽ
 *
 */
typedef enum
{
    Motor_DJI_GM6020_Driver_Version_DEFAULT = 0,
    Motor_DJI_GM6020_Driver_Version_2023,
}Enum_Motor_DJI_GM6020_Driver_Version;

/**
 * @brief �󽮵��Դ����
 *
 */
typedef struct __attribute__((packed))
{
    uint16_t Encoder_Reverse;
    int16_t Omega_Reverse;
    int16_t Current_Reverse;
    uint8_t Temperature;
    uint8_t Reserved;
} Struct_Motor_DJI_CAN_Rx_Data;

/**
 * @brief �󽮵���������������
 *
 */
typedef struct
{
    float Now_Angle;
    float Now_Omega;
    float Now_Current;
    float Now_Temperature;
    float Now_Power;
    float Now_Velocity;
    uint32_t Pre_Encoder;
    int32_t Total_Encoder;
    int32_t Total_Round;
}Struct_Motor_DJI_Rx_Data;

/**
 * @brief CAN�����IDö������
 *
 */
typedef enum {
    CAN_Motor_ID_UNDEFINED = 0,
    CAN_Motor_ID_0x201,
    CAN_Motor_ID_0x202,
    CAN_Motor_ID_0x203,
    CAN_Motor_ID_0x204,
    CAN_Motor_ID_0x205,
    CAN_Motor_ID_0x206,
    CAN_Motor_ID_0x207,
    CAN_Motor_ID_0x208,
    CAN_Motor_ID_0x209,
    CAN_Motor_ID_0x20A,
    CAN_Motor_ID_0x20B,
} Enum_CAN_Motor_ID;

/**
 * @brief CAN�����ID�������
 *
 */
typedef enum {
    CAN_Motor_ID_Status_FREE = 0,
    CAN_Motor_ID_Status_ALLOCATED,
} Enum_CAN_Motor_ID_Status;

/**
 * @brief ������Ʒ�ʽ
 *
 */
typedef enum {
    Motor_DJI_Control_Method_VOLTAGE = 0,
    Motor_DJI_Control_Method_Velocity,
    Motor_DJI_Control_Method_CURRENT,
    Motor_DJI_Control_Method_TORQUE,
    Motor_DJI_Control_Method_OMEGA,
    Motor_DJI_Control_Method_ANGLE,
} Enum_Control_Method;

/**
 * @brief GM6020��ˢ���, ��Ƭ�����������ѹ
 *
 */
typedef struct {
    // PID�ǶȻ�����
    PID_control PID_Angle;
    // PID���ٶȻ�����
    PID_control PID_Omega;
    // PIDŤ�ػ�����
    PID_control PID_Current;

    // ��ʼ����ر���
    Enum_Motor_DJI_GM6020_Driver_Version Driver_Version;
    Struct_CAN_Manage_Object *CAN_Manage_Object; // �󶨵�CAN
    Enum_CAN_Motor_ID CAN_ID; // �����ݰ󶨵�CAN ID
    uint8_t *CAN_Tx_Data; // ���ͻ�����
    uint32_t Encoder_Offset; // ������ƫ��
    float Omega_Max; // ����ٶ�
    float Voltage_Max;
    float Current_Max;

    // ����
    uint16_t Encoder_Num_Per_Round; // һȦ�������̶�
    uint16_t Output_Max; // ��������ѹ

    // �ڲ�����
    uint32_t Flag; // ��ǰʱ�̵ĵ������flag
    uint32_t Pre_Flag; // ǰһʱ�̵ĵ������flag

    // �������ӿ���Ϣ
    Struct_Motor_DJI_Rx_Data Rx_Data;   //���������
    Struct_Motor_DJI_CAN_Rx_Data Rx_Origin; //CAN����ԭʼ����

    // ������
    Enum_CAN_Motor_Status CAN_Motor_Status; // ���״̬

    // д����
    Enum_Control_Method Control_Method; // ������Ʒ�ʽ
    float Target_Angle;         // Ŀ��ĽǶ�
    float Target_Omega;         // Ŀ����ٶ�
    float Target_Current;       // Ŀ��ĵĵ���
    float Target_Voltage;       // Ŀ��ĵĵ�ѹ
    float Feedforward_Omega;    // ǰ�����ٶ�, rad/s
    float Feedforward_Current;  // ǰ���ĵ���, A
    float Feedforward_Voltage;  // ǰ���ĵ�ѹ, V
    float Out;                  // �����

    //���ʿ������
    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;
    float Power_Factor;         // ����˥������
    float Power_Estimate;       // ��һʱ�̵Ĺ��ʹ���ֵ, W
    float Power_K_0;            // GM6020���ʼ���ϵ��
    float Power_K_1;
    float Power_K_2;
    float Power_A;

} Motor_GM6020;

/**
 * @brief C610��ˢ���, �Դ�Ť�ػ�, ��Ƭ���������Ť��
 *
 */
typedef struct {
    // PID�ǶȻ�����
    PID_control PID_Angle;
    // PID���ٶȻ�����
    PID_control PID_Omega;

    // ��ʼ����ر���
    Struct_CAN_Manage_Object *CAN_Manage_Object; // �󶨵�CAN
    Enum_CAN_Motor_ID CAN_ID; // �����ݰ󶨵�CAN ID
    uint8_t *CAN_Tx_Data; // ���ͻ�����
    float Gearbox_Rate; // ���ٱ�
    float Torque_Max; // ���Ť��
    float Current_Max;

    // ����
    uint16_t Encoder_Num_Per_Round; // һȦ�������̶�
    uint16_t Output_Max; // ������Ť��

    // �ڲ�����
    uint32_t Flag; // ��ǰʱ�̵ĵ������flag
    uint32_t Pre_Flag; // ǰһʱ�̵ĵ������flag

    uint16_t Pre_Encoder; // ֮ǰ�ı�����λ��
    int32_t Total_Encoder; // �ܱ�����λ��
    int32_t Total_Round; // ��Ȧ��

    // ������
    Enum_CAN_Motor_Status CAN_Motor_Status; // ���״̬

    // �������ӿ���Ϣ
    Struct_Motor_DJI_Rx_Data Rx_Data;       //���������
    Struct_Motor_DJI_CAN_Rx_Data Rx_Origin; //CAN����ԭʼ����

    // д����
    Enum_Control_Method Control_Method; // ������Ʒ�ʽ
    float Target_Angle; // Ŀ��ĽǶ�
    float Target_Omega; // Ŀ����ٶ�
    float Target_Current; // Ŀ���Ť��
    float Feedforward_Omega;// ǰ�����ٶ�, rad/s
    float Feedforward_Current;// ǰ���ĵ���, A
    float Out; // �����

    float Power_K_0;
    float Power_K_1;
    float Power_K_2;
    float Power_A;

} Motor_C610;

/**
 * @brief C620��ˢ���, �Դ�Ť�ػ�, ��Ƭ���������Ť��
 *
 */
typedef struct {
    // PID�ǶȻ�����
    PID_control PID_Angle;
    // PID���ٶȻ�����
    PID_control PID_Omega;
	// PID���ٶȻ�����
	PID_control PID_Velocity;

    // ��ʼ����ر���
    Struct_CAN_Manage_Object *CAN_Manage_Object; // �󶨵�CAN
    Enum_CAN_Motor_ID CAN_ID; // �����ݰ󶨵�CAN ID
    uint8_t *CAN_Tx_Data; // ���ͻ�����
    float Gearbox_Rate; // ���ٱ�
    float Torque_Max; // ���Ť��
    float Current_Max;

    // ����
    uint16_t Encoder_Num_Per_Round; // һȦ�������̶�
    uint16_t Output_Max; // ������Ť��

    // �ڲ�����
    uint32_t Flag; // ��ǰʱ�̵ĵ������flag
    uint32_t Pre_Flag; // ǰһʱ�̵ĵ������flag

    // ������
    Enum_CAN_Motor_Status CAN_Motor_Status; // ���״̬
    Struct_Motor_DJI_Rx_Data Rx_Data;       //���������
    Struct_Motor_DJI_CAN_Rx_Data Rx_Origin; //CAN����ԭʼ����

    // д����
    Enum_Control_Method Control_Method; // ������Ʒ�ʽ
    float Target_Angle; 	// Ŀ��ĽǶ�
    float Target_Omega; 	// Ŀ��Ľ��ٶ� //��λ rad/s
	float Target_Velocity;  // Ŀ������ٶ� //��λ m/s
    float Target_Current; 	// Ŀ���Ť��
    float Feedforward_Omega;    // ǰ�����ٶ�, rad/s
    float Feedforward_Current;    // ǰ���ĵ���, A
    float Out; 				// �����

    //���ʿ������
    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;
    float Power_Factor;         // ����˥������
    float Power_Estimate;       // ��һʱ�̵Ĺ��ʹ���ֵ, W
    float Power_K_0;            // C620���ʼ���ϵ��
    float Power_K_1;
    float Power_K_2;
    float Power_A;

} Motor_C620;

/* Exported function declarations --------------------------------------------*/
float power_calculate(float K_0, float K_1, float K_2, float A, float Current, float Omega);
void Motor_GM6020_Init(Motor_GM6020 *motor, FDCAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, int32_t __Encoder_Offset, Enum_Motor_DJI_Power_Limit_Status __Power_Limit_Status, Enum_Motor_DJI_GM6020_Driver_Version __Driver_Version, float __Voltage_Max, float __Current_Max);
void Motor_GM6020_Output(Motor_GM6020 *motor);
void Motor_GM6020_Set_Out(Motor_GM6020 *motor, float __Out);
void Motor_GM6020_TIM_Calculate_PeriodElapsedCallback(Motor_GM6020 *motor);
void Motor_GM6020_CAN_RxCpltCallback(Motor_GM6020 *motor, uint8_t *Rx_Data);
void Motor_GM6020_TIM_Alive_PeriodElapsedCallback(Motor_GM6020 *motor);
void Motor_GM6020_TIM_PID_PeriodElapsedCallback(Motor_GM6020 *motor);
void Motor_DJI_GM6020_Power_Limit_Control(Motor_GM6020 *motor);
void Motor_DJI_GM6020_TIM_Power_Limit_After_Calculate_PeriodElapsedCallback(Motor_GM6020 *motor);

void Motor_C610_Init(Motor_C610 *motor, FDCAN_HandleTypeDef *__hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method);
void Motor_C610_Output(Motor_C610 *motor);
void Motor_C610_Set_Out(Motor_C610 *motor, float __Out);
void Motor_C610_CAN_RxCpltCallback(Motor_C610 *motor, uint8_t *Rx_Data);
void Motor_C610_TIM_Alive_PeriodElapsedCallback(Motor_C610 *motor);
void Motor_C610_TIM_Calculate_PeriodElapsedCallback(Motor_C610 *motor);
void Motor_C610_TIM_PID_PeriodElapsedCallback(Motor_C610 *motor);

void Motor_C620_Init(Motor_C620 *motor, FDCAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, Enum_Motor_DJI_Power_Limit_Status __Power_Limit_Status, float __Current_Max);
void Motor_C620_Output(Motor_C620 *motor);
void Motor_C620_Set_Out(Motor_C620 *motor, float __Out);
void Motor_C620_CAN_RxCpltCallback(Motor_C620 *motor, uint8_t *Rx_Data);
void Motor_C620_TIM_Alive_PeriodElapsedCallback(Motor_C620 *motor);
void Motor_C620_TIM_Calculate_PeriodElapsedCallback(Motor_C620 *motor);
void Motor_C620_TIM_PID_PeriodElapsedCallback(Motor_C620 *motor);
void Motor_DJI_C620_Power_Limit_Control(Motor_C620 *motor);
void Motor_DJI_C620_TIM_Power_Limit_After_Calculate_PeriodElapsedCallback(Motor_C620 *motor);

#endif
