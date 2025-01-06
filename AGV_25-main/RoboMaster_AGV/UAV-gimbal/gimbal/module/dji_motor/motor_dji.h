/**
 * @file motor_dji.h
 * @author CHR
 * @brief 大疆CAN电机配置与操作
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

// RPM换算到rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)

// 电压到输出的转化系数
#define GM6020_Voltage_To_Out (25000.0f / 24.0f)
// 电流到输出的转化系数
#define GM6020_Current_To_Out (16384.0f / 3.0f)
// 理论最大输出电压
#define GM6020_Theoretical_Output_Voltage_Max 24.0f;
// 理论最大输出电流
#define GM6020_Theoretical_Output_Current_Max 3.0f;

// 电流到输出的转化系数
#define C610_Current_To_Out (10000.0f / 10.0f)

// 电流到输出的转化系数
#define C620_Current_To_Out (16384.0f / 20.0f)
// 理论最大输出电流
#define C620_Theoretical_Output_Current_Max = 20.0f;


/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机状态
 *
 */
typedef enum {
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE,
} Enum_CAN_Motor_Status;

/**
 * @brief 是否开启功率控制, 此时电机须电流作为输出模式, 不可电压控制
 *
 */
typedef enum
{
    Motor_DJI_Power_Limit_Status_DISABLE = 0,
    Motor_DJI_Power_Limit_Status_ENABLE,
}Enum_Motor_DJI_Power_Limit_Status;

/**
 * @brief 大疆电机驱动版本, 影响GM6020电机驱动方式
 *
 */
typedef enum
{
    Motor_DJI_GM6020_Driver_Version_DEFAULT = 0,
    Motor_DJI_GM6020_Driver_Version_2023,
}Enum_Motor_DJI_GM6020_Driver_Version;

/**
 * @brief 大疆电机源数据
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
 * @brief 大疆电机经过处理的数据
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
 * @brief CAN电机的ID枚举类型
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
 * @brief CAN电机的ID分配情况
 *
 */
typedef enum {
    CAN_Motor_ID_Status_FREE = 0,
    CAN_Motor_ID_Status_ALLOCATED,
} Enum_CAN_Motor_ID_Status;

/**
 * @brief 电机控制方式
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
 * @brief GM6020无刷电机, 单片机控制输出电压
 *
 */
typedef struct {
    // PID角度环控制
    PID_control PID_Angle;
    // PID角速度环控制
    PID_control PID_Omega;
    // PID扭矩环控制
    PID_control PID_Current;

    // 初始化相关变量
    Enum_Motor_DJI_GM6020_Driver_Version Driver_Version;
    Struct_CAN_Manage_Object *CAN_Manage_Object; // 绑定的CAN
    Enum_CAN_Motor_ID CAN_ID; // 收数据绑定的CAN ID
    uint8_t *CAN_Tx_Data; // 发送缓存区
    uint32_t Encoder_Offset; // 编码器偏移
    float Omega_Max; // 最大速度
    float Voltage_Max;
    float Current_Max;

    // 常量
    uint16_t Encoder_Num_Per_Round; // 一圈编码器刻度
    uint16_t Output_Max; // 最大输出电压

    // 内部变量
    uint32_t Flag; // 当前时刻的电机接收flag
    uint32_t Pre_Flag; // 前一时刻的电机接收flag

    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;   //结算后数据
    Struct_Motor_DJI_CAN_Rx_Data Rx_Origin; //CAN接收原始数据

    // 读变量
    Enum_CAN_Motor_Status CAN_Motor_Status; // 电机状态

    // 写变量
    Enum_Control_Method Control_Method; // 电机控制方式
    float Target_Angle;         // 目标的角度
    float Target_Omega;         // 目标的速度
    float Target_Current;       // 目标的的电流
    float Target_Voltage;       // 目标的的电压
    float Feedforward_Omega;    // 前馈的速度, rad/s
    float Feedforward_Current;  // 前馈的电流, A
    float Feedforward_Voltage;  // 前馈的电压, V
    float Out;                  // 输出量

    //功率控制相关
    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;
    float Power_Factor;         // 功率衰减因数
    float Power_Estimate;       // 下一时刻的功率估计值, W
    float Power_K_0;            // GM6020功率计算系数
    float Power_K_1;
    float Power_K_2;
    float Power_A;

} Motor_GM6020;

/**
 * @brief C610无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
typedef struct {
    // PID角度环控制
    PID_control PID_Angle;
    // PID角速度环控制
    PID_control PID_Omega;

    // 初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object; // 绑定的CAN
    Enum_CAN_Motor_ID CAN_ID; // 收数据绑定的CAN ID
    uint8_t *CAN_Tx_Data; // 发送缓存区
    float Gearbox_Rate; // 减速比
    float Torque_Max; // 最大扭矩
    float Current_Max;

    // 常量
    uint16_t Encoder_Num_Per_Round; // 一圈编码器刻度
    uint16_t Output_Max; // 最大输出扭矩

    // 内部变量
    uint32_t Flag; // 当前时刻的电机接收flag
    uint32_t Pre_Flag; // 前一时刻的电机接收flag

    uint16_t Pre_Encoder; // 之前的编码器位置
    int32_t Total_Encoder; // 总编码器位置
    int32_t Total_Round; // 总圈数

    // 读变量
    Enum_CAN_Motor_Status CAN_Motor_Status; // 电机状态

    // 电机对外接口信息
    Struct_Motor_DJI_Rx_Data Rx_Data;       //结算后数据
    Struct_Motor_DJI_CAN_Rx_Data Rx_Origin; //CAN接收原始数据

    // 写变量
    Enum_Control_Method Control_Method; // 电机控制方式
    float Target_Angle; // 目标的角度
    float Target_Omega; // 目标的速度
    float Target_Current; // 目标的扭矩
    float Feedforward_Omega;// 前馈的速度, rad/s
    float Feedforward_Current;// 前馈的电流, A
    float Out; // 输出量

    float Power_K_0;
    float Power_K_1;
    float Power_K_2;
    float Power_A;

} Motor_C610;

/**
 * @brief C620无刷电调, 自带扭矩环, 单片机控制输出扭矩
 *
 */
typedef struct {
    // PID角度环控制
    PID_control PID_Angle;
    // PID角速度环控制
    PID_control PID_Omega;
	// PID线速度环控制
	PID_control PID_Velocity;

    // 初始化相关变量
    Struct_CAN_Manage_Object *CAN_Manage_Object; // 绑定的CAN
    Enum_CAN_Motor_ID CAN_ID; // 收数据绑定的CAN ID
    uint8_t *CAN_Tx_Data; // 发送缓存区
    float Gearbox_Rate; // 减速比
    float Torque_Max; // 最大扭矩
    float Current_Max;

    // 常量
    uint16_t Encoder_Num_Per_Round; // 一圈编码器刻度
    uint16_t Output_Max; // 最大输出扭矩

    // 内部变量
    uint32_t Flag; // 当前时刻的电机接收flag
    uint32_t Pre_Flag; // 前一时刻的电机接收flag

    // 读变量
    Enum_CAN_Motor_Status CAN_Motor_Status; // 电机状态
    Struct_Motor_DJI_Rx_Data Rx_Data;       //结算后数据
    Struct_Motor_DJI_CAN_Rx_Data Rx_Origin; //CAN接收原始数据

    // 写变量
    Enum_Control_Method Control_Method; // 电机控制方式
    float Target_Angle; 	// 目标的角度
    float Target_Omega; 	// 目标的角速度 //单位 rad/s
	float Target_Velocity;  // 目标的线速度 //单位 m/s
    float Target_Current; 	// 目标的扭矩
    float Feedforward_Omega;    // 前馈的速度, rad/s
    float Feedforward_Current;    // 前馈的电流, A
    float Out; 				// 输出量

    //功率控制相关
    Enum_Motor_DJI_Power_Limit_Status Power_Limit_Status;
    float Power_Factor;         // 功率衰减因数
    float Power_Estimate;       // 下一时刻的功率估计值, W
    float Power_K_0;            // C620功率计算系数
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
