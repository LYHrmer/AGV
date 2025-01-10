/**
 * @file motor_dm.c
 * @author CHR
 * @brief 达妙CAN电机配置与操作
 * @version 0.1
 * @date 2024-11-04
 *
 *
 */
 
/* Includes ------------------------------------------------------------------*/
 
#include "motor_dm.h"

// 清除电机错误信息, 传统模式有效
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xfb};
// 使能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xfc};
// 失能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Exit[8] = {0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xff,
                                        0xfd};
// 保存当前电机位置为零点, 传统模式有效
uint8_t DM_Motor_CAN_Message_Save_Zero[8] = {0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xff,
                                             0xfe};

											
/**
 * @brief 分配CAN发送缓冲区, 一拖四模式有效
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data(CAN_HandleTypeDef *hcan, Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID_1_To_4)
{
    uint8_t *tmp_tx_data_ptr = NULL;
    if (hcan == &hcan1)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case Motor_DM_ID_0x301:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x302:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x303:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x304:
            tmp_tx_data_ptr = &(CAN1_0x3fe_Tx_Data[6]);
            break;
        case Motor_DM_ID_0x305:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x306:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x307:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x308:
            tmp_tx_data_ptr = &(CAN1_0x4fe_Tx_Data[6]);
            break;
        }
    }
	
	else if (hcan == &hcan2)
    {
        switch (__CAN_Rx_ID_1_To_4)
        {
        case Motor_DM_ID_0x301:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x302:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x303:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x304:
            tmp_tx_data_ptr = &(CAN2_0x3fe_Tx_Data[6]);
            break;
        case Motor_DM_ID_0x305:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[0]);
            break;
        case Motor_DM_ID_0x306:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[2]);
            break;
        case Motor_DM_ID_0x307:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[4]);
            break;
        case Motor_DM_ID_0x308:
            tmp_tx_data_ptr = &(CAN2_0x4fe_Tx_Data[6]);
            break;
        }
    
	

    }
    return tmp_tx_data_ptr;
}
											 
/**
 * @brief 电机初始化
 *
 * @param motor 电机实例
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致, 传统模式有效
 * @param __CAN_Tx_ID 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param __Motor_DM_Control_Method 电机控制方式
 * @param __Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param __Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param __Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Motor_DM_Normal_Init(Motor_DM_Normal *motor, CAN_HandleTypeDef *hcan, uint8_t __CAN_Rx_ID, uint8_t __CAN_Tx_ID, Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, float __Angle_Max, float __Omega_Max, float __Torque_Max, float __Current_Max)
{
    if (hcan->Instance == CAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
//	else if (hcan->Instance == FDCAN3)
//    {
//        motor->CAN_Manage_Object = &CAN3_Manage_Object;
//    }

    motor->CAN_Rx_ID = __CAN_Rx_ID;
    switch (__Motor_DM_Control_Method)
    {
    case Motor_DM_Control_Method_NORMAL_MIT:
        motor->CAN_Tx_ID = __CAN_Tx_ID;
        break;
    case Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA:
        motor->CAN_Tx_ID = __CAN_Tx_ID + 0x100;
        break;
    case Motor_DM_Control_Method_NORMAL_OMEGA:
        motor->CAN_Tx_ID = __CAN_Tx_ID + 0x200;
        break;
    case Motor_DM_Control_Method_NORMAL_EMIT:
        motor->CAN_Tx_ID = __CAN_Tx_ID + 0x300;
        break;
    }
    motor->Motor_DM_Control_Method = __Motor_DM_Control_Method;
    motor->Angle_Max = __Angle_Max;
    motor->Omega_Max = __Omega_Max;
    motor->Torque_Max = __Torque_Max;
    motor->Current_Max = __Current_Max;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param motor 电机实例
 * @param Rx_Data 接收的数据
 */
void Motor_DM_Normal_CAN_RxCpltCallback(Motor_DM_Normal *motor, uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    motor->Flag += 1;
    Motor_DM_Normal_Data_Process(motor);
}

/**
 * @brief 发送清除错误信息
 *
 * @param motor 电机实例
 */
void Motor_DM_Normal_CAN_Send_Clear_Error(Motor_DM_Normal *motor)
{
    CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, DM_Motor_CAN_Message_Clear_Error, 8);
}

/**
 * @brief 发送使能电机
 *
 * @param motor 电机实例
 */
void Motor_DM_Normal_CAN_Send_Enable(Motor_DM_Normal *motor)
{
    CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, DM_Motor_CAN_Message_Enter, 8);
}

/**
 * @brief 发送失能电机
 *
 * @param motor 电机实例
 */
void Motor_DM_Normal_CAN_Send_Disable(Motor_DM_Normal *motor)
{
    CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, DM_Motor_CAN_Message_Exit, 8);
}

/**
 * @brief 发送保存当前位置为零点
 *
 * @param motor 电机实例
 */
void Motor_DM_Normal_CAN_Send_Save_Zero(Motor_DM_Normal *motor)
{
    CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, DM_Motor_CAN_Message_Save_Zero, 8);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活, 检测周期取决于电机掉线时长
 *
 * @param motor 电机实例
 */
void Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(Motor_DM_Normal *motor)
{
    // 判断该时间段内是否接收过电机数据
    if (motor->Flag == motor->Pre_Flag)
    {
        // 电机断开连接
        motor->Motor_DM_Status = Motor_DM_Status_DISABLE;
    }
    else
    {
        // 电机保持连接
        motor->Motor_DM_Status = Motor_DM_Status_ENABLE;
    }

    motor->Pre_Flag = motor->Flag;

    if (motor->Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        Motor_DM_Normal_CAN_Send_Enable(motor);
    }
}

/**
 * @brief TIM定时器中断发送出去的回调函数, 计算周期取决于自主设置的控制周期
 *
 * @param motor 电机实例
 */
void Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(Motor_DM_Normal *motor)
{
    if (motor->Rx_Data.Control_Status == Motor_DM_Status_ENABLE)
    {
        // 电机在线, 正常控制
        Math_Constrain(&motor->Control_Angle, -motor->Angle_Max, motor->Angle_Max);
        Math_Constrain(&motor->Control_Omega, -motor->Omega_Max, motor->Omega_Max);
        Math_Constrain(&motor->Control_Torque, -motor->Torque_Max, motor->Torque_Max);
        Math_Constrain(&motor->Control_Current, -motor->Current_Max, motor->Current_Max);
        Math_Constrain(&motor->K_P, 0.0f, 500.0f);
        Math_Constrain(&motor->K_D, 0.0f, 5.0f);

        Motor_DM_Normal_Output(motor);
    }
    else if (motor->Rx_Data.Control_Status == Motor_DM_Status_DISABLE)
    {
        // 电机可能掉线, 使能电机
        Motor_DM_Normal_CAN_Send_Enable(motor);
    }
    else
    {
        // 电机错误, 发送清除错误帧
        Motor_DM_Normal_CAN_Send_Clear_Error(motor);
    }
}

/**
 * @brief 数据处理过程
 *
 * @param motor 电机实例
 */
void Motor_DM_Normal_Data_Process(Motor_DM_Normal *motor)
{
    // 数据处理过程
    int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;
    Struct_Motor_DM_CAN_Rx_Data_Normal *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_Normal *)motor->CAN_Manage_Object->Rx_Buffer.Data;

    // 电机ID不匹配, 则不进行处理
    if (tmp_buffer->CAN_ID != (motor->CAN_Tx_ID & 0x0f))
    {
        return;
    }

    // 处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Angle_Reverse, &tmp_encoder);
    tmp_omega = (tmp_buffer->Omega_11_4 << 4) | (tmp_buffer->Omega_3_0_Torque_11_8 >> 4);
    tmp_torque = ((tmp_buffer->Omega_3_0_Torque_11_8 & 0x0f) << 8) | (tmp_buffer->Torque_7_0);

    motor->Rx_Data.Control_Status = (Enum_Motor_DM_Control_Status_Normal)tmp_buffer->Control_Status_Enum;

    // 计算圈数与总角度值
    delta_encoder = tmp_encoder - motor->Rx_Data.Pre_Encoder;
    if (delta_encoder < -(1 << 15))
    {
        // 正方向转过了一圈
        motor->Rx_Data.Total_Round++;
    }
    else if (delta_encoder > (1 << 15))
    {
        // 反方向转过了一圈
        motor->Rx_Data.Total_Round--;
    }
    motor->Rx_Data.Total_Encoder = motor->Rx_Data.Total_Round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);

		
		    // 计算电机本身信息
    static float now_angle=0;
    now_angle = (float) (motor->Rx_Data.Total_Encoder) / (float) ((1 << 16) - 1) * motor->Angle_Max * 2.0f;
    if(now_angle < -12.56637061) now_angle+=25;
    motor->Rx_Data.Now_Angle=now_angle;
    // 计算电机本身信息
    //motor->Rx_Data.Now_Angle = (float)(motor->Rx_Data.Total_Encoder) / (float)((1 << 16) - 1) * motor->Angle_Max * 2.0f;
		
		
		
    motor->Rx_Data.Now_Omega = Math_Int_To_Float(tmp_omega, 0x7ff, (1 << 12) - 1, 0, motor->Omega_Max);
    motor->Rx_Data.Now_Torque = Math_Int_To_Float(tmp_torque, 0x7ff, (1 << 12) - 1, 0, motor->Torque_Max);
    motor->Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    motor->Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    motor->Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 电机数据输出到CAN总线
 *
 */
void Motor_DM_Normal_Output(Motor_DM_Normal *motor)
{
    // 电机控制
    switch (motor->Motor_DM_Control_Method)
    {
    case Motor_DM_Control_Method_NORMAL_MIT:
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_MIT *)motor->Tx_Data;

        uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;

        tmp_angle = Math_Float_To_Int(motor->Control_Angle, 0, motor->Angle_Max, 0x7fff, (1 << 16) - 1);
        tmp_omega = Math_Float_To_Int(motor->Control_Omega, 0, motor->Omega_Max, 0x7ff, (1 << 12) - 1);
        tmp_torque = Math_Float_To_Int(motor->Control_Torque, 0, motor->Torque_Max, 0x7ff, (1 << 12) - 1);
        tmp_k_p = Math_Float_To_Int(motor->K_P, 0, 500.0f, 0, (1 << 12) - 1);
        tmp_k_d = Math_Float_To_Int(motor->K_D, 0, 5.0f, 0, (1 << 12) - 1);

        tmp_buffer->Control_Angle_Reverse = Math_Endian_Reverse_16(&tmp_angle, NULL);
        tmp_buffer->Control_Omega_11_4 = tmp_omega >> 4;
        tmp_buffer->Control_Omega_3_0_K_P_11_8 = ((tmp_omega & 0x0f) << 4) | (tmp_k_p >> 8);
        tmp_buffer->K_P_7_0 = tmp_k_p & 0xff;
        tmp_buffer->K_D_11_4 = tmp_k_d >> 4;
        tmp_buffer->K_D_3_0_Control_Torque_11_8 = ((tmp_k_d & 0x0f) << 4) | (tmp_torque >> 8);
        tmp_buffer->Control_Torque_7_0 = tmp_torque & 0xff;

        CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, motor->Tx_Data, 8);
        break;
    }
    case Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA:
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Angle_Omega *)motor->Tx_Data;

        tmp_buffer->Control_Angle = motor->Control_Angle;
        tmp_buffer->Control_Omega = motor->Control_Omega;

        CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, motor->Tx_Data, 8);
        break;
    }
    case Motor_DM_Control_Method_NORMAL_OMEGA:
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_Omega *)motor->Tx_Data;

        tmp_buffer->Control_Omega = motor->Control_Omega;

        CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, motor->Tx_Data, 4);
        break;
    }
    case Motor_DM_Control_Method_NORMAL_EMIT:
    {
        Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *tmp_buffer = (Struct_Motor_DM_CAN_Tx_Data_Normal_EMIT *)motor->Tx_Data;

        tmp_buffer->Control_Angle = motor->Control_Angle;
        tmp_buffer->Control_Omega = (uint16_t)(motor->Control_Omega * 100.0f);
        tmp_buffer->Control_Current = (uint16_t)(motor->Control_Current / motor->Current_Max * 10000.0f);

        CAN_Send_Data(motor->CAN_Manage_Object->CAN_Handler, motor->CAN_Tx_ID, motor->Tx_Data, 8);
        break;
    }
    }
}

/**
 * @brief 电机初始化
 * 
 * @param motor 电机实例
 * @param hcan 绑定的CAN总线
 * @param __CAN_Rx_ID 绑定的CAN ID
 * @param __Motor_DM_Control_Method 电机控制方式, 默认角度
 * @param __Encoder_Offset 编码器偏移, 默认0
 * @param __Current_Max 最大电流
 */
void Motor_DM_1_To_4_Init(Motor_DM_1_To_4 *motor, CAN_HandleTypeDef *hcan, Enum_Motor_DM_Motor_ID_1_To_4 __CAN_Rx_ID, Enum_Motor_DM_Control_Method __Motor_DM_Control_Method, int32_t __Encoder_Offset, float __Current_Max)
{
    if (hcan->Instance == CAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
    motor->CAN_Rx_ID = __CAN_Rx_ID;
    motor->Motor_DM_Control_Method = __Motor_DM_Control_Method;
    motor->Encoder_Offset = __Encoder_Offset;
    motor->Current_Max = __Current_Max;
    motor->Tx_Data = allocate_tx_data(hcan, __CAN_Rx_ID);
	motor->Encoder_Num_Per_Round = 8192;
	motor->Current_To_Out = 16384.0f / 10.261194f;
	motor->Theoretical_Output_Current_Max = 10.261194f;
}

/**
 * @brief CAN通信接收回调函数
 *
 * @param motor 电机实例
 * @param Rx_Data 接收的数据
 */
void Motor_DM_1_To_4_CAN_RxCpltCallback(Motor_DM_1_To_4 *motor, uint8_t *Rx_Data)
{
    // 滑动窗口, 判断电机是否在线
    motor->Flag += 1;
    Motor_DM_1_To_4_Data_Process(motor);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 * @param motor 电机实例
 */
void Motor_DM_1_To_4_TIM_100ms_Alive_PeriodElapsedCallback(Motor_DM_1_To_4 *motor)
{
    // 判断该时间段内是否接收过电机数据
    if (motor->Flag == motor->Pre_Flag)
    {
        // 电机断开连接
        motor->Motor_DM_Status = Motor_DM_Status_DISABLE;
//        PID_Angle_Set_Integral_Error(&motor->PID_Angle, 0.0f);
//        PID_Omega_Set_Integral_Error(&motor->PID_Omega, 0.0f);
    }
    else
    {
        // 电机保持连接
        motor->Motor_DM_Status = Motor_DM_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

/**
 * @brief TIM定时器中断计算回调函数, 计算周期取决于电机反馈周期
 *
 * @param motor 电机实例
 */
void Motor_DM_1_To_4_TIM_1ms_Calculate_PeriodElapsedCallback(Motor_DM_1_To_4 *motor)
{
    Motor_DM_1_To_4_PID_Calculate(motor);

    float tmp_value = motor->Target_Current + motor->Feedforward_Current;
    Math_Constrain(&tmp_value, -motor->Current_Max, motor->Current_Max);
    motor->Out = tmp_value * motor->Current_To_Out;

    Motor_DM_1_To_4_Output(motor);

    motor->Feedforward_Current = 0.0f;
    motor->Feedforward_Omega = 0.0f;
}

/**
 * @brief 数据处理过程
 *
 * @param motor 电机实例
 */
void Motor_DM_1_To_4_Data_Process(Motor_DM_1_To_4 *motor)
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DM_CAN_Rx_Data_1_To_4 *tmp_buffer = (Struct_Motor_DM_CAN_Rx_Data_1_To_4 *)motor->CAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端
    Math_Endian_Reverse_16((void *)&tmp_buffer->Encoder_Reverse, (void *)&tmp_encoder);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Omega_Reverse, (void *)&tmp_omega);
    Math_Endian_Reverse_16((void *)&tmp_buffer->Current_Reverse, (void *)&tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - motor->Rx_Data.Pre_Encoder;
    if (delta_encoder < -motor->Encoder_Num_Per_Round / 2)
    {
        // 正方向转过了一圈
        motor->Rx_Data.Total_Round++;
    }
    else if (delta_encoder > motor->Encoder_Num_Per_Round / 2)
    {
        // 反方向转过了一圈
        motor->Rx_Data.Total_Round--;
    }
    motor->Rx_Data.Total_Encoder = motor->Rx_Data.Total_Round * motor->Encoder_Num_Per_Round + tmp_encoder + motor->Encoder_Offset;

    // 计算电机本身信息
    motor->Rx_Data.Now_Angle = (float)motor->Rx_Data.Total_Encoder / (float)motor->Encoder_Num_Per_Round * 2.0f * PI;
    motor->Rx_Data.Now_Omega = tmp_omega / 100.0f * RPM_TO_RADPS;
    motor->Rx_Data.Now_Current = tmp_current / 1000.0f;
    motor->Rx_Data.Now_MOS_Temperature = tmp_buffer->MOS_Temperature + CELSIUS_TO_KELVIN;
    motor->Rx_Data.Now_Rotor_Temperature = tmp_buffer->Rotor_Temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    motor->Rx_Data.Pre_Encoder = tmp_encoder;
}

/**
 * @brief 计算PID
 *
 * @param motor 电机实例
 */
void Motor_DM_1_To_4_PID_Calculate(Motor_DM_1_To_4 *motor)
{
    switch (motor->Motor_DM_Control_Method)
    {
    case (Motor_DM_Control_Method_1_TO_4_CURRENT):
        break;
    case (Motor_DM_Control_Method_1_TO_4_OMEGA):
	{
		motor->Omega_PID.Target = motor->Target_Omega + motor->Feedforward_Omega;
		motor->Omega_PID.Now = motor->Rx_Data.Now_Omega;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->Omega_PID);
		
		motor->Target_Current = motor->Omega_PID.Out;
        break;
	}
    case (Motor_DM_Control_Method_1_TO_4_ANGLE):
	{
		motor->Angle_PID.Target = motor->Target_Angle;
		motor->Angle_PID.Now = motor->Rx_Data.Now_Angle;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->Angle_PID);
		
		motor->Target_Omega = motor->Angle_PID.Out;
		
		motor->Omega_PID.Target = motor->Target_Omega + motor->Feedforward_Omega;
		motor->Omega_PID.Now = motor->Rx_Data.Now_Omega;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->Omega_PID);
		
		motor->Target_Current = motor->Omega_PID.Out;
		
        break;
	}
    default:
        motor->Target_Current = 0.0f;
        break;
    }
}

/**
 * @brief 电机数据输出到CAN总线发送缓冲区
 *
 * @param motor 电机实例
 */
void Motor_DM_1_To_4_Output(Motor_DM_1_To_4 *motor)
{
    *(int16_t *)motor->Tx_Data = (int16_t)(motor->Out);
}
