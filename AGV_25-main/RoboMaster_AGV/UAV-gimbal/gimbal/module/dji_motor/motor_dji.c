/**
 * @file motor_dji.c
 * @author CHR
 * @brief 大疆CAN电机配置与操作
 * @version 0.1
 * @date 2024-11-04
 *
 *
 */
 
/* Includes ------------------------------------------------------------------*/
 
#include "motor_dji.h"

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param __CAN_ID CAN ID
 * @return uint8_t* 缓冲区指针
 */
uint8_t *allocate_tx_data_dji(FDCAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Motor_DJI_GM6020_Driver_Version __DJI_Motor_Driver_Version)
{
    uint8_t *tmp_tx_data_ptr = NULL;
    if (hcan == &hfdcan1)
    {
        switch (__CAN_ID)
        {
        case CAN_Motor_ID_0x201:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x202:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x203:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
            break;
        case CAN_Motor_ID_0x204:
            tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
            break;
        case CAN_Motor_ID_0x205:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[0]);
            }
            break;
        case CAN_Motor_ID_0x206:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[2]);
            }
            break;
        case CAN_Motor_ID_0x207:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[4]);
            }
            break;
        case CAN_Motor_ID_0x208:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x1ff_Tx_Data[6]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN1_0x1fe_Tx_Data[6]);
            }
            break;
        case CAN_Motor_ID_0x209:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[0]);
            }
            break;
        case CAN_Motor_ID_0x20A:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[2]);
            }
            break;
        case CAN_Motor_ID_0x20B:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN1_0x2ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN1_0x2fe_Tx_Data[4]);
            }
            break;
        }
    }
	else if (hcan == &hfdcan2)
    {
        switch (__CAN_ID)
        {
        case CAN_Motor_ID_0x201:
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
            break;
        case CAN_Motor_ID_0x202:
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
            break;
        case CAN_Motor_ID_0x203:
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
            break;                 
        case CAN_Motor_ID_0x204:   
            tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
            break;                 
        case CAN_Motor_ID_0x205:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[0]);
            }
            break;                 
        case CAN_Motor_ID_0x206:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[2]);
            }
            break;                 
        case CAN_Motor_ID_0x207:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[4]);
            }
            break;                 
        case CAN_Motor_ID_0x208:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x1ff_Tx_Data[6]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN2_0x1fe_Tx_Data[6]);
            }
            break;                 
        case CAN_Motor_ID_0x209:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[0]);
            }
            break;                 
        case CAN_Motor_ID_0x20A:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[2]);
            }
            break;
        case CAN_Motor_ID_0x20B:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN2_0x2ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN2_0x2fe_Tx_Data[4]);
            }
            break;
        }
    }
	else if (hcan == &hfdcan3)
    {
        switch (__CAN_ID)
        {
        case CAN_Motor_ID_0x201:
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[0]);
            break;                 
        case CAN_Motor_ID_0x202:   
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[2]);
            break;                 
        case CAN_Motor_ID_0x203:   
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[4]);
            break;                 
        case CAN_Motor_ID_0x204:   
            tmp_tx_data_ptr = &(CAN3_0x200_Tx_Data[6]);
            break;
        case CAN_Motor_ID_0x205:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[0]);
            }
            break;
        case CAN_Motor_ID_0x206:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[2]);
            }
            break;
        case CAN_Motor_ID_0x207:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[4]);
            }
            break;
        case CAN_Motor_ID_0x208:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x1ff_Tx_Data[6]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN3_0x1fe_Tx_Data[6]);
            }
            break;
        case CAN_Motor_ID_0x209:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[0]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[0]);
            }
            break;
        case CAN_Motor_ID_0x20A:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[2]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[2]);
            }
            break;
        case CAN_Motor_ID_0x20B:
            if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
            {
                tmp_tx_data_ptr = &(CAN3_0x2ff_Tx_Data[4]);
            }
            else if (__DJI_Motor_Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
            {
                tmp_tx_data_ptr = &(CAN3_0x2fe_Tx_Data[4]);
            }
            break;
        }
    }
    return tmp_tx_data_ptr;
}

float power_calculate(float K_0, float K_1, float K_2, float A, float Current, float Omega)
{
    return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
}

void Motor_GM6020_Init(Motor_GM6020 *motor, FDCAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, int32_t __Encoder_Offset, Enum_Motor_DJI_Power_Limit_Status __Power_Limit_Status, Enum_Motor_DJI_GM6020_Driver_Version __Driver_Version, float __Voltage_Max, float __Current_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
	else if (hcan->Instance == FDCAN3)
    {
        motor->CAN_Manage_Object = &CAN3_Manage_Object;
    }
	
    motor->CAN_ID = __CAN_ID;
    motor->Control_Method = __Control_Method;
    motor->Encoder_Offset = __Encoder_Offset;
    motor->Power_Limit_Status = __Power_Limit_Status;
    motor->Voltage_Max = __Voltage_Max;
    motor->Current_Max = __Current_Max;
    motor->Driver_Version = __Driver_Version;
    motor->CAN_Tx_Data = allocate_tx_data_dji(hcan, __CAN_ID, __Driver_Version);
	motor->Encoder_Num_Per_Round = 8192;
	motor->Output_Max = 30000;
    motor->Power_K_0 = 0.8130f;
    motor->Power_K_1 = -0.0005f;
    motor->Power_K_2 = 6.0021f;
    motor->Power_A = 1.3715f;
}

void Motor_GM6020_Output(Motor_GM6020 *motor)
{
    motor->CAN_Tx_Data[0] = (int16_t)(motor->Out) >> 8;
    motor->CAN_Tx_Data[1] = (int16_t)(motor->Out);
}

void Motor_GM6020_Set_Out(Motor_GM6020 *motor, float __Out)
{
    motor->Out = __Out;
}

void Motor_GM6020_CAN_RxCpltCallback(Motor_GM6020 *motor, uint8_t *Rx_Data)
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *) motor->CAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

    // 计算圈数与总编码器值
    delta_encoder = tmp_encoder - motor->Rx_Data.Pre_Encoder-12;
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

    //读取电机原始信息
    //motor->Rx_Origin.Last_Encoder_Reverse = motor->Rx_Origin.Encoder_Reverse;
    motor->Rx_Origin.Encoder_Reverse = tmp_buffer->Encoder_Reverse;
    motor->Rx_Origin.Omega_Reverse = tmp_buffer->Omega_Reverse;
    motor->Rx_Origin.Current_Reverse = tmp_buffer->Current_Reverse;
    motor->Rx_Origin.Temperature = tmp_buffer->Temperature;

    // 计算电机本身信息
    motor->Rx_Data.Now_Angle = (float) motor->Rx_Data.Total_Encoder / (float) motor->Encoder_Num_Per_Round * 2.0f * PI;
    motor->Rx_Data.Now_Omega = (float) tmp_omega * RPM_TO_RADPS;
    motor->Rx_Data.Now_Current = tmp_current / GM6020_Current_To_Out;
    motor->Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;
    motor->Rx_Data.Now_Power = power_calculate(motor->Power_K_0, motor->Power_K_1, motor->Power_K_2, motor->Power_A, motor->Rx_Data.Now_Current, motor->Rx_Data.Now_Omega);

    // 存储预备信息
    motor->Rx_Data.Pre_Encoder = tmp_encoder;
}

void Motor_GM6020_TIM_Alive_PeriodElapsedCallback(Motor_GM6020 *motor)
{
    if (motor->Flag == motor->Pre_Flag)
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        PID_Set_Integral_Error(&motor->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Omega, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Current, 0.0f);
    }
    else
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

void Motor_GM6020_TIM_Calculate_PeriodElapsedCallback(Motor_GM6020 *motor)
{
    Motor_GM6020_TIM_PID_PeriodElapsedCallback(motor);

    if(motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
    {
        float tmp_value = motor->Target_Voltage + motor->Feedforward_Voltage;
        Math_Constrain(&tmp_value, -motor->Voltage_Max, motor->Voltage_Max);
        motor->Out = tmp_value * GM6020_Voltage_To_Out;
    }

    else if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
    {
        float tmp_value = motor->Target_Current + motor->Feedforward_Current;
        Math_Constrain(&tmp_value, -motor->Current_Max, motor->Current_Max);
        motor->Out = tmp_value * GM6020_Current_To_Out;
    }

    Motor_GM6020_Output(motor);

    if(motor->Power_Limit_Status == Motor_DJI_Power_Limit_Status_DISABLE)
    {
        motor->Feedforward_Voltage = 0.0f;
        motor->Feedforward_Current = 0.0f;
        motor->Feedforward_Omega = 0.0f;
    }
}

void Motor_GM6020_TIM_PID_PeriodElapsedCallback(Motor_GM6020 *motor)
{
    switch (motor->Control_Method)
    {
    case Motor_DJI_Control_Method_VOLTAGE:
    {
        if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {

        }
        else if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            motor->Target_Voltage = 0.0f;
            motor->Target_Current = 0.0f;
        }
        break;
    }
    case Motor_DJI_Control_Method_CURRENT:
    {
        if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            motor->PID_Current.Target = motor->Target_Current + motor->Feedforward_Current;
            motor->PID_Current.Now = motor->Rx_Data.Now_Current;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Current);

            motor->Target_Voltage = motor->PID_Current.Out;
        }
        else if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {

        }
        break;
    }
    case Motor_DJI_Control_Method_OMEGA:
    {
        if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
            motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

            motor->Target_Current = motor->PID_Omega.Out;

            motor->PID_Current.Target = motor->Target_Current + motor->Feedforward_Current;
            motor->PID_Current.Now = motor->Rx_Data.Now_Current;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Current);

            motor->Target_Voltage = motor->PID_Current.Out;
        }
        else if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
            motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

            motor->Target_Current = motor->PID_Omega.Out;
        }
        break;
    }
    case Motor_DJI_Control_Method_ANGLE:
    {
        if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            motor->PID_Angle.Target = motor->Target_Angle;
            motor->PID_Angle.Now = motor->Rx_Data.Now_Angle;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);

            motor->Target_Omega = motor->PID_Angle.Out;

            motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
            motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

            motor->Target_Current = motor->PID_Omega.Out;

            motor->PID_Current.Target = motor->Target_Current + motor->Feedforward_Current;
            motor->PID_Current.Now = motor->Rx_Data.Now_Current;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Current);

            motor->Target_Voltage = motor->PID_Current.Out;
        }
        else if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            motor->PID_Angle.Target = motor->Target_Angle;
            motor->PID_Angle.Now = motor->Rx_Data.Now_Angle;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);

            motor->Target_Omega = motor->PID_Angle.Out;

            motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
            motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
            PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

            motor->Target_Current = motor->PID_Omega.Out;
        }
        break;
    }
    default:
        if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
        {
            motor->Target_Voltage = 0.0f;
        }
        else if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
        {
            motor->Target_Current = 0.0f;
        }
        break;
    }
}

void Motor_DJI_GM6020_Power_Limit_Control(Motor_GM6020 *motor)
{
    // 计算功率估计值
    motor->Power_Estimate = power_calculate(motor->Power_K_0, motor->Power_K_1, motor->Power_K_2, motor->Power_A, motor->Target_Current, motor->Rx_Data.Now_Omega);

    // 若功率为正则考虑功率控制限制
    if (motor->Power_Estimate > 0.0f)
    {
        if (motor->Power_Factor >= 1.0f)
        {
            // 无需功率控制
        }
        else
        {
            // 需要功率控制

            // 根据功率估计公式解一元二次方程求电流值
            float a = motor->Power_K_2;
            float b = motor->Power_K_0 * motor->Rx_Data.Now_Omega;
            float c = motor->Power_A + motor->Power_K_1 * motor->Rx_Data.Now_Omega * motor->Rx_Data.Now_Omega - motor->Power_Factor * motor->Power_Estimate;
            float delta, h;
            delta = b * b - 4 * a * c;
            if (delta < 0.0f)
            {
                // 无解
                motor->Target_Current = 0.0f;
            }
            else
            {
                arm_sqrt_f32(delta, &h);
                float result_1, result_2;
                result_1 = (-b + h) / (2.0f * a);
                result_2 = (-b - h) / (2.0f * a);

                // 两个潜在的可行电流值, 取绝对值最小的那个
                if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
                {
                    if ((motor->Target_Current > 0.0f && result_1 > 0.0f) || (motor->Target_Current < 0.0f && result_1 < 0.0f))
                    {
                        motor->Target_Current = result_1;
                    }
                    else
                    {
                        motor->Target_Current = result_2;
                    }
                }
                else
                {
                    if (Float_Math_Abs(result_1) < Float_Math_Abs(result_2))
                    {
                        motor->Target_Current = result_1;
                    }
                    else
                    {
                        motor->Target_Current = result_2;
                    }
                }
            }
        }
    }
}

void Motor_DJI_GM6020_TIM_Power_Limit_After_Calculate_PeriodElapsedCallback(Motor_GM6020 *motor)
{
    if (motor->Power_Limit_Status == Motor_DJI_Power_Limit_Status_ENABLE)
    {
        Motor_DJI_GM6020_Power_Limit_Control(motor);
    }

    if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_DEFAULT)
    {
        float tmp_value = motor->Target_Voltage + motor->Feedforward_Voltage;
        Math_Constrain(&tmp_value, -motor->Voltage_Max, motor->Voltage_Max);
        motor->Out = tmp_value * GM6020_Voltage_To_Out;
    }
    else if (motor->Driver_Version == Motor_DJI_GM6020_Driver_Version_2023)
    {
        float tmp_value = motor->Target_Current + motor->Feedforward_Current;
        Math_Constrain(&tmp_value, -motor->Current_Max, motor->Current_Max);
        motor->Out = tmp_value * GM6020_Current_To_Out;
    }

    Motor_GM6020_Output(motor);

    motor->Feedforward_Voltage = 0.0f;
    motor->Feedforward_Current = 0.0f;
    motor->Feedforward_Omega = 0.0f;
}

void Motor_C610_Init(Motor_C610 *motor, FDCAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method)
{
    if (hcan->Instance == FDCAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
	else if (hcan->Instance == FDCAN3)
    {
        motor->CAN_Manage_Object = &CAN3_Manage_Object;
    }
    motor->CAN_ID = __CAN_ID;
    motor->Control_Method = __Control_Method;
    motor->CAN_Tx_Data = allocate_tx_data_dji(hcan, __CAN_ID, Motor_DJI_GM6020_Driver_Version_DEFAULT);
	motor->Gearbox_Rate = 36.0f;
	motor->Current_Max = 10.0f;
	motor->Encoder_Num_Per_Round = 8192;
	motor->Output_Max = 10000;
}

void Motor_C610_Output(Motor_C610 *motor)
{
    motor->CAN_Tx_Data[0] = (int16_t)(motor->Out) >> 8;
    motor->CAN_Tx_Data[1] = (int16_t)(motor->Out);
}

void Motor_C610_Set_Out(Motor_C610 *motor, float __Out)
{
    motor->Out = __Out;
}

void Motor_C610_CAN_RxCpltCallback(Motor_C610 *motor, uint8_t *Rx_Data)
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *) motor->CAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

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
    motor->Rx_Data.Total_Encoder = motor->Rx_Data.Total_Round * motor->Encoder_Num_Per_Round + tmp_encoder;

    //读取电机原始信息
    //motor->Rx_Origin.Last_Encoder_Reverse = motor->Rx_Origin.Encoder_Reverse;
    motor->Rx_Origin.Encoder_Reverse = tmp_buffer->Encoder_Reverse;
    motor->Rx_Origin.Omega_Reverse = tmp_buffer->Omega_Reverse;
    motor->Rx_Origin.Current_Reverse = tmp_buffer->Current_Reverse;
    motor->Rx_Origin.Temperature = tmp_buffer->Temperature;

    // 计算电机本身信息
    motor->Rx_Data.Now_Angle = (float) motor->Rx_Data.Total_Encoder / (float) motor->Encoder_Num_Per_Round * 2.0f * PI / motor->Gearbox_Rate;
    motor->Rx_Data.Now_Omega = (float) tmp_omega * RPM_TO_RADPS / motor->Gearbox_Rate;
    motor->Rx_Data.Now_Current = tmp_current / C610_Current_To_Out;
    motor->Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;

    // 存储预备信息
    motor->Rx_Data.Pre_Encoder = tmp_encoder;
}

void Motor_C610_TIM_Alive_PeriodElapsedCallback(Motor_C610 *motor)
{
    if (motor->Flag == motor->Pre_Flag)
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
		PID_Set_Integral_Error(&motor->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Omega, 0.0f);
    }
    else
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

void Motor_C610_TIM_Calculate_PeriodElapsedCallback(Motor_C610 *motor)
{
    Motor_C610_TIM_PID_PeriodElapsedCallback(motor);

    float tmp_value = motor->Target_Current + motor->Feedforward_Current;
    Math_Constrain(&tmp_value, -motor->Current_Max, motor->Current_Max);
    motor->Out = tmp_value * C610_Current_To_Out;

    Motor_C610_Output(motor);

    motor->Feedforward_Current = 0.0f;
    motor->Feedforward_Omega = 0.0f;
}

void Motor_C610_TIM_PID_PeriodElapsedCallback(Motor_C610 *motor)
{
    switch (motor->Control_Method)
    {
    case Motor_DJI_Control_Method_CURRENT:

        break;
    case Motor_DJI_Control_Method_OMEGA:
		motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
		motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        motor->Target_Current = motor->PID_Omega.Out;
        break;
    case Motor_DJI_Control_Method_ANGLE:
		motor->PID_Angle.Target = motor->Target_Angle;
		motor->PID_Angle.Now = motor->Rx_Data.Now_Angle;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);
	
		motor->Target_Omega = motor->PID_Angle.Out;
	
		motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
		motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        motor->Target_Current = motor->PID_Omega.Out;
        break;
    default:
        Motor_C610_Set_Out(motor, 0.0f);
        break;
    }
}

/* C620 motor functions would follow a similar pattern as C610 */

void Motor_C620_Init(Motor_C620 *motor, FDCAN_HandleTypeDef *hcan, Enum_CAN_Motor_ID __CAN_ID, Enum_Control_Method __Control_Method, Enum_Motor_DJI_Power_Limit_Status __Power_Limit_Status, float __Current_Max)
{
    if (hcan->Instance == FDCAN1)
    {
        motor->CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        motor->CAN_Manage_Object = &CAN2_Manage_Object;
    }
	else if (hcan->Instance == FDCAN3)
    {
        motor->CAN_Manage_Object = &CAN3_Manage_Object;
    }
    motor->CAN_ID = __CAN_ID;
    motor->Control_Method = __Control_Method;
    motor->Power_Limit_Status = __Power_Limit_Status;
    motor->Current_Max = __Current_Max;
    motor->CAN_Tx_Data = allocate_tx_data_dji(hcan, __CAN_ID, Motor_DJI_GM6020_Driver_Version_DEFAULT);
	motor->Gearbox_Rate = 3591.0f / 187.0f;
	motor->Encoder_Num_Per_Round = 8192;
	motor->Output_Max = 16384;
    /*motor->Power_K_0 = 0.2962f;
    motor->Power_K_1 = 0.0000f;
    motor->Power_K_2 = 0.1519f;
    motor->Power_A = 1.3544f;*/

    /*motor->Power_K_0 = 0.2413f;
    motor->Power_K_1 = 0.0094f;
    motor->Power_K_2 = 0.1118f;
    motor->Power_A = 0.6951f;*/
    motor->Power_K_0 = 0.2421f;
    motor->Power_K_1 = 0.0093f;
    motor->Power_K_2 = 0.1118f;
    motor->Power_A = 0.6972f;
}

void Motor_C620_Output(Motor_C620 *motor)
{
    motor->CAN_Tx_Data[0] = (int16_t)(motor->Out) >> 8;
    motor->CAN_Tx_Data[1] = (int16_t)(motor->Out);
}

void Motor_C620_Set_Out(Motor_C620 *motor, float __Out)
{
    motor->Out = __Out;
}

void Motor_C620_CAN_RxCpltCallback(Motor_C620 *motor, uint8_t *Rx_Data)
{
    // 数据处理过程
    int16_t delta_encoder;
    uint16_t tmp_encoder;
    int16_t tmp_omega, tmp_current;
    Struct_Motor_DJI_CAN_Rx_Data *tmp_buffer = (Struct_Motor_DJI_CAN_Rx_Data *) motor->CAN_Manage_Object->Rx_Buffer.Data;

    // 处理大小端
    Math_Endian_Reverse_16((void *) &tmp_buffer->Encoder_Reverse, (void *) &tmp_encoder);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Omega_Reverse, (void *) &tmp_omega);
    Math_Endian_Reverse_16((void *) &tmp_buffer->Current_Reverse, (void *) &tmp_current);

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
    motor->Rx_Data.Total_Encoder = motor->Rx_Data.Total_Round * motor->Encoder_Num_Per_Round + tmp_encoder;

    //读取电机原始信息
    //motor->Rx_Origin.Last_Encoder_Reverse = motor->Rx_Origin.Encoder_Reverse;
    motor->Rx_Origin.Encoder_Reverse = tmp_buffer->Encoder_Reverse;
    motor->Rx_Origin.Omega_Reverse = tmp_buffer->Omega_Reverse;
    motor->Rx_Origin.Current_Reverse = tmp_buffer->Current_Reverse;
    motor->Rx_Origin.Temperature = tmp_buffer->Temperature;

    // 计算电机本身信息
    motor->Rx_Data.Now_Angle = (float) motor->Rx_Data.Total_Encoder / (float) motor->Encoder_Num_Per_Round * 2.0f * PI / motor->Gearbox_Rate;
    motor->Rx_Data.Now_Omega = (float) tmp_omega * RPM_TO_RADPS / motor->Gearbox_Rate;
    motor->Rx_Data.Now_Current = tmp_current / C620_Current_To_Out;
    motor->Rx_Data.Now_Temperature = tmp_buffer->Temperature + CELSIUS_TO_KELVIN;
    motor->Rx_Data.Now_Velocity = (float)tmp_omega * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
    motor->Rx_Data.Now_Power = power_calculate(motor->Power_K_0, motor->Power_K_1, motor->Power_K_2, motor->Power_A, motor->Rx_Data.Now_Current, motor->Rx_Data.Now_Omega);

    // 存储预备信息
    motor->Rx_Data.Pre_Encoder = tmp_encoder;
}

void Motor_C620_TIM_Alive_PeriodElapsedCallback(Motor_C620 *motor)
{
    if (motor->Flag == motor->Pre_Flag)
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        PID_Set_Integral_Error(&motor->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&motor->PID_Omega, 0.0f);
    }
    else
    {
        motor->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    motor->Pre_Flag = motor->Flag;
}

void Motor_C620_TIM_Calculate_PeriodElapsedCallback(Motor_C620 *motor)
{
    Motor_C620_TIM_PID_PeriodElapsedCallback(motor);

    float tmp_value = motor->Target_Current + motor->Feedforward_Current;
    Math_Constrain(&tmp_value, -motor->Current_Max, motor->Current_Max);
    motor->Out = tmp_value * C620_Current_To_Out;

    Motor_C620_Output(motor);

    if (motor->Power_Limit_Status == Motor_DJI_Power_Limit_Status_DISABLE)
    {
        motor->Feedforward_Current = 0.0f;
        motor->Feedforward_Omega = 0.0f;
    }
}

void Motor_C620_TIM_PID_PeriodElapsedCallback(Motor_C620 *motor)
{
    switch (motor->Control_Method)
    {
    case Motor_DJI_Control_Method_CURRENT:
	
        break;
    case Motor_DJI_Control_Method_OMEGA:
		
		motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
		motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        motor->Target_Current = motor->PID_Omega.Out;
	
        break;
	case Motor_DJI_Control_Method_Velocity:
		
		motor->PID_Velocity.Target = motor->Target_Velocity;
		motor->PID_Velocity.Now = motor->Rx_Data.Now_Velocity;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Velocity);


        motor->Target_Current = motor->PID_Velocity.Out;
	
		break;
    case Motor_DJI_Control_Method_ANGLE:
		
		motor->PID_Angle.Target = motor->Target_Angle;
		motor->PID_Angle.Now = motor->Rx_Data.Now_Angle;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Angle);
	
		motor->Target_Omega = motor->PID_Angle.Out;
	
		motor->PID_Omega.Target = motor->Target_Omega + motor->Feedforward_Omega;
		motor->PID_Omega.Now = motor->Rx_Data.Now_Omega;
		PID_TIM_Adjust_PeriodElapsedCallback(&motor->PID_Omega);

        motor->Target_Current = motor->PID_Omega.Out;
        break;
    default:
        Motor_C620_Set_Out(motor, 0.0f);
        break;
    }
}

void Motor_DJI_C620_Power_Limit_Control(Motor_C620 *motor)
{
    // 计算功率估计值
    motor->Power_Estimate = power_calculate(motor->Power_K_0, motor->Power_K_1, motor->Power_K_2, motor->Power_A, motor->Target_Current, motor->Rx_Data.Now_Omega);

    // 若功率为正则考虑功率控制限制
    if (motor->Power_Estimate > 0.0f)
    {
        if (motor->Power_Factor >= 1.0f)
        {
            // 无需功率控制
        }
        else
        {
            // 需要功率控制

            // 根据功率估计公式解一元二次方程求电流值
            float a = motor->Power_K_2;
            float b = motor->Power_K_0 * motor->Rx_Data.Now_Omega;
            float c = motor->Power_A + motor->Power_K_1 * motor->Rx_Data.Now_Omega * motor->Rx_Data.Now_Omega - motor->Power_Factor * motor->Power_Estimate;
            float delta, h;
            delta = b * b - 4 * a * c;
            if (delta < 0.0f)
            {
                // 无解
                motor->Target_Current = 0.0f;
            }
            else
            {
                arm_sqrt_f32(delta, &h);
                float result_1, result_2;
                result_1 = (-b + h) / (2.0f * a);
                result_2 = (-b - h) / (2.0f * a);

                // 两个潜在的可行电流值, 取绝对值最小的那个
                if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
                {
                    if ((motor->Target_Current > 0.0f && result_1 > 0.0f) || (motor->Target_Current < 0.0f && result_1 < 0.0f))
                    {
                        motor->Target_Current = result_1;
                    }
                    else
                    {
                        motor->Target_Current = result_2;
                    }
                }
                else
                {
                    if (Float_Math_Abs(result_1) < Float_Math_Abs(result_2))
                    {
                        motor->Target_Current = result_1;
                    }
                    else
                    {
                        motor->Target_Current = result_2;
                    }
                }
            }
        }
    }
}

void Motor_DJI_C620_TIM_Power_Limit_After_Calculate_PeriodElapsedCallback(Motor_C620 *motor)
{
    if (motor->Power_Limit_Status == Motor_DJI_Power_Limit_Status_ENABLE)
    {
        Motor_DJI_C620_Power_Limit_Control(motor);
    }

    Math_Constrain(&motor->Target_Current, -motor->Current_Max, motor->Current_Max);
    motor->Out = motor->Target_Current * C620_Current_To_Out;

    Motor_C620_Output(motor);

    motor->Feedforward_Current = 0.0f;
    motor->Feedforward_Omega = 0.0f;
}
