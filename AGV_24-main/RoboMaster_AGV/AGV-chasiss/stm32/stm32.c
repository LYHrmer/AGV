/*
 * File: stm32.c
 *
 * Code generated for Simulink model :stm32.
 *
 * Model version      : 1.3
 * Simulink Coder version    : 9.3 (R2020a) 18-Nov-2019
 * TLC version       : 9.3 (Jan 23 2020)
 * C/C++ source code generated on  : Sat Feb  5 19:50:19 2022
 *
 * Target selection: stm32.tlc
 * Embedded hardware selection: STM32CortexM
 * Code generation objectives: Unspecified
 * Validation result: Not run
 *
 *
 *
 * ******************************************************************************
 * * attention
 * *
 * * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * *
 * ******************************************************************************
 */

#include "stm32.h"
#include "stm32_private.h"
#include "main.h"
#include "chassis_task.h"
extern chassis_move_t chassis_move;
#define PI 3.1415926f
/* Block states (default storage) */
/**
  * @brief          舵电机PID计算
  * @author         XQL 1.0
	*                 LYH 2.0
  * @param[in]      angle_set: 角度设定
  * @param[in]      angle_feedback：当前角度
  * @param[in]      speed_feedback： 速度反馈
  * @param[in]      rudder_PID：PID参数
  * @retval         返回空
  */
void Matlab_PID_Calc(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback,Rudder_control*rudder_PID) 
{   
  rudder_PID->rudder_in.angle_set=angle_set;
  rudder_PID->rudder_in.angle_feedback=angle_feedback;
  rudder_PID->rudder_in.speed_feedback=speed_feedback;
	
  rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF = rudder_PID->rudder_in.P_N * 0.0005f;
  rudder_PID->stm32_PID_param.rtb_Sum1 = 1.0f / (rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF + 1.0f);
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
  (rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF - 1.0f) * rudder_PID->stm32_PID_param.rtb_Sum1;
  rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF = rudder_PID->rudder_in.S_N * 0.0005f;
  rudder_PID->stm32_PID_param.rtb_Reciprocal = 1.0f / (rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF + 1.0f);
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
  (rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF - 1.0f) * rudder_PID->stm32_PID_param.rtb_Reciprocal;
  rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF = rudder_PID->rudder_in.angle_set - rudder_PID->rudder_in.angle_feedback;
	
//	if(rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF>1.5f*PI)
//	{
//		rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF-=2.f*PI;
//	}
//	else if(rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF<-1.5f*PI)
//	{
//	   rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF+=2.f*PI;
//	}
	
  rudder_PID->stm32_PID_param.rtb_IProdOut = rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF * rudder_PID->rudder_in.P_I;
  rudder_PID->stm32_PID_param.Integrator = 0.0005f * rudder_PID->stm32_PID_param.rtb_IProdOut + rudder_PID->rudder_param.Integrator_DSTATE;
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
  rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF * rudder_PID->rudder_in.P_D -
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
  rudder_PID->rudder_param.FilterDifferentiatorTF_states;
  rudder_PID->stm32_PID_param.rtb_Sum1 = ((rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
  -rudder_PID->rudder_param.FilterDifferentiatorTF_states) * rudder_PID->stm32_PID_param.rtb_Sum1 * rudder_PID->rudder_in.P_N
  + (rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF * rudder_PID->rudder_in.P_P + rudder_PID->stm32_PID_param.Integrator)) -
  rudder_PID->rudder_in.speed_feedback;
  rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF = rudder_PID->stm32_PID_param.rtb_Sum1 * rudder_PID->rudder_in.S_D -
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
  rudder_PID->rudder_param.FilterDifferentiatorTF_states_o;
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = rudder_PID->stm32_PID_param.rtb_Sum1 *
  rudder_PID->rudder_in.S_I;
  rudder_PID->stm32_PID_param.Integrator_d = 0.0005f *
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
  rudder_PID->rudder_param.Integrator_DSTATE_p;
	
  rudder_PID->rudder_out.Out1 = (rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF +
  -rudder_PID->rudder_param.FilterDifferentiatorTF_states_o) * rudder_PID->stm32_PID_param.rtb_Reciprocal *
  rudder_PID->rudder_in.S_N + (rudder_PID->stm32_PID_param.rtb_Sum1 * rudder_PID->rudder_in.S_P + rudder_PID->stm32_PID_param.Integrator_d);
	
	if(rudder_PID->rudder_out.Out1>=9000.f) rudder_PID->rudder_out.Out1=9000.f;       //控制舵电机峰值电流
	else if(rudder_PID->rudder_out.Out1<=-9000.f) rudder_PID->rudder_out.Out1=-9000.f;

//		if (get_capA.cap_voltage*0.001f < 15) // 当缓冲能量过低时，全面降低功率
//	{
//		if(rudder_PID->rudder_out.Out1>=1000.f) rudder_PID->rudder_out.Out1=1000.f;       //控制舵电机峰值电流
//	else if(rudder_PID->rudder_out.Out1<=-1000.f) rudder_PID->rudder_out.Out1=-1000.f;
//	}
//	else
//	{
		if(rudder_PID->rudder_out.Out1>=9000.f) rudder_PID->rudder_out.Out1=9000.f;       //控制舵电机峰值电流
	else if(rudder_PID->rudder_out.Out1<=-9000.f) rudder_PID->rudder_out.Out1=-9000.f;
//	}

  rudder_PID->rudder_param.Integrator_DSTATE = 0.0005f * rudder_PID->stm32_PID_param.rtb_IProdOut + rudder_PID->stm32_PID_param.Integrator;
  rudder_PID->rudder_param.FilterDifferentiatorTF_states =
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  rudder_PID->rudder_param.FilterDifferentiatorTF_states_o = rudder_PID->stm32_PID_param.rtb_FilterDifferentiatorTF;
  rudder_PID->rudder_param.Integrator_DSTATE_p = 0.0005f *
  rudder_PID->stm32_PID_param.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 + rudder_PID->stm32_PID_param.Integrator_d;
}
/* Model initialize function */
void stm32_initialize(void)
{
  /* (no initialization code required) */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] stm32.c
 */
