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

/* Block states (default storage) */
DW_stm32 stm32_DW;

/* External inputs (root inport signals with default storage) */
ExtU_stm32 stm32_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_stm32 stm32_Y;

/* Real-time model */
RT_MODEL_stm32 stm32_M_;
RT_MODEL_stm32 *const stm32_M = &stm32_M_;

/* Model step function */
void stm32_step(void)
{
  real_T rtb_Sum1;
  real_T rtb_Reciprocal;
  real_T rtb_FilterDifferentiatorTF;
  real_T rtb_IProdOut;
  real_T Integrator;
  real_T Integrator_d;
  real_T TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  real_T TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1;
  rtb_FilterDifferentiatorTF = stm32_U.P_N * 0.0005;
  rtb_Sum1 = 1.0 / (rtb_FilterDifferentiatorTF + 1.0);
  TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    (rtb_FilterDifferentiatorTF - 1.0) * rtb_Sum1;
  rtb_FilterDifferentiatorTF = stm32_U.S_N * 0.0005;
  rtb_Reciprocal = 1.0 / (rtb_FilterDifferentiatorTF + 1.0);
  TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
    (rtb_FilterDifferentiatorTF - 1.0) * rtb_Reciprocal;
  rtb_FilterDifferentiatorTF = stm32_U.angle_set - stm32_U.angle_feedback;
  rtb_IProdOut = rtb_FilterDifferentiatorTF * stm32_U.P_I;
  Integrator = 0.0005 * rtb_IProdOut + stm32_DW.Integrator_DSTATE;
  TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    rtb_FilterDifferentiatorTF * stm32_U.P_D -
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
    stm32_DW.FilterDifferentiatorTF_states;
  rtb_Sum1 = ((TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
               -stm32_DW.FilterDifferentiatorTF_states) * rtb_Sum1 * stm32_U.P_N
              + (rtb_FilterDifferentiatorTF * stm32_U.P_P + Integrator)) -
    stm32_U.speed_feedback;
  rtb_FilterDifferentiatorTF = rtb_Sum1 * stm32_U.S_D -
    TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
    stm32_DW.FilterDifferentiatorTF_states_o;
  TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = rtb_Sum1 *
    stm32_U.S_I;
  Integrator_d = 0.0005 *
    TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
    stm32_DW.Integrator_DSTATE_p;
  stm32_Y.Out1 = (rtb_FilterDifferentiatorTF +
                  -stm32_DW.FilterDifferentiatorTF_states_o) * rtb_Reciprocal *
    stm32_U.S_N + (rtb_Sum1 * stm32_U.S_P + Integrator_d);
  stm32_DW.Integrator_DSTATE = 0.0005 * rtb_IProdOut + Integrator;
  stm32_DW.FilterDifferentiatorTF_states =
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  stm32_DW.FilterDifferentiatorTF_states_o = rtb_FilterDifferentiatorTF;
  stm32_DW.Integrator_DSTATE_p = 0.0005 *
    TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 + Integrator_d;
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
