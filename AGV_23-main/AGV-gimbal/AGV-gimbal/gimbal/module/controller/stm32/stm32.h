/*
 * File: stm32.h
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

#ifndef RTW_HEADER_stm32_h_
#define RTW_HEADER_stm32_h_
#include "STM32_Config.h"
#include "stm32_External_Functions.h"
#ifndef stm32_COMMON_INCLUDES_
# define stm32_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* stm32_COMMON_INCLUDES_ */

#include "stm32_types.h"
#include "struct_typedef.h"
/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  fp32 Integrator_DSTATE;            /* '<S36>/Integrator' */
  fp32 FilterDifferentiatorTF_states;/* '<S29>/Filter Differentiator TF' */
  fp32 FilterDifferentiatorTF_states_o;/* '<S79>/Filter Differentiator TF' */
  fp32 Integrator_DSTATE_p;          /* '<S86>/Integrator' */
} DW_stm32;

/* External inputs (root inport signals with default storage) */
typedef struct {
  fp32 angle_set;                    /* '<Root>/angle_set' */
  fp32 angle_feedback;               /* '<Root>/angle_feedback' */
  fp32 speed_feedback;               /* '<Root>/speed_feedback' */
  fp32 P_P;                          /* '<Root>/P_P' */
  fp32 P_I;                          /* '<Root>/P_I' */
  fp32 P_D;                          /* '<Root>/P_D' */
  fp32 P_N;                          /* '<Root>/P_N' */
  fp32 S_P;                          /* '<Root>/S_P' */
  fp32 S_I;                          /* '<Root>/S_I' */
  fp32 S_D;                          /* '<Root>/S_D' */
  fp32 S_N;                          /* '<Root>/S_N' */
  fp32 KP;
  fp32 KI;
  fp32 KD;
  fp32 N;
	fp32 speed_set;
	fp32 speed_back;
} ExtU_stm32;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  fp32 Out1; 
  fp32 out_shoot;
  fp32 out_shoot1;	/* '<Root>/Out1' */
} ExtY_stm32;

/* Real-time Model Data Structure */
struct tag_RTM_stm32 {
  const char_T *errorStatus;
};

/* Block states (default storage) */
extern DW_stm32 stm32_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_stm32 stm32_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_stm32 stm32_Y;

/* Model entry point functions */
extern void stm32_initialize(void);
extern void stm32_step(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback);
extern void sm32_pid_init(void);

extern void stm32_pid_init_pitch(void);
extern void stm32_step_pitch(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback);
extern void stm32_pid_init_pitch_gyro(void);
extern void stm32_step_shoot_0(fp32 speedset,fp32 speedback);
extern void stm32_step_shoot_1(fp32 speedset,fp32 speedback);
extern void stm32_shoot_pid_init(void) ;
extern void stm32_step_auto(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback);
extern void stm32_step_pitch_auto(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback);
extern void stm32_auto_pid_init();
extern void stm32_pid_auto_init_pitch(void);
/* Real-time Model object */
extern RT_MODEL_stm32 *const stm32_M;
void stm32_relative_pid_init_pitch(void);
//pidÇå³ý
void stm32_step_shoot_pid_clear(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S29>/Passthrough for tuning' : Eliminate redundant data type conversion
 * Block '<S79>/Passthrough for tuning' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'stm32'
 * '<S1>'   : 'stm32/Discrete PID Controller'
 * '<S2>'   : 'stm32/Discrete PID Controller1'
 * '<S3>'   : 'stm32/Discrete PID Controller/Anti-windup'
 * '<S4>'   : 'stm32/Discrete PID Controller/D Gain'
 * '<S5>'   : 'stm32/Discrete PID Controller/Filter'
 * '<S6>'   : 'stm32/Discrete PID Controller/Filter ICs'
 * '<S7>'   : 'stm32/Discrete PID Controller/I Gain'
 * '<S8>'   : 'stm32/Discrete PID Controller/Ideal P Gain'
 * '<S9>'   : 'stm32/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S10>'  : 'stm32/Discrete PID Controller/Integrator'
 * '<S11>'  : 'stm32/Discrete PID Controller/Integrator ICs'
 * '<S12>'  : 'stm32/Discrete PID Controller/N Copy'
 * '<S13>'  : 'stm32/Discrete PID Controller/N Gain'
 * '<S14>'  : 'stm32/Discrete PID Controller/P Copy'
 * '<S15>'  : 'stm32/Discrete PID Controller/Parallel P Gain'
 * '<S16>'  : 'stm32/Discrete PID Controller/Reset Signal'
 * '<S17>'  : 'stm32/Discrete PID Controller/Saturation'
 * '<S18>'  : 'stm32/Discrete PID Controller/Saturation Fdbk'
 * '<S19>'  : 'stm32/Discrete PID Controller/Sum'
 * '<S20>'  : 'stm32/Discrete PID Controller/Sum Fdbk'
 * '<S21>'  : 'stm32/Discrete PID Controller/Tracking Mode'
 * '<S22>'  : 'stm32/Discrete PID Controller/Tracking Mode Sum'
 * '<S23>'  : 'stm32/Discrete PID Controller/Tsamp - Integral'
 * '<S24>'  : 'stm32/Discrete PID Controller/Tsamp - Ngain'
 * '<S25>'  : 'stm32/Discrete PID Controller/postSat Signal'
 * '<S26>'  : 'stm32/Discrete PID Controller/preSat Signal'
 * '<S27>'  : 'stm32/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S28>'  : 'stm32/Discrete PID Controller/D Gain/External Parameters'
 * '<S29>'  : 'stm32/Discrete PID Controller/Filter/Disc. Trapezoidal Filter'
 * '<S30>'  : 'stm32/Discrete PID Controller/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S31>'  : 'stm32/Discrete PID Controller/Filter/Disc. Trapezoidal Filter/Tsamp/Internal Ts'
 * '<S32>'  : 'stm32/Discrete PID Controller/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'stm32/Discrete PID Controller/I Gain/External Parameters'
 * '<S34>'  : 'stm32/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S35>'  : 'stm32/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'stm32/Discrete PID Controller/Integrator/Discrete'
 * '<S37>'  : 'stm32/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'stm32/Discrete PID Controller/N Copy/External Parameters'
 * '<S39>'  : 'stm32/Discrete PID Controller/N Gain/External Parameters'
 * '<S40>'  : 'stm32/Discrete PID Controller/P Copy/Disabled'
 * '<S41>'  : 'stm32/Discrete PID Controller/Parallel P Gain/External Parameters'
 * '<S42>'  : 'stm32/Discrete PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'stm32/Discrete PID Controller/Saturation/Passthrough'
 * '<S44>'  : 'stm32/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S45>'  : 'stm32/Discrete PID Controller/Sum/Sum_PID'
 * '<S46>'  : 'stm32/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S47>'  : 'stm32/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S48>'  : 'stm32/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S49>'  : 'stm32/Discrete PID Controller/Tsamp - Integral/Passthrough'
 * '<S50>'  : 'stm32/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S51>'  : 'stm32/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S52>'  : 'stm32/Discrete PID Controller/preSat Signal/Forward_Path'
 * '<S53>'  : 'stm32/Discrete PID Controller1/Anti-windup'
 * '<S54>'  : 'stm32/Discrete PID Controller1/D Gain'
 * '<S55>'  : 'stm32/Discrete PID Controller1/Filter'
 * '<S56>'  : 'stm32/Discrete PID Controller1/Filter ICs'
 * '<S57>'  : 'stm32/Discrete PID Controller1/I Gain'
 * '<S58>'  : 'stm32/Discrete PID Controller1/Ideal P Gain'
 * '<S59>'  : 'stm32/Discrete PID Controller1/Ideal P Gain Fdbk'
 * '<S60>'  : 'stm32/Discrete PID Controller1/Integrator'
 * '<S61>'  : 'stm32/Discrete PID Controller1/Integrator ICs'
 * '<S62>'  : 'stm32/Discrete PID Controller1/N Copy'
 * '<S63>'  : 'stm32/Discrete PID Controller1/N Gain'
 * '<S64>'  : 'stm32/Discrete PID Controller1/P Copy'
 * '<S65>'  : 'stm32/Discrete PID Controller1/Parallel P Gain'
 * '<S66>'  : 'stm32/Discrete PID Controller1/Reset Signal'
 * '<S67>'  : 'stm32/Discrete PID Controller1/Saturation'
 * '<S68>'  : 'stm32/Discrete PID Controller1/Saturation Fdbk'
 * '<S69>'  : 'stm32/Discrete PID Controller1/Sum'
 * '<S70>'  : 'stm32/Discrete PID Controller1/Sum Fdbk'
 * '<S71>'  : 'stm32/Discrete PID Controller1/Tracking Mode'
 * '<S72>'  : 'stm32/Discrete PID Controller1/Tracking Mode Sum'
 * '<S73>'  : 'stm32/Discrete PID Controller1/Tsamp - Integral'
 * '<S74>'  : 'stm32/Discrete PID Controller1/Tsamp - Ngain'
 * '<S75>'  : 'stm32/Discrete PID Controller1/postSat Signal'
 * '<S76>'  : 'stm32/Discrete PID Controller1/preSat Signal'
 * '<S77>'  : 'stm32/Discrete PID Controller1/Anti-windup/Passthrough'
 * '<S78>'  : 'stm32/Discrete PID Controller1/D Gain/External Parameters'
 * '<S79>'  : 'stm32/Discrete PID Controller1/Filter/Disc. Trapezoidal Filter'
 * '<S80>'  : 'stm32/Discrete PID Controller1/Filter/Disc. Trapezoidal Filter/Tsamp'
 * '<S81>'  : 'stm32/Discrete PID Controller1/Filter/Disc. Trapezoidal Filter/Tsamp/Internal Ts'
 * '<S82>'  : 'stm32/Discrete PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S83>'  : 'stm32/Discrete PID Controller1/I Gain/External Parameters'
 * '<S84>'  : 'stm32/Discrete PID Controller1/Ideal P Gain/Passthrough'
 * '<S85>'  : 'stm32/Discrete PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S86>'  : 'stm32/Discrete PID Controller1/Integrator/Discrete'
 * '<S87>'  : 'stm32/Discrete PID Controller1/Integrator ICs/Internal IC'
 * '<S88>'  : 'stm32/Discrete PID Controller1/N Copy/External Parameters'
 * '<S89>'  : 'stm32/Discrete PID Controller1/N Gain/External Parameters'
 * '<S90>'  : 'stm32/Discrete PID Controller1/P Copy/Disabled'
 * '<S91>'  : 'stm32/Discrete PID Controller1/Parallel P Gain/External Parameters'
 * '<S92>'  : 'stm32/Discrete PID Controller1/Reset Signal/Disabled'
 * '<S93>'  : 'stm32/Discrete PID Controller1/Saturation/Passthrough'
 * '<S94>'  : 'stm32/Discrete PID Controller1/Saturation Fdbk/Disabled'
 * '<S95>'  : 'stm32/Discrete PID Controller1/Sum/Sum_PID'
 * '<S96>'  : 'stm32/Discrete PID Controller1/Sum Fdbk/Disabled'
 * '<S97>'  : 'stm32/Discrete PID Controller1/Tracking Mode/Disabled'
 * '<S98>'  : 'stm32/Discrete PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S99>'  : 'stm32/Discrete PID Controller1/Tsamp - Integral/Passthrough'
 * '<S100>' : 'stm32/Discrete PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S101>' : 'stm32/Discrete PID Controller1/postSat Signal/Forward_Path'
 * '<S102>' : 'stm32/Discrete PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_stm32_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] stm32.h
 */
