#include "stm32.h"
#include "stm32_private.h"
#include <string.h>

#define PI 3.1415926
/* Block states (default storage) */
DW_stm32 stm32_DW;
DW_stm32 stm32_DW_pitch;
DW_stm32 stm32_DW_shoot;
DW_stm32 stm32_DW_shoot_1;
DW_stm32 stm32_DW_yaw_auto;
DW_stm32 stm32_DW_pitch_auto;
/* External inputs (root inport signals with default storage) */
ExtU_stm32 stm32_U;
ExtU_stm32 stm32_U_pitch;
ExtU_stm32 stm32_U_auto;
ExtU_stm32 stm32_U_pitch_auto;
/* External outputs (root outports fed by signals with default storage) */
ExtY_stm32 stm32_Y;
ExtY_stm32 stm32_Y_pitch;
ExtY_stm32 stm32_Y_auto;
ExtY_stm32 stm32_Y_pitch_auto;
/* Real-time model */
RT_MODEL_stm32 stm32_M_;
RT_MODEL_stm32 *const stm32_M = &stm32_M_;

/* Model step function */
void stm32_auto_pid_init(void)  //yaw
{
	stm32_U_auto.P_P=1200;
	stm32_U_auto.P_I=1;
	stm32_U_auto.P_D=10	;
	stm32_U_auto.P_N=120;
	stm32_U_auto.S_P=50;
	stm32_U_auto.S_I=0;
	stm32_U_auto.S_D=5;
	stm32_U_auto.S_N=40;
		
}
void stm32_pid_auto_init_pitch(void)  //pitch
{
	stm32_U_pitch_auto.P_P=1200;//   1100;
	stm32_U_pitch_auto.P_I=1;  //1
	stm32_U_pitch_auto.P_D=52;   //50	 ; 
	stm32_U_pitch_auto.P_N=30	;  //30
	stm32_U_pitch_auto.S_P=110;  //280
	stm32_U_pitch_auto.S_I=1;     //1
	stm32_U_pitch_auto.S_D=3;    //3
	stm32_U_pitch_auto.S_N=20;  //20
		
}
void stm32_pid_init(void)  //yaw
{

	
//	stm32_U.P_P=1300;
//	stm32_U.P_I=1;
//	stm32_U.P_D=10	;
//	stm32_U.P_N=35;
//	stm32_U.S_P=50;
//	stm32_U.S_I=0;
//	stm32_U.S_D=5;
//	stm32_U.S_N=20;
	
	stm32_U.P_P=1400;
	stm32_U.P_I=1;
	stm32_U.P_D=10;
	stm32_U.P_N=30;
	stm32_U.S_P=50;
	stm32_U.S_I=0;
	stm32_U.S_D=5;
	stm32_U.S_N=20;

}
void stm32_shoot_pid_init(void)  
{
	stm32_U.KP=10000;
	stm32_U.KI=80;
	stm32_U.KD=10	;
	stm32_U.N=70;
		
}

void stm32_pid_init_pitch(void)  //pitch
{
	stm32_U_pitch.P_P=1050;
	stm32_U_pitch.P_I=5;
	stm32_U_pitch.P_D=20;
	stm32_U_pitch.P_N=20;
	stm32_U_pitch.S_P=90;
	stm32_U_pitch.S_I=5;
	stm32_U_pitch.S_D=10;
	stm32_U_pitch.S_N=20;
	
//			stm32_U_pitch.P_P=1000;
//	stm32_U_pitch.P_I=5;
//	stm32_U_pitch.P_D=20;
//	stm32_U_pitch.P_N=20;
//	stm32_U_pitch.S_P=90;
//	stm32_U_pitch.S_I=5;
//	stm32_U_pitch.S_D=10;
//	stm32_U_pitch.S_N=40;

//	stm32_U_pitch.P_P=1100;
//	stm32_U_pitch.P_I=1;
//	stm32_U_pitch.P_D=52;
//	stm32_U_pitch.P_N=20;
//	stm32_U_pitch.S_P=110;
//	stm32_U_pitch.S_I=1;
//	stm32_U_pitch.S_D=3;
//	stm32_U_pitch.S_N=15;
}
void stm32_relative_pid_init_pitch(void)  //pitch
{
	stm32_U_pitch.P_P=1300;
	stm32_U_pitch.P_I=0.2;  
	stm32_U_pitch.P_D=19;   
	stm32_U_pitch.P_N=35	;  
	stm32_U_pitch.S_P=80;  
	stm32_U_pitch.S_I=1;     
	stm32_U_pitch.S_D=2;    
	stm32_U_pitch.S_N=45;  
		
}
void stm32_pid_init_pitch_gyro(void)  //pitch
{
	stm32_U_pitch.P_P=-1200;         //-1400
	stm32_U_pitch.P_I=-4;            //-1
	stm32_U_pitch.P_D=-35;          //-35
	stm32_U_pitch.P_N=200;           //100
	stm32_U_pitch.S_P=125;           //120
	stm32_U_pitch.S_I=4;             //1
	stm32_U_pitch.S_D=1;             //4
	stm32_U_pitch.S_N=55;            //15
		
}
typedef struct
{
   fp32 rtb_Sum1;
   fp32 rtb_Reciprocal;
   fp32 rtb_FilterDifferentiatorTF;
   fp32 rtb_IProdOut;
   fp32 Integrator;
   fp32 Integrator_d;
   fp32 TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
   fp32 TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1;
		
} stm32_PID_t;

stm32_PID_t stm32_pid;  //yaw
stm32_PID_t stm32_pid_pitch; // pitch
stm32_PID_t stm32_auto_pid;  //yaw_auto
stm32_PID_t stm32_pid_auto_pitch; // pitch_auto
void stm32_step(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback)  //yaw
{   
   
  stm32_U.angle_set=angle_set;
  stm32_U.angle_feedback=angle_feedback;
  stm32_U.speed_feedback=speed_feedback;
  stm32_pid.rtb_FilterDifferentiatorTF = stm32_U.P_N * 0.0005f;
  stm32_pid.rtb_Sum1 = 1.0 / (stm32_pid.rtb_FilterDifferentiatorTF + 1.0);
  stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    (stm32_pid.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid.rtb_Sum1;
  stm32_pid.rtb_FilterDifferentiatorTF = stm32_U.S_N * 0.0005f;
  stm32_pid.rtb_Reciprocal = 1.0 / (stm32_pid.rtb_FilterDifferentiatorTF + 1.0);
  stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
    (stm32_pid.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid.rtb_Reciprocal;
  stm32_pid.rtb_FilterDifferentiatorTF = stm32_U.angle_set - stm32_U.angle_feedback;
	if(stm32_pid.rtb_FilterDifferentiatorTF>1.5f*PI)
	{
		stm32_pid.rtb_FilterDifferentiatorTF-=2*PI;
	}
	else if(stm32_pid.rtb_FilterDifferentiatorTF<-1.5f*PI)
	{
	   stm32_pid.rtb_FilterDifferentiatorTF+=2*PI;
	}
  stm32_pid.rtb_IProdOut = stm32_pid.rtb_FilterDifferentiatorTF * stm32_U.P_I;
  stm32_pid.Integrator = 0.0005f * stm32_pid.rtb_IProdOut + stm32_DW.Integrator_DSTATE;
  stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    stm32_pid.rtb_FilterDifferentiatorTF * stm32_U.P_D -
    stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
    stm32_DW.FilterDifferentiatorTF_states;
  stm32_pid.rtb_Sum1 = ((stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
               -stm32_DW.FilterDifferentiatorTF_states) * stm32_pid.rtb_Sum1 * stm32_U.P_N
              + (stm32_pid.rtb_FilterDifferentiatorTF * stm32_U.P_P + stm32_pid.Integrator)) -
    stm32_U.speed_feedback;
  stm32_pid.rtb_FilterDifferentiatorTF = stm32_pid.rtb_Sum1 * stm32_U.S_D -
    stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
    stm32_DW.FilterDifferentiatorTF_states_o;
  stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid.rtb_Sum1 *
    stm32_U.S_I;
  stm32_pid.Integrator_d = 0.0005f *
    stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
    stm32_DW.Integrator_DSTATE_p;
  stm32_Y.Out1 = (stm32_pid.rtb_FilterDifferentiatorTF +
                  -stm32_DW.FilterDifferentiatorTF_states_o) * stm32_pid.rtb_Reciprocal *
    stm32_U.S_N + (stm32_pid.rtb_Sum1 * stm32_U.S_P + stm32_pid.Integrator_d);
	
	if(stm32_Y.Out1>=30000) stm32_Y.Out1=30000;
	else if(stm32_Y.Out1<=-30000) stm32_Y.Out1=-30000;
  stm32_DW.Integrator_DSTATE = 0.0005f * stm32_pid.rtb_IProdOut + stm32_pid.Integrator;
  stm32_DW.FilterDifferentiatorTF_states =
    stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  stm32_DW.FilterDifferentiatorTF_states_o = stm32_pid.rtb_FilterDifferentiatorTF;
  stm32_DW.Integrator_DSTATE_p = 0.0005f *
    stm32_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 + stm32_pid.Integrator_d;
}
void stm32_step_auto(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback)  //yaw
{   
   
  stm32_U_auto.angle_set=angle_set;
  stm32_U_auto.angle_feedback=angle_feedback;
  stm32_U_auto.speed_feedback=speed_feedback;
  stm32_auto_pid.rtb_FilterDifferentiatorTF = stm32_U_auto.P_N * 0.0005f;
  stm32_auto_pid.rtb_Sum1 = 1.0f / (stm32_auto_pid.rtb_FilterDifferentiatorTF + 1.0f);
  stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    (stm32_auto_pid.rtb_FilterDifferentiatorTF - 1.0f) * stm32_auto_pid.rtb_Sum1;
  stm32_auto_pid.rtb_FilterDifferentiatorTF = stm32_U_auto.S_N * 0.0005f;
  stm32_auto_pid.rtb_Reciprocal = 1.0 / (stm32_auto_pid.rtb_FilterDifferentiatorTF + 1.0);
  stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
    (stm32_auto_pid.rtb_FilterDifferentiatorTF - 1.0) * stm32_auto_pid.rtb_Reciprocal;
  stm32_auto_pid.rtb_FilterDifferentiatorTF = stm32_U_auto.angle_set - stm32_U_auto.angle_feedback;
	if(stm32_auto_pid.rtb_FilterDifferentiatorTF>1.5*PI)
	{
		stm32_auto_pid.rtb_FilterDifferentiatorTF-=2*PI;
	}
	else if(stm32_auto_pid.rtb_FilterDifferentiatorTF<-1.5*PI)
	{
	   stm32_auto_pid.rtb_FilterDifferentiatorTF+=2*PI;
	}
  stm32_auto_pid.rtb_IProdOut = stm32_auto_pid.rtb_FilterDifferentiatorTF * stm32_U_auto.P_I;
  stm32_auto_pid.Integrator = 0.0005f * stm32_auto_pid.rtb_IProdOut + stm32_DW_yaw_auto.Integrator_DSTATE;
  stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    stm32_auto_pid.rtb_FilterDifferentiatorTF * stm32_U_auto.P_D -
    stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
    stm32_DW_yaw_auto.FilterDifferentiatorTF_states;
  stm32_auto_pid.rtb_Sum1 = ((stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
               -stm32_DW_yaw_auto.FilterDifferentiatorTF_states) * stm32_auto_pid.rtb_Sum1 * stm32_U_auto.P_N
              + (stm32_auto_pid.rtb_FilterDifferentiatorTF * stm32_U_auto.P_P + stm32_auto_pid.Integrator)) -
    stm32_U_auto.speed_feedback;
  stm32_auto_pid.rtb_FilterDifferentiatorTF = stm32_auto_pid.rtb_Sum1 * stm32_U_auto.S_D -
    stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
    stm32_DW_yaw_auto.FilterDifferentiatorTF_states_o;
  stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_auto_pid.rtb_Sum1 *
    stm32_U_auto.S_I;
  stm32_auto_pid.Integrator_d = 0.0005f *
    stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
    stm32_DW_yaw_auto.Integrator_DSTATE_p;
  stm32_Y_auto.Out1 = (stm32_auto_pid.rtb_FilterDifferentiatorTF +
                  -stm32_DW_yaw_auto.FilterDifferentiatorTF_states_o) * stm32_auto_pid.rtb_Reciprocal *
    stm32_U_auto.S_N + (stm32_auto_pid.rtb_Sum1 * stm32_U.S_P + stm32_auto_pid.Integrator_d);
	
	if(stm32_Y_auto.Out1>=30000) stm32_Y_auto.Out1=30000;
	else if(stm32_Y_auto.Out1<=-30000) stm32_Y_auto.Out1=-30000;
  stm32_DW_yaw_auto.Integrator_DSTATE = 0.0005f * stm32_auto_pid.rtb_IProdOut + stm32_auto_pid.Integrator;
  stm32_DW_yaw_auto.FilterDifferentiatorTF_states =
    stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  stm32_DW_yaw_auto.FilterDifferentiatorTF_states_o = stm32_auto_pid.rtb_FilterDifferentiatorTF;
  stm32_DW_yaw_auto.Integrator_DSTATE_p = 0.0005f *
    stm32_auto_pid.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 + stm32_auto_pid.Integrator_d;
}
void stm32_step_pitch_auto(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback)  //pitch
{   
   
  stm32_U_pitch_auto.angle_set=angle_set;
  stm32_U_pitch_auto.angle_feedback=angle_feedback;
  stm32_U_pitch_auto.speed_feedback=speed_feedback;
  stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch_auto.P_N * 0.0005f;
  stm32_pid_auto_pitch.rtb_Sum1 = 1.0 / (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF + 1.0);
  stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_auto_pitch.rtb_Sum1;
  stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch_auto.S_N * 0.0005f;
  stm32_pid_auto_pitch.rtb_Reciprocal = 1.0f / (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF + 1.0);
  stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
    (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_auto_pitch.rtb_Reciprocal;
  stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch_auto.angle_set - stm32_U_pitch_auto.angle_feedback;
	
  stm32_pid_auto_pitch.rtb_IProdOut = stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch_auto.P_I;
  stm32_pid_auto_pitch.Integrator = 0.0005f * stm32_pid_auto_pitch.rtb_IProdOut + stm32_DW_pitch_auto.Integrator_DSTATE;
  stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch_auto.P_D -
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
    stm32_DW_pitch_auto.FilterDifferentiatorTF_states;
  stm32_pid_auto_pitch.rtb_Sum1 = ((stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
               -stm32_DW_pitch_auto.FilterDifferentiatorTF_states) * stm32_pid_auto_pitch.rtb_Sum1 * stm32_U_pitch_auto.P_N
              + (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch_auto.P_P + stm32_pid_auto_pitch.Integrator)) -
    stm32_U_pitch_auto.speed_feedback;
  stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF = stm32_pid_auto_pitch.rtb_Sum1 * stm32_U_pitch_auto.S_D -
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
    stm32_DW_pitch_auto.FilterDifferentiatorTF_states_o;
  stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_auto_pitch.rtb_Sum1 *
    stm32_U_pitch_auto.S_I;
  stm32_pid_auto_pitch.Integrator_d = 0.0005f *
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
    stm32_DW_pitch_auto.Integrator_DSTATE_p;
  stm32_Y_pitch_auto.Out1 = (stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF +
                  -stm32_DW_pitch_auto.FilterDifferentiatorTF_states_o) * stm32_pid_auto_pitch.rtb_Reciprocal *
    stm32_U_pitch_auto.S_N + (stm32_pid_auto_pitch.rtb_Sum1 * stm32_U_pitch_auto.S_P + stm32_pid_auto_pitch.Integrator_d);
	
	if(stm32_Y_pitch_auto.Out1>=30000) stm32_Y_pitch_auto.Out1=30000;
	else if(stm32_Y_pitch_auto.Out1<=-30000) stm32_Y_pitch_auto.Out1=-30000;
  stm32_DW_pitch_auto.Integrator_DSTATE = 0.0005f * stm32_pid_auto_pitch.rtb_IProdOut + stm32_pid_auto_pitch.Integrator;
  stm32_DW_pitch_auto.FilterDifferentiatorTF_states =
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  stm32_DW_pitch_auto.FilterDifferentiatorTF_states_o = stm32_pid_auto_pitch.rtb_FilterDifferentiatorTF;
  stm32_DW_pitch_auto.Integrator_DSTATE_p = 0.0005f *
    stm32_pid_auto_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 + stm32_pid_auto_pitch.Integrator_d;
}
void stm32_step_pitch(fp32 angle_set,fp32 angle_feedback,fp32 speed_feedback)  //pitch
{   
   
  stm32_U_pitch.angle_set=angle_set;
  stm32_U_pitch.angle_feedback=angle_feedback;
  stm32_U_pitch.speed_feedback=speed_feedback;
  stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.P_N * 0.0005f;
  stm32_pid_pitch.rtb_Sum1 = 1.0 / (stm32_pid_pitch.rtb_FilterDifferentiatorTF + 1.0);
  stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    (stm32_pid_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_pitch.rtb_Sum1;
  stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.S_N * 0.0005f;
  stm32_pid_pitch.rtb_Reciprocal = 1.0 / (stm32_pid_pitch.rtb_FilterDifferentiatorTF + 1.0);
  stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
    (stm32_pid_pitch.rtb_FilterDifferentiatorTF - 1.0) * stm32_pid_pitch.rtb_Reciprocal;
  stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.angle_set - stm32_U_pitch.angle_feedback;
	
  stm32_pid_pitch.rtb_IProdOut = stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_I;
  stm32_pid_pitch.Integrator = 0.0005f * stm32_pid_pitch.rtb_IProdOut + stm32_DW_pitch.Integrator_DSTATE;
  stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
    stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_D -
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
    stm32_DW_pitch.FilterDifferentiatorTF_states;
  stm32_pid_pitch.rtb_Sum1 = ((stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
               -stm32_DW_pitch.FilterDifferentiatorTF_states) * stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.P_N
              + (stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_P + stm32_pid_pitch.Integrator)) -
    stm32_U_pitch.speed_feedback;
  stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.S_D -
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
    stm32_DW_pitch.FilterDifferentiatorTF_states_o;
  stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_pitch.rtb_Sum1 *
    stm32_U_pitch.S_I;
  stm32_pid_pitch.Integrator_d = 0.0005f *
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
    stm32_DW_pitch.Integrator_DSTATE_p;
  stm32_Y_pitch.Out1 = (stm32_pid_pitch.rtb_FilterDifferentiatorTF +
                  -stm32_DW_pitch.FilterDifferentiatorTF_states_o) * stm32_pid_pitch.rtb_Reciprocal *
    stm32_U_pitch.S_N + (stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.S_P + stm32_pid_pitch.Integrator_d);
	
	if(stm32_Y_pitch.Out1>=30000) stm32_Y_pitch.Out1=30000;
	else if(stm32_Y_pitch.Out1<=-30000) stm32_Y_pitch.Out1=-30000;
  stm32_DW_pitch.Integrator_DSTATE = 0.0005f * stm32_pid_pitch.rtb_IProdOut + stm32_pid_pitch.Integrator;
  stm32_DW_pitch.FilterDifferentiatorTF_states =
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  stm32_DW_pitch.FilterDifferentiatorTF_states_o = stm32_pid_pitch.rtb_FilterDifferentiatorTF;
  stm32_DW_pitch.Integrator_DSTATE_p = 0.0005f *
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 + stm32_pid_pitch.Integrator_d;
}


void stm32_step_shoot_0(fp32 speedset,fp32 speedback)
{
  real_T rtb_Reciprocal;
  real_T rtb_Sum_p;
  real_T rtb_IProdOut;
  real_T Integrator;
  real_T TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  rtb_Sum_p = stm32_U.N * 0.0005f;
  rtb_Reciprocal = 1.0f / (rtb_Sum_p + 1.0f);
  TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = (rtb_Sum_p - 1.0f) *
    rtb_Reciprocal;
  rtb_Sum_p = speedset - speedback;
  TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = rtb_Sum_p *
    stm32_U.KD - TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
    stm32_DW_shoot.FilterDifferentiatorTF_states;
  rtb_IProdOut = rtb_Sum_p * stm32_U.KI;
  Integrator = 0.0005f * rtb_IProdOut + stm32_DW_shoot.Integrator_DSTATE;
  stm32_Y.out_shoot = (TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                  -stm32_DW_shoot.FilterDifferentiatorTF_states) * rtb_Reciprocal *
    stm32_U.N + (rtb_Sum_p * stm32_U.KP + Integrator);
  stm32_DW_shoot.FilterDifferentiatorTF_states =
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  stm32_DW_shoot.Integrator_DSTATE = 0.0005f * rtb_IProdOut + Integrator;
}
void stm32_step_shoot_1(fp32 speedset,fp32 speedback)
{
  real_T rtb_Reciprocal;
  real_T rtb_Sum_p;
  real_T rtb_IProdOut;
  real_T Integrator;
  real_T TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  rtb_Sum_p = stm32_U.N * 0.0005f;
  rtb_Reciprocal = 1.0 / (rtb_Sum_p + 1.0);
  TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = (rtb_Sum_p - 1.0) *
    rtb_Reciprocal;
  rtb_Sum_p = speedset - speedback;
  TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = rtb_Sum_p *
    stm32_U.KD - TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
    stm32_DW_shoot_1.FilterDifferentiatorTF_states;
  rtb_IProdOut = rtb_Sum_p * stm32_U.KI;
  Integrator = 0.0005f * rtb_IProdOut + stm32_DW_shoot_1.Integrator_DSTATE;
  stm32_Y.out_shoot1 = (TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                  -stm32_DW_shoot_1.FilterDifferentiatorTF_states) * rtb_Reciprocal *
    stm32_U.N + (rtb_Sum_p * stm32_U.KP + Integrator);
  stm32_DW_shoot_1.FilterDifferentiatorTF_states =
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
  stm32_DW_shoot_1.Integrator_DSTATE = 0.0005f * rtb_IProdOut + Integrator;
}
/* Model initialize function */
void stm32_initialize(void)
{
  /* (no initialization code required) */
}

void stm32_step_shoot_pid_clear(void)
{
    // 清空发射摩擦轮电机PID数据
    memset((void *)&stm32_DW_shoot, 0, sizeof(stm32_DW_shoot));
    memset((void *)&stm32_DW_shoot_1, 0, sizeof(stm32_DW_shoot_1));
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] stm32.c
 */