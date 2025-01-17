/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
//前馈控制
typedef struct{
    float K1;
    float K2;
    float Last_DeltIn;
    float Now_DeltIn;
    float Out;
    float OutMax;
}FeedForward_Typedef;

//普通PID部分
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

enum PID_MODE_AGAIN
{
    KI_SEPRATE = 0, //积分分离
    KD_NO_FULL,     //不完全微分
};
typedef struct
{
    uint8_t mode;
		uint8_t mode_again;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
		fp32 Kf;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出
	 

    fp32 set;
    fp32 fdb;

    fp32 out;
		fp32 last_out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
		fp32 Last_Dout;
		fp32 Fout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
    int flag;
		fp32 F_divider;//前馈分离
		fp32 F_out_limit;//前馈限幅

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[8], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_Clear(PidTypeDef *pid);


//模糊PID部分
#define NL -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PL 3

typedef struct
{
    float SetPoint; //设定目标值

    float ActualValue; //实际值

    float DeadZone;

    float LastError; //前次误差
    float PreError;  //当前误差
    float SumError;  //积分误差

    float IMax; //积分限制

    float POut;      //比例输出
    float IOut;      //积分输出
    float DOut;      //微分输出
    float DOut_last; //上一次微分输出
    float OutMax;    //限幅
    float Out;       //总输出
    float Out_last;  //上一次输出

    float I_U; //变速积分上限
    float I_L; //变速积分下限

    float RC_DM; //微分先行滤波系数
    float RC_DF; //不完全微分滤波系数

    float Kp0; // PID初值
    float Ki0;
    float Kd0;

    float dKp; // PID变化量
    float dKi;
    float dKd;

    float stair;    //动态调整梯度   //0.25f
    float Kp_stair; // 0.015f
    float Ki_stair; // 0.0005f
    float Kd_stair; // 0.001f

} FuzzyPID;

extern float FuzzyPID_Calc(FuzzyPID *fuzzy_PID);
#endif
