#ifndef PID_H
#define PID_H
#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};
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
typedef struct
{
    //smc 三参数
    fp32 C;         //c越大，收敛速度越快
    fp32 delta;     //delta(0，1]为饱和函数的边界层，用于抑制抖震
    fp32 eplison;  //用于控制稳态误差，epsilon越大稳态误差越小
	
    fp32 max_out;  //最大输出
    fp32 set;
    fp32 fdb;
	fp32 speed;
    fp32 out;
	fp32 s;       //滑模面
	fp32 K;      //为控制状态不离开滑模面的增益
} smc_type_def;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
float forwardfeed(float in);
extern void SMC_init(smc_type_def *smc,fp32 C,fp32 K, fp32 eplison, fp32 delta ,fp32 max_out);
extern fp32 SMC_calc(smc_type_def *smc, fp32 ref, fp32 set, fp32 speed);
#endif
