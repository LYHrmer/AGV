/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#define int_abs(x) ((x) > 0 ? (x) : (-x))
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
void ABSLimit(float *a, float abs_max)
{
	if(*a > abs_max)
		*a = abs_max;
	if(*a < -abs_max)
		*a = -abs_max;
}
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
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/**********************************************************************************************************
*功能说明: PID+各种优化

*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[8], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->Kf = PID[3];
    pid->F_divider = PID[6];
    pid->F_out_limit = PID[7];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
	
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
	
}

fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    static fp32 M = 0.10;
    static fp32 a = 0.8;
    pid->last_out = pid->out;
    pid->Last_Dout = pid->Dout;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        if (int_abs(set) > int_abs(pid->F_divider))
        {
            pid->Fout = pid->Kf * set;
            ABSLimit(&(pid->Fout), pid->F_out_limit);
        }
        else if (pid->F_divider != 0)
        {
            pid->Fout = int_abs(set) / (int_abs(pid->F_divider) + 1) * pid->Kf * set;
        }

        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        //		if(pid->mode_again==KI_SEPRATE)      //积分分离
        //		{
        //			if( int_abs(pid->error[0])>M)
        //			{
        //				pid->Iout=0.0f;
        //			}
        //
        //		}
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);

        if (pid->mode_again == KD_NO_FULL) // 不完全微分
        {
            pid->Dout = pid->Kd * (1 - a) * pid->Dbuf[0] + a * pid->Last_Dout - pid->Kd * (1 - a) * pid->Dbuf[1];
        }
        else
        {
            pid->Dout = pid->Kd * pid->Dbuf[0];
        }

        pid->out = pid->Pout + pid->Iout + pid->Dout + pid->Fout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        if (pid->mode_again == KI_SEPRATE) // 积分分离
        {
            if (int_abs(pid->error[0]) > M)
            {
                pid->Iout = 0.0f;
            }
        }
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

        if (pid->mode_again == KD_NO_FULL || pid->flag == 1) // 不完全微分
        {
            pid->Dout = pid->Kd * (1 - a) * pid->Dbuf[0] + a * pid->Last_Dout - pid->Kd * (1 - a) * pid->Dbuf[1];
        }
        else
        {
            pid->Dout = pid->Kd * pid->Dbuf[0];
        }
        pid->out += pid->Pout + pid->Iout - pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

    return pid->out;
}

void PID_Clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


/**
  前馈控制
  采样周期: T = 0.001;
  转动惯量: J =1；
  摩擦系数: f = 1；
	角速度/力矩：G(s) = 1/(s + 1)
  前馈环节：Gf(s) = s + 1
	输出: out = in' + in = (in - last_in)/T + in
  */
float forwardfeed(float in)
{
	static float last_in = 0;
	float T = 0.001;
	float out;
	out = (in - last_in)/T + in;
	last_in = in ;
	return out ;
}

/**********************************************************************************************************
*函 数 名: FeedForward_Calc
*功能说明: 前馈算法
*形    参: PID_Struct *P  PID参数结构体
  *        ActualValue    PID计算反馈量（当前真实检测值）
*返 回 值: PID反馈计算输出值
**********************************************************************************************************/
float FeedForward_Calc(FeedForward_Typedef *FF)
{
	  FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
	  FF->Last_DeltIn = FF->Now_DeltIn;
    return LIMIT_MAX_MIN(FF->Out,FF->OutMax,-FF->OutMax);
}

/*
* 模糊PID部分
*/
static const float fuzzyRuleKp[7][7] = {
    PL, PL, PM, PM, PS, ZE, ZE,
    PL, PL, PM, PS, PS, ZE, NS,
    PM, PM, PM, PS, ZE, NS, NS,
    PM, PM, PS, ZE, NS, NM, NM,
    PS, PS, ZE, NS, NS, NM, NM,
    PS, ZE, NS, NM, NM, NM, NL,
    ZE, ZE, NM, NM, NM, NL, NL};

static const float fuzzyRuleKi[7][7] = {
    NL, NL, NM, NM, NS, ZE, ZE,
    NL, NL, NM, NS, NS, ZE, ZE,
    NL, NM, NS, NS, ZE, PS, PS,
    NM, NM, NS, ZE, PS, PM, PM,
    NS, NS, ZE, PS, PS, PM, PL,
    ZE, ZE, PS, PS, PM, PL, PL,
    ZE, ZE, PS, PM, PM, PL, PL};

static const float fuzzyRuleKd[7][7] = {
    PS, NS, NL, NL, NL, NM, PS,
    PS, NS, NL, NM, NM, NS, ZE,
    ZE, NS, NM, NM, NS, NS, ZE,
    ZE, NS, NS, NS, NS, NS, ZE,
    ZE, ZE, ZE, ZE, ZE, ZE, ZE,
    PL, NS, PS, PS, PS, PS, PL,
    PL, PM, PM, PM, PS, PS, PL};

//关键算法
void fuzzy(FuzzyPID *fuzzy_PID)
{
    float e = fuzzy_PID->PreError / fuzzy_PID->stair;
    float ec = (fuzzy_PID->Out - fuzzy_PID->Out_last) / fuzzy_PID->stair;
    short etemp, ectemp;
    float eLefttemp, ecLefttemp; //隶属度
    float eRighttemp, ecRighttemp;

    short eLeftIndex, ecLeftIndex; //标签
    short eRightIndex, ecRightIndex;

    //模糊化
    if (e >= PL)
        etemp = PL; //超出范围
    else if (e >= PM)
        etemp = PM;
    else if (e >= PS)
        etemp = PS;
    else if (e >= ZE)
        etemp = ZE;
    else if (e >= NS)
        etemp = NS;
    else if (e >= NM)
        etemp = NM;
    else if (e >= NL)
        etemp = NL;
    else
        etemp = 2 * NL;

    if (etemp == PL)
    {
        //计算E隶属度
        eRighttemp = 0; //右溢出
        eLefttemp = 1;

        //计算标签
        eLeftIndex = 6;
        eRightIndex = 6;
    }
    else if (etemp == 2 * NL)
    {

        //计算E隶属度
        eRighttemp = 1; //左溢出
        eLefttemp = 0;

        //计算标签
        eLeftIndex = 0;
        eRightIndex = 0;
    }
    else
    {

        //计算E隶属度
        eRighttemp = (e - etemp); //线性函数作为隶属函数
        eLefttemp = (1 - eRighttemp);

        //计算标签
        eLeftIndex = (short)(etemp - NL); //例如 etemp=2.5，NL=-3，那么得到的序列号为5  【0 1 2 3 4 5 6】
        eRightIndex = (short)(eLeftIndex + 1);
    }

    if (ec >= PL)
        ectemp = PL;
    else if (ec >= PM)
        ectemp = PM;
    else if (ec >= PS)
        ectemp = PS;
    else if (ec >= ZE)
        ectemp = ZE;
    else if (ec >= NS)
        ectemp = NS;
    else if (ec >= NM)
        ectemp = NM;
    else if (ec >= NL)
        ectemp = NL;
    else
        ectemp = 2 * NL;

    if (ectemp == PL)
    {
        //计算EC隶属度
        ecRighttemp = 0; //右溢出
        ecLefttemp = 1;

        ecLeftIndex = 6;
        ecRightIndex = 6;
    }
    else if (ectemp == 2 * NL)
    {
        //计算EC隶属度
        ecRighttemp = 1;
        ecLefttemp = 0;

        ecLeftIndex = 0;
        ecRightIndex = 0;
    }
    else
    {
        //计算EC隶属度
        ecRighttemp = (ec - ectemp);
        ecLefttemp = (1 - ecRighttemp);

        ecLeftIndex = (short)(ectemp - NL);
        ecRightIndex = (short)(eLeftIndex + 1);
    }

    /*************************************反模糊*************************************/

    fuzzy_PID->dKp = fuzzy_PID->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);

    fuzzy_PID->dKi = fuzzy_PID->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

    fuzzy_PID->dKd = fuzzy_PID->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
}

float FuzzyPID_Calc(FuzzyPID *fuzzy_PID)
{

    fuzzy_PID->LastError = fuzzy_PID->PreError;

    if ((ABS(fuzzy_PID->PreError) < fuzzy_PID->DeadZone)) //死区控制
    {
        fuzzy_PID->PreError = 0.0f;
    }
    else
    {
        fuzzy_PID->PreError = fuzzy_PID->SetPoint - fuzzy_PID->ActualValue;
    }

    fuzzy(fuzzy_PID); //模糊调整  kp,ki,kd   形参1当前误差，形参2前后误差的差值

    float Kp = fuzzy_PID->Kp0 + fuzzy_PID->dKp, Ki = fuzzy_PID->Ki0 + fuzzy_PID->dKi, Kd = fuzzy_PID->Kd0 + fuzzy_PID->dKd; // PID均模糊

    //微分先行
    float DM = Kd * (fuzzy_PID->Out - fuzzy_PID->Out_last); //微分先行
    //变速积分
    if (ABS(fuzzy_PID->PreError) < fuzzy_PID->I_L)
    {
        //梯形积分
        fuzzy_PID->SumError += (fuzzy_PID->PreError + fuzzy_PID->LastError) / 2;
        fuzzy_PID->SumError = LIMIT_MAX_MIN(fuzzy_PID->SumError, fuzzy_PID->IMax, -fuzzy_PID->IMax);    //限幅
    }
    else if (ABS(fuzzy_PID->PreError) < fuzzy_PID->I_U)
    {
        //梯形积分
        fuzzy_PID->SumError += (fuzzy_PID->PreError + fuzzy_PID->LastError) / 2 * (fuzzy_PID->PreError - fuzzy_PID->I_L) / (fuzzy_PID->I_U - fuzzy_PID->I_L);
        fuzzy_PID->SumError = LIMIT_MAX_MIN(fuzzy_PID->SumError, fuzzy_PID->IMax, -fuzzy_PID->IMax);
    }

    fuzzy_PID->POut = Kp * fuzzy_PID->PreError;

    fuzzy_PID->IOut = Ki * fuzzy_PID->SumError;

    //不完全微分
    fuzzy_PID->DOut_last = fuzzy_PID->DOut;
    fuzzy_PID->DOut = DM * fuzzy_PID->RC_DF + fuzzy_PID->DOut_last * (1 - fuzzy_PID->RC_DF);

    fuzzy_PID->Out_last = fuzzy_PID->Out;
    fuzzy_PID->Out = LIMIT_MAX_MIN(fuzzy_PID->POut + fuzzy_PID->IOut + fuzzy_PID->DOut, fuzzy_PID->OutMax, -fuzzy_PID->OutMax);

    return fuzzy_PID->Out;
}

