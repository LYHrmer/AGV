/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
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
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
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


/**********************************************************************************************************
*����˵��: PID+�����Ż�

*�� �� ֵ: PID�����������ֵ
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
        //		if(pid->mode_again==KI_SEPRATE)      //���ַ���
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

        if (pid->mode_again == KD_NO_FULL) // ����ȫ΢��
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
        if (pid->mode_again == KI_SEPRATE) // ���ַ���
        {
            if (int_abs(pid->error[0]) > M)
            {
                pid->Iout = 0.0f;
            }
        }
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);

        if (pid->mode_again == KD_NO_FULL || pid->flag == 1) // ����ȫ΢��
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

/**********************************************************************************************************
*�� �� ��: FeedForward_Calc
*����˵��: ǰ���㷨
*��    ��: PID_Struct *P  PID�����ṹ��
  *        ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
*�� �� ֵ: PID�����������ֵ
**********************************************************************************************************/
float FeedForward_Calc(FeedForward_Typedef *FF)
{
	  FF->Out = FF->Now_DeltIn*FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn)*FF->K2;
	  FF->Last_DeltIn = FF->Now_DeltIn;
    return LIMIT_MAX_MIN(FF->Out,FF->OutMax,-FF->OutMax);
}

/*
* ģ��PID����
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

//�ؼ��㷨
void fuzzy(FuzzyPID *fuzzy_PID)
{
    float e = fuzzy_PID->PreError / fuzzy_PID->stair;
    float ec = (fuzzy_PID->Out - fuzzy_PID->Out_last) / fuzzy_PID->stair;
    short etemp, ectemp;
    float eLefttemp, ecLefttemp; //������
    float eRighttemp, ecRighttemp;

    short eLeftIndex, ecLeftIndex; //��ǩ
    short eRightIndex, ecRightIndex;

    //ģ����
    if (e >= PL)
        etemp = PL; //������Χ
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
        //����E������
        eRighttemp = 0; //�����
        eLefttemp = 1;

        //�����ǩ
        eLeftIndex = 6;
        eRightIndex = 6;
    }
    else if (etemp == 2 * NL)
    {

        //����E������
        eRighttemp = 1; //�����
        eLefttemp = 0;

        //�����ǩ
        eLeftIndex = 0;
        eRightIndex = 0;
    }
    else
    {

        //����E������
        eRighttemp = (e - etemp); //���Ժ�����Ϊ��������
        eLefttemp = (1 - eRighttemp);

        //�����ǩ
        eLeftIndex = (short)(etemp - NL); //���� etemp=2.5��NL=-3����ô�õ������к�Ϊ5  ��0 1 2 3 4 5 6��
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
        //����EC������
        ecRighttemp = 0; //�����
        ecLefttemp = 1;

        ecLeftIndex = 6;
        ecRightIndex = 6;
    }
    else if (ectemp == 2 * NL)
    {
        //����EC������
        ecRighttemp = 1;
        ecLefttemp = 0;

        ecLeftIndex = 0;
        ecRightIndex = 0;
    }
    else
    {
        //����EC������
        ecRighttemp = (ec - ectemp);
        ecLefttemp = (1 - ecRighttemp);

        ecLeftIndex = (short)(ectemp - NL);
        ecRightIndex = (short)(eLeftIndex + 1);
    }

    /*************************************��ģ��*************************************/

    fuzzy_PID->dKp = fuzzy_PID->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);

    fuzzy_PID->dKi = fuzzy_PID->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

    fuzzy_PID->dKd = fuzzy_PID->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex] + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
}

float FuzzyPID_Calc(FuzzyPID *fuzzy_PID)
{

    fuzzy_PID->LastError = fuzzy_PID->PreError;

    if ((ABS(fuzzy_PID->PreError) < fuzzy_PID->DeadZone)) //��������
    {
        fuzzy_PID->PreError = 0.0f;
    }
    else
    {
        fuzzy_PID->PreError = fuzzy_PID->SetPoint - fuzzy_PID->ActualValue;
    }

    fuzzy(fuzzy_PID); //ģ������  kp,ki,kd   �β�1��ǰ���β�2ǰ�����Ĳ�ֵ

    float Kp = fuzzy_PID->Kp0 + fuzzy_PID->dKp, Ki = fuzzy_PID->Ki0 + fuzzy_PID->dKi, Kd = fuzzy_PID->Kd0 + fuzzy_PID->dKd; // PID��ģ��

    //΢������
    float DM = Kd * (fuzzy_PID->Out - fuzzy_PID->Out_last); //΢������
    //���ٻ���
    if (ABS(fuzzy_PID->PreError) < fuzzy_PID->I_L)
    {
        //���λ���
        fuzzy_PID->SumError += (fuzzy_PID->PreError + fuzzy_PID->LastError) / 2;
        fuzzy_PID->SumError = LIMIT_MAX_MIN(fuzzy_PID->SumError, fuzzy_PID->IMax, -fuzzy_PID->IMax);    //�޷�
    }
    else if (ABS(fuzzy_PID->PreError) < fuzzy_PID->I_U)
    {
        //���λ���
        fuzzy_PID->SumError += (fuzzy_PID->PreError + fuzzy_PID->LastError) / 2 * (fuzzy_PID->PreError - fuzzy_PID->I_L) / (fuzzy_PID->I_U - fuzzy_PID->I_L);
        fuzzy_PID->SumError = LIMIT_MAX_MIN(fuzzy_PID->SumError, fuzzy_PID->IMax, -fuzzy_PID->IMax);
    }

    fuzzy_PID->POut = Kp * fuzzy_PID->PreError;

    fuzzy_PID->IOut = Ki * fuzzy_PID->SumError;

    //����ȫ΢��
    fuzzy_PID->DOut_last = fuzzy_PID->DOut;
    fuzzy_PID->DOut = DM * fuzzy_PID->RC_DF + fuzzy_PID->DOut_last * (1 - fuzzy_PID->RC_DF);

    fuzzy_PID->Out_last = fuzzy_PID->Out;
    fuzzy_PID->Out = LIMIT_MAX_MIN(fuzzy_PID->POut + fuzzy_PID->IOut + fuzzy_PID->DOut, fuzzy_PID->OutMax, -fuzzy_PID->OutMax);

    return fuzzy_PID->Out;
}

