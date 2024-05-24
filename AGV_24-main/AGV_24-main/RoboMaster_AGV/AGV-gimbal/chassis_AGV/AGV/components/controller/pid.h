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
    KI_SEPRATE = 0, //���ַ���
    KD_NO_FULL,     //����ȫ΢��
};
typedef struct
{
    uint8_t mode;
	uint8_t mode_again;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;
	fp32 Kf;

    fp32 max_out;  //������
    fp32 max_iout; //���������
	 

    fp32 set;
    fp32 fdb;

    fp32 out;
	fp32 last_out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	fp32 Last_Dout;
	fp32 Fout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�
    int flag;
	fp32 F_divider;//ǰ������
	fp32 F_out_limit;//ǰ���޷�

} PidTypeDef;
typedef struct
{
    //smc ������
    fp32 C;         //cԽ�������ٶ�Խ��
    fp32 delta;     //delta(0��1]Ϊ���ͺ����ı߽�㣬�������ƶ���
    fp32 eplison;  //���ڿ�����̬��epsilonԽ����̬���ԽС
	
    fp32 max_out;  //������
    fp32 set;
    fp32 fdb;
	fp32 speed;
    fp32 out;
	fp32 s;       //��ģ��
	fp32 K;      //Ϊ����״̬���뿪��ģ�������
} smc_type_def;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
float forwardfeed(float in);
extern void SMC_init(smc_type_def *smc,fp32 C,fp32 K, fp32 eplison, fp32 delta ,fp32 max_out);
extern fp32 SMC_calc(smc_type_def *smc, fp32 ref, fp32 set, fp32 speed);
#endif
