/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define RUDDER_CAN hcan2
#define CAPID_CAN hcan1
#define SHOOT_CAN hcan2


#define CAN_FEEDBACK_FREAM_ID_A       0x222
#define CAN_FEEDBACK_FREAM_ID_B       0x223
#define CAN_CTRL_FREAM_ID             0x224       //CAN֡ID��
#define CAN_PITCHANGLE_ID  0x213
typedef enum
{
    BATTERY = 1,
    CAPACITY,
    OUT_OFF,
}power_source_enum; 
typedef struct 
{
    uint16_t    input_voltage;      //�����ѹ
    uint16_t    current;            //�������
    uint16_t    cap_voltage;        //���ݵ�ѹ
    uint8_t     p_set;              //�趨����
    uint8_t     crc_checksum;
}can_feedback_a_typedef;  //CAN��������A
typedef struct 
{
    uint16_t    output_voltage;     //�����ѹ
    uint8_t     power_source:7;       //��Դ��Դ
    uint8_t     out_auto_en:1;     //��������Ƿ��Զ�����
    uint8_t     nc1;                
    uint8_t     nc2;                
    uint8_t     nc3;                
    uint8_t     nc4;                //��
    uint8_t     crc_checksum;
}can_feedback_b_typedef;  //CAN��������B

typedef struct 
{
    uint8_t     p_set;                  //�趨����
    uint8_t     power_source:7;           //���Ƶ�Դ��Դ-1ǰ����Դ  2������  3����ر�
    uint8_t     out_auto_en:1;     //��������Ƿ��Զ�����
    uint16_t    freq_feedback:15;       //����Ƶ�ʣ�Ĭ��100
    uint16_t    wireless_en:1;          //���߿���
    uint8_t     nc1;                
    uint8_t     nc2;                
    uint8_t     nc3;                    //��
    uint8_t     crc_checksum;
}can_control_typedef;  //CAN��������
//���г���
typedef enum
{
	  CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_Forward_L_ID = 0x205,
    CAN_Forward_R_ID = 0x206,
    CAN_BACK_L_ID=0x207,
	  CAN_BACK_R_ID=0x208,
   
    
	  CAN_CAPID=0x211,
	  CAN_POWERID=0x212,
	  CAN_GIMBAL_CALL_BACK_ID=0x218,
		CAN_REFEREE_ID=0x219,
} can_msg_id_e;


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    
    fp32 invot;
    fp32 capvot;
    fp32 current;
    fp32 power;

} cap_measure_t;

extern cap_measure_t get_cap;
extern can_feedback_a_typedef get_capA;
extern can_feedback_b_typedef get_capB;

/**
  * @brief          ���ͳ������ݳ�繦��
  * @param[in]      temPower��0x210������繦��
  * @retval         none
  */
//void CAN_CMD_cap(uint8_t temPower);
void CAN_CMD_cap(int16_t temPower);//Ϫ��
/**
  * @brief          ���ͳ������ݵ�ѹ
  */
void CAN_cmd_gimbal(int16_t capvot, int16_t rev);
/**
  * @brief          ���Ͷ������Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      forward_L: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      forward_R: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      back_L: (0x207) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      back_R: (0x208) 6020������Ƶ���, ��Χ [-30000,30000]
  * @retval         none
  */
extern void CAN_cmd_rudder(int16_t forward_L, int16_t forward_R, int16_t back_L,int16_t back_R);



/**
  * @brief          �����ֵ�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


/** 
  * @brief          ���ض��� 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_Forward_L_motor_measure_point(void);
extern const motor_measure_t *get_Forward_R_motor_measure_point(void);
extern const motor_measure_t *get_Back_R_motor_measure_point(void);
extern const motor_measure_t *get_Back_L_motor_measure_point(void);
 
 
/**
  * @brief          �����ֵ�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


/**
  * @brief          ���س�����������ָ��
  * @param[in]      ��
  * @retval         �������ָ��
  */
extern  cap_measure_t *get_cap_measure_point(void);
#endif
