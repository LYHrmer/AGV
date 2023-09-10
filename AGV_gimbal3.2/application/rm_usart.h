#ifndef RM_USART_H
#define RM_USART_H
#include"main.h"
typedef struct {
	fp32 yaw_fifo;//YAW��Ϣ
	fp32 pitch_fifo;//PITCH��Ϣ
	fp32 yaw_speed_fifo;//YAW�ٶ���Ϣ
	fp32 yaw_disdance;//YAW�ٶ���Ϣ
	int32_t pitch_speed_fifo;//PITCH�ٶ���Ϣ
	int rx_change_flag;//ʶ��Ŀ���л�
	int rx_flag;//ʶ��Ŀ��	
	int rx_update_flag;//�Ӿ�����
} vision_rxfifo_t;

#define VISION_RX_LEN_2 58u
#define VISION_RX_LEN 29u

#define HEAD0_BASE 0
#define HEAD1_BASE 4
#define YAW_FIFO_BASE 8
#define PITCH_FIFO_BASE 12
#define YAW_SPEED_FIFO_BASE 16
#define PITCH_SPEED_FIFO_BASE 20
#define CHANGE_FLAG_FIFO_BASE 21

extern vision_rxfifo_t vision_rxfifo;
//extern uint8_t test_code[24];
extern uint8_t vision_rx_buf[2][VISION_RX_LEN_2];


/**
 * @brief ��ȡ�洢��λ�����ݵĽṹ��ָ��
 * 
 * @return vision_rxfifo_t* 
 */
vision_rxfifo_t * get_vision_rxfifo_point(void);

extern void vision_init(void);
extern void vision_rx_decode(uint8_t *test_code);
//void USAR_UART_IDLECallback(UART_HandleTypeDef *huart);

#endif
