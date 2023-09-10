#ifndef RM_USART_H
#define RM_USART_H
#include"main.h"
typedef struct {
	fp32 yaw_fifo;//YAW信息
	fp32 pitch_fifo;//PITCH信息
	fp32 yaw_speed_fifo;//YAW速度信息
	fp32 yaw_disdance;//YAW速度信息
	int32_t pitch_speed_fifo;//PITCH速度信息
	int rx_change_flag;//识别目标切换
	int rx_flag;//识别到目标	
	int rx_update_flag;//视觉更新
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
 * @brief 获取存储上位机数据的结构体指针
 * 
 * @return vision_rxfifo_t* 
 */
vision_rxfifo_t * get_vision_rxfifo_point(void);

extern void vision_init(void);
extern void vision_rx_decode(uint8_t *test_code);
//void USAR_UART_IDLECallback(UART_HandleTypeDef *huart);

#endif
