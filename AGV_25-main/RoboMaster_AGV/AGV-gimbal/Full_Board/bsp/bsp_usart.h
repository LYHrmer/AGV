#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"

#define VISION_RX_LEN_2 50u
#define VISION_RX_LEN 25u
typedef struct 
	{
	uint8_t header;
	
	fp32 vx;
	fp32 vy;
	fp32 vz;
	
	fp32 ang_z;
	fp32 ang_y;
	fp32 ang_x;
	
} vision_rxfifo_t;




extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);



extern vision_rxfifo_t vision_rxfifo;
extern uint8_t vision_rx_buf[2][VISION_RX_LEN_2];


extern void vision_init(void);
extern void vision_rx_decode(uint8_t *test_code);
extern vision_rxfifo_t *get_vision_fifo(void);
#endif