#include "main.h"
#include "usart.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
#include "rm_usart.h"
#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t vision_rx_buf[2][VISION_RX_LEN_2];

vision_rxfifo_t vision_rxfifo = {0};

uint16_t base=7;
uint8_t i=0;

void vision_init(void)
{
	    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(vision_rx_buf[0]);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(vision_rx_buf[1]);
    //data length
    //���ݳ���
    hdma_usart1_rx.Instance->NDTR = 48u;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

}


void vision_rx_decode(uint8_t *test_code)
{
	if((fp32)((test_code[HEAD0_BASE+0] << 8*3) | (test_code[HEAD0_BASE+1] << 8*2)
					| (test_code[HEAD0_BASE+2] << 8*1) | (test_code[HEAD0_BASE+3] << 8*0)) == 0x34)
	{
		if((fp32)((test_code[HEAD1_BASE+0] << 8*3) | (test_code[HEAD1_BASE+1] << 8*2)
						| (test_code[HEAD1_BASE+2] << 8*1) | (test_code[HEAD1_BASE+3] << 8*0)) == 0x43)
		{
			vision_rxfifo.rx_flag = 1;
			
			vision_rxfifo.yaw_fifo 	 = (test_code[YAW_FIFO_BASE+0] << 8*3) | (test_code[YAW_FIFO_BASE+1] << 8*2)
															 | (test_code[YAW_FIFO_BASE+2] << 8*1) | (test_code[YAW_FIFO_BASE+3] << 8*0);
			vision_rxfifo.pitch_fifo = (test_code[PITCH_FIFO_BASE+0] << 8*3) | (test_code[PITCH_FIFO_BASE+1] << 8*2)
															 | (test_code[PITCH_FIFO_BASE+2] << 8*1) | (test_code[PITCH_FIFO_BASE+3] << 8*0);
			vision_rxfifo.yaw_disdance  = (test_code[YAW_SPEED_FIFO_BASE+0] << 8*3) | (test_code[YAW_SPEED_FIFO_BASE+1] << 8*2)
																		| (test_code[YAW_SPEED_FIFO_BASE+2] << 8*1) | (test_code[YAW_SPEED_FIFO_BASE+3] << 8*0);
			vision_rxfifo.pitch_speed_fifo  = (test_code[PITCH_SPEED_FIFO_BASE+0] << 8*3) | (test_code[PITCH_SPEED_FIFO_BASE+1] << 8*2)
																			| (test_code[PITCH_SPEED_FIFO_BASE+2] << 8*1) | (test_code[PITCH_SPEED_FIFO_BASE+3] << 8*0);
			vision_rxfifo.rx_change_flag = test_code[CHANGE_FLAG_FIFO_BASE+3];
			
			vision_rxfifo.rx_update_flag = 1;
			///
			vision_rxfifo.yaw_fifo = (fp32)vision_rxfifo.yaw_fifo / 10000.0f;
			vision_rxfifo.pitch_fifo = (fp32)vision_rxfifo.pitch_fifo / 10000.0f;
			vision_rxfifo.yaw_disdance=(fp32)vision_rxfifo.yaw_disdance/10000.0f;
		}
	}
	else if((fp32)((test_code[HEAD0_BASE+0] << 8*3) | (test_code[HEAD0_BASE+1] << 8*2)
					| (test_code[HEAD0_BASE+2] << 8*1) | (test_code[HEAD0_BASE+3] << 8*0)) == 0x66)
	{
		if((fp32)((test_code[HEAD1_BASE+0] << 8*3) | (test_code[HEAD1_BASE+1] << 8*2)
						| (test_code[HEAD1_BASE+2] << 8*1) | (test_code[HEAD1_BASE+3] << 8*0)) == 0x66)
		{
			vision_rxfifo.rx_flag 	= 0;
			
			vision_rxfifo.yaw_fifo 	= 180;
			vision_rxfifo.pitch_fifo = 180;
			vision_rxfifo.yaw_disdance = 0;
			vision_rxfifo.pitch_speed_fifo  = 0;
			vision_rxfifo.rx_change_flag	= 0;
			
			vision_rxfifo.rx_update_flag = 1;
		}
	}
}

vision_rxfifo_t * get_vision_rxfifo_point(void)
{
    return &vision_rxfifo;
}



void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
	}
	else if(USART1->SR & UART_FLAG_IDLE)//�����ж�
	{
		static uint16_t this_time_rx_len = 0;//��Ž��յ������ݳ���

		__HAL_UART_CLEAR_PEFLAG(&huart1);

		if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//ʧЧDMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//get receive data length, length = set_data_length - remain_length
			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = VISION_RX_LEN_2 - hdma_usart1_rx.Instance->NDTR;

			//reset set_data_lenght
			//�����趨���ݳ���
			hdma_usart1_rx.Instance->NDTR = VISION_RX_LEN_2;

			//t memory buffer 1
			//�趨������1
			hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
			
			//enable DMA
			//ʹ��DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			if(this_time_rx_len == 24)
			{
				vision_rx_decode(vision_rx_buf[0]);
			}
		}else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//ʧЧDMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//get receive data length, length = set_data_length - remain_length
			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = VISION_RX_LEN_2 - hdma_usart1_rx.Instance->NDTR;

			//reset set_data_lenght
			//�����趨���ݳ���
			hdma_usart1_rx.Instance->NDTR = VISION_RX_LEN_2;

			//t memory buffer 0
			//�趨������0
			DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
			
			//enable DMA
			//ʹ��DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			if(this_time_rx_len == 24)
			{
				vision_rx_decode(vision_rx_buf[1]);
			}
		}
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

