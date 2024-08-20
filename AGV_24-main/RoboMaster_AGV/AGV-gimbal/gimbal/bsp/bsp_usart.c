#include "bsp_usart.h"
#include "main.h"
#include "bsp_usart.h"
#include "string.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

uint8_t vision_rx_buf[2][VISION_RX_LEN_2];
vision_rxfifo_t vision_rxfifo = {0};


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
	hdma_usart1_rx.Instance->NDTR = VISION_RX_LEN_2;
	//enable double memory buffer
	//ʹ��˫������
	SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

	//enable DMA
	//ʹ��DMA
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
}
 
void vision_rx_decode(uint8_t *test_code)
{
	if((uint8_t)test_code[0]==0xA5)
	{
		memset(&vision_rxfifo, 0, sizeof(vision_rxfifo));
		
		memcpy(&vision_rxfifo, test_code,sizeof(vision_rxfifo));
	
	}
}


void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}

vision_rxfifo_t *get_vision_fifo(void)
{
	return &vision_rxfifo;
}
