/**********************************************************************************************************
 * @文件     tim4.c
 * @说明     tim4初始化(用于看门狗)
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
#include "tim4.h"
/**********************************************************************************************************
*函 数 名: TIM4_Configuration
*功能说明: TIM4初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void TIM4_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 

    TIM_TimeBaseInitStructure.TIM_Period = 1000-1; 	
    TIM_TimeBaseInitStructure.TIM_Prescaler = 71;  
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM4, ENABLE); 

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************************************************************
*函 数 名: TIM4_IRQHandler
*功能说明: tim4中断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//volatile uint32_t CPU_RunTime = 0UL;
uint16_t spi_timer=0;
float start_time_SD=0;
char SD_init_flag=1;
extern TaskHandle_t User_Tasks[TASK_NUM];
void  TIM4_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM4, TIM_IT_Update) != RESET ) 
	{	
    //CPU_RunTime++;
		//中断内判断如果SPI初始化失败则关闭SD卡任务
		if(SD_init_flag==0)
		{
			start_time_SD+=GetDeltaT(&spi_timer);
			IWDG_Feed();//喂狗	
			if(start_time_SD > 5.0){
				SD_init_flag = 1;
				vTaskDelete(User_Tasks[SDCARD_TASK]);
			}
		}
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);  		 
	}		 	
}

