/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "motor_dm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t Counter = 0;
Motor_DM_Normal motor_j4310;
Motor_DM_Normal motor_j8009;
Enum_Motor_DM_Control_Method Motor_DM_Control_Method;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief CAN报文回调函数
 *
 * @param Rx_Buffer CAN接收的信息结构体
 */
void CAN_Motor_Call_Back(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.StdId)
    {
        // case (0x201):
        // {
        //     motor_3508.CAN_RxCpltCallback(Rx_Buffer->Data);
        // }
        // break;
        // case (0x205):
        // {
        //     motor_6020.CAN_RxCpltCallback(Rx_Buffer->Data);
        // }
        // break;
//        case (0x11):
//        {
//			Motor_DM_Normal_CAN_RxCpltCallback(&motor_j8009,Rx_Buffer->Data);
//			break;
//        }
		case (0x11):
        {
            Motor_DM_Normal_CAN_RxCpltCallback(&motor_j4310,Rx_Buffer->Data);
			break;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
	Motor_DM_Control_Method = Motor_DM_Control_Method_NORMAL_MIT;
    //can_filter_init();
	CAN_Init(&hcan1, CAN_Motor_Call_Back);
	Motor_DM_Normal_Init(&motor_j4310,&hcan1,0x11,0x01,Motor_DM_Control_Method_NORMAL_MIT,12.5f,25.0f,10.0f,10.261194f);
//	Motor_DM_Normal_Init(&motor_j8009,&hcan1,0x11,0x01,Motor_DM_Control_Method_NORMAL_MIT,12.5f,45.0f,54.0f,41.044777f);
//	HAL_Delay(1500);
	
//	motor_j4310.Control_Angle = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Motor_DM_Normal_CAN_Send_Enable(&motor_j4310);
	  
//	  Motor_DM_Normal_CAN_Send_Enable(&motor_j8009);
	  
	  
	  
//	  Motor_DM_Normal_CAN_Send_Disable(&motor_j4310);
	  
	  Counter = Counter + PI/2;
	  if(Counter > 2*PI)
	  {
		  Counter = 0;
	  }
	  
	  Counter++;
//	  motor_j4310.Control_Angle = ((Counter / 3000) % 2 == 0 ? 0.0f : PI);
	  motor_j4310.Control_Angle = (Counter);
	  motor_j4310.Control_Omega = 0.0f;
	  motor_j4310.Control_Current = 0.0f;
	  motor_j4310.K_P = 40.0f;
	  motor_j4310.K_D = 1.0f;
	  Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&motor_j4310);
//	  
//	  // 保持存活
//      static uint32_t Counter_KeepAlive = 0;
//      if (Counter_KeepAlive++ > 100)
//      {
//          Counter_KeepAlive = 0;
//          
//          Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&motor_j4310);
//		  //Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&motor_j8009);
//      }
//	  
//	  // 保持存活
//      static uint32_t Counter_KeepAlive_1 = 0;
//      if (Counter_KeepAlive_1++ > 100)
//      {
//          Counter_KeepAlive_1 = 0;
//          
//          //Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&motor_j4310);
//		  Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&motor_j8009);
//      }
////	  
//	  motor_j8009.Control_Angle = ((Counter / 3000) % 2 == 0 ? 0.0f : PI);
//	  motor_j8009.Control_Omega = 0.0f;
//	  motor_j8009.Control_Current = 0.0f;
//	  motor_j8009.K_P = 56.28f;
//	  motor_j8009.K_D = 1.87f;
//	  Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&motor_j8009);
	  
	  
		HAL_Delay(1);
    /* USER CODE END WHILE */
        
    /* USER CODE BEGIN 3 */
        
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
