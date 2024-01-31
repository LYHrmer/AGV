/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "shoot_task.h"
#include "led_flow_task.h"
#include "referee_usart_task.h"
#include "vision_task.h"
#include "can_comm_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId chassisTaskHandle;
osThreadId detect_handle;
osThreadId gimbalTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId referee_usart_task_handle;
osThreadId shoot_task_handle;
osThreadId vision_task_handle;
osThreadId INSTaskHandle;
osThreadId can_comm_task_handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void StartINSTask(void const *argument);
/* USER CODE END FunctionPrototypes */

void test_task(void const *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    osThreadDef(ChassisTask, chassis_task, osPriorityHigh, 0, 512);
    chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

    osThreadDef(DETECT, detect_task, osPriorityNormal, 0, 256);
    detect_handle = osThreadCreate(osThread(DETECT), NULL);

    osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 512);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    osThreadDef(INSTask, StartINSTask, osPriorityNormal, 0, 1024);
    INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

    osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);

    osThreadDef(REFEREE, referee_usart_task, osPriorityNormal, 0, 128);
    referee_usart_task_handle = osThreadCreate(osThread(REFEREE), NULL);

    osThreadDef(SHOOT, shoot_task, osPriorityHigh, 0, 512);
    shoot_task_handle = osThreadCreate(osThread(SHOOT), NULL);

    osThreadDef(VISION, vision_task, osPriorityNormal, 0, 512);
    vision_task_handle = osThreadCreate(osThread(VISION), NULL);

    osThreadDef(CanComm, can_comm_task, osPriorityHigh, 0, 512);
    can_comm_task_handle = osThreadCreate(osThread(CanComm), NULL);

    /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_test_task */
/**
 * @brief  Function implementing the test thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_test_task */
__weak void test_task(void const *argument)
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();

    /* USER CODE BEGIN test_task */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartINSTask(void const *argument)
{
    /* USER CODE BEGIN StartINSTask */
    INS_Init();
    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
    /* USER CODE END StartINSTask */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
