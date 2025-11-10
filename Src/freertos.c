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

#include "calibrate_task.h"
//#include "chassis_task.h"
#include "chassisR_task.h"
#include "chassisL_task.h"
#include "observe_task.h"


#include "detect_task.h"
#include "INS_task.h"
#include "led_flow_task.h"


#include "referee_usart_task.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId calibrate_tast_handle;
osThreadId chassisTaskHandle;
osThreadId detect_handle;
//osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId referee_usart_task_handle;










/* USER CODE END Variables */
osThreadId testHandle;
osThreadId CHASSISR_TASKHandle;
osThreadId INS_TASKHandle;
osThreadId CHASSISL_TASKHandle;
osThreadId OBSERVE_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void ChassisR_Task(void const * argument);
void INS_Task(void const * argument);
void ChassisL_Task(void const * argument);
void OBSERVE_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
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
void MX_FREERTOS_Init(void) {
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
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of CHASSISR_TASK */
  osThreadDef(CHASSISR_TASK, ChassisR_Task, osPriorityAboveNormal, 0, 512);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 1024);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of CHASSISL_TASK */
  osThreadDef(CHASSISL_TASK, ChassisL_Task, osPriorityAboveNormal, 0, 512);
  CHASSISL_TASKHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

  /* definition and creation of OBSERVE_TASK */
  osThreadDef(OBSERVE_TASK, OBSERVE_Task, osPriorityHigh, 0, 512);
  OBSERVE_TASKHandle = osThreadCreate(osThread(OBSERVE_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
    osThreadDef(REFEREE, referee_usart_task, osPriorityNormal, 0, 128);//RM����ϵͳ���ݴ���
    referee_usart_task_handle = osThreadCreate(osThread(REFEREE), NULL);

//    osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
//    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

//    osThreadDef(ChassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
//    chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

    osThreadDef(DETECT, detect_task, osPriorityNormal, 0, 256);
    detect_handle = osThreadCreate(osThread(DETECT), NULL);


//    osThreadDef(imuTask, INS_Task, osPriorityRealtime, 0, 1024);
//    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

//    osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
//    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);




  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_ChassisR_Task */
/**
* @brief Function implementing the CHASSISR_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisR_Task */
void ChassisR_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisR_Task */
  /* Infinite loop */
  for(;;)
  {
	ChassisR_task();
  }
  /* USER CODE END ChassisR_Task */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
	

	INS_Init();
	
  /* Infinite loop */
  for(;;)
  {
    INS_task();
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_ChassisL_Task */
/**
* @brief Function implementing the CHASSISL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisL_Task */
void ChassisL_Task(void const * argument)
{
  /* USER CODE BEGIN ChassisL_Task */
  /* Infinite loop */
  for(;;)
  {
	ChassisL_task(); 
  }
  /* USER CODE END ChassisL_Task */
}

/* USER CODE BEGIN Header_OBSERVE_Task */
/**
* @brief Function implementing the OBSERVE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OBSERVE_Task */
void OBSERVE_Task(void const * argument)
{
  /* USER CODE BEGIN OBSERVE_Task */
  /* Infinite loop */
  for(;;)
  {
    observe_task();
  }
  /* USER CODE END OBSERVE_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
