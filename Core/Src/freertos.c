/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "app_touchgfx.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId aluMainHandle;
osThreadId aluSubProgressHandle;
osMessageQId alu_temp_listHandle;
osSemaphoreId alu_temperatureHandle;
osSemaphoreId alu_thresholdHandle;
osSemaphoreId alu_screenHandle;
osSemaphoreId alu_chooseHandle;
osSemaphoreId alu_savenameHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTouchGFX(void const * argument);
void AluMain(void const * argument);
void AluSubProgress(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of alu_temperature */
  osSemaphoreDef(alu_temperature);
  alu_temperatureHandle = osSemaphoreCreate(osSemaphore(alu_temperature), 1);

  /* definition and creation of alu_threshold */
  osSemaphoreDef(alu_threshold);
  alu_thresholdHandle = osSemaphoreCreate(osSemaphore(alu_threshold), 1);

  /* definition and creation of alu_screen */
  osSemaphoreDef(alu_screen);
  alu_screenHandle = osSemaphoreCreate(osSemaphore(alu_screen), 1);

  /* definition and creation of alu_choose */
  osSemaphoreDef(alu_choose);
  alu_chooseHandle = osSemaphoreCreate(osSemaphore(alu_choose), 1);

  /* definition and creation of alu_savename */
  osSemaphoreDef(alu_savename);
  alu_savenameHandle = osSemaphoreCreate(osSemaphore(alu_savename), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of alu_temp_list */
  osMessageQDef(alu_temp_list, 100, uint32_t);
  alu_temp_listHandle = osMessageCreate(osMessageQ(alu_temp_list), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartTouchGFX, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of aluMain */
  osThreadDef(aluMain, AluMain, osPriorityRealtime, 0, 4096);
  aluMainHandle = osThreadCreate(osThread(aluMain), NULL);

  /* definition and creation of aluSubProgress */
  osThreadDef(aluSubProgress, AluSubProgress, osPriorityHigh, 0, 2048);
  aluSubProgressHandle = osThreadCreate(osThread(aluSubProgress), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTouchGFX */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTouchGFX */
__weak void StartTouchGFX(void const * argument)
{
  /* USER CODE BEGIN StartTouchGFX */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTouchGFX */
}

/* USER CODE BEGIN Header_AluMain */
/**
* @brief Function implementing the aluMain thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AluMain */
__weak void AluMain(void const * argument)
{
  /* USER CODE BEGIN AluMain */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AluMain */
}

/* USER CODE BEGIN Header_AluSubProgress */
/**
* @brief Function implementing the aluSubProgress thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AluSubProgress */
__weak void AluSubProgress(void const * argument)
{
  /* USER CODE BEGIN AluSubProgress */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AluSubProgress */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
