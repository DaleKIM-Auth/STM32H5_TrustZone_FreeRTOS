/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for GreenTask */
osThreadId_t GreenTaskHandle;
const osThreadAttr_t GreenTask_attributes = {
  .name = "GreenTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for RedTask */
osThreadId_t RedTaskHandle;
const osThreadAttr_t RedTask_attributes = {
  .name = "RedTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GreenTask */
  GreenTaskHandle = osThreadNew(StartGreenTask, NULL, &GreenTask_attributes);

  /* creation of RedTask */
  RedTaskHandle = osThreadNew(StartRedTask, NULL, &RedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END defaultTask */
}

/* USER CODE BEGIN Header_StartGreenTask */
/**
* @brief Function implementing the GreenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGreenTask */
void StartGreenTask(void *argument)
{
  /* USER CODE BEGIN GreenTask */
  uint32_t count = 0;
  (void) argument;

  /* Infinite loop */
  for(;;)
  {
    count = osKernelGetTickCount() + 5000;

    while (count >= osKernelGetTickCount())
    {
      HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);

      osDelay(200);
    }

    HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

    osThreadSuspend(NULL);

    count = osKernelGetTickCount() + 5000;

    while (count >= osKernelGetTickCount()) {
      HAL_GPIO_TogglePin(GREEN_GPIO_Port, GREEN_Pin);

      osDelay(400);
    }
    osThreadResume(RedTaskHandle);
  }
  /* USER CODE END GreenTask */
}

/* USER CODE BEGIN Header_StartRedTask */
/**
* @brief Function implementing the RedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRedTask */
void StartRedTask(void *argument)
{
  uint32_t count;
  (void) argument;
  
  /* USER CODE BEGIN RedTask */
  /* Infinite loop */
  for(;;)
  {
    count = osKernelGetTickCount() + 10000;

    while(count >= osKernelGetTickCount()) {
      HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);

      osDelay(500);
    }

    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);

    osThreadResume(GreenTaskHandle);

    osThreadSuspend(NULL);
  }
  /* USER CODE END RedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

