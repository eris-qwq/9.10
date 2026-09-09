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
#include "task_init.h"
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

osThreadId defaultTaskHandle;//默认任务句柄 default：默认。
osThreadId KeyTaskHandle;//按键任务句柄
uint32_t KeyTaskBuffer[ 128 ];// 按键任务的静态栈
osStaticThreadDef_t KeyTaskControlBlock;// 按键任务的静态任务控制块
osThreadId LcdTaskHandle;//LCD任务句柄
uint32_t LcdTaskBuffer[ 256 ];
osStaticThreadDef_t LcdTaskControlBlock;
osThreadId ImuTaskHandle;//IMU任务句柄
uint32_t ImuTaskBuffer[ 1024 ];
osStaticThreadDef_t ImuTaskControlBlock;
osThreadId FunTestHandle;//功能测试任务句柄
uint32_t FunTestBuffer[ 128 ];
osStaticThreadDef_t FunTestControlBlock;
// 这里的“静态”表示：内存在编译时就提前分配好，不需要FreeRTOS运行时从堆中申请。
// 任务栈（Stack）           → 保存任务运行时的数据
// 任务控制块（TCB）         → 保存任务的管理信息

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

//这些是五个FreeRTOS任务入口函数的声明。
void StartDefaultTask(void const * argument);
void KeyTask_Entry(void const * argument);
void LcdTask_Entry(void const * argument);
void ImuTask_Entry(void const * argument);
void FunTest_Entry(void const * argument);

extern void MX_USB_DEVICE_Init(void);//外部USB设备初始化函数声明。
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
/* 获取空闲任务内存的函数原型，与静态内存分配功能有关 */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
/* 获取定时器任务内存的函数原型，与静态内存分配功能有关 */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;// 定义一个结构体，空闲任务的静态任务控制块
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];// 定义一个数组，空闲任务的静态任务栈



void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  /* 通过二级指针，将静态TCB的地址返回给FreeRTOS内核 */
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  /* 通过二级指针，将静态栈的地址返回给FreeRTOS内核 */
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  /* 将任务栈深度返回给FreeRTOS内核，单位为StackType_t元素 */
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;// 定义一个结构体，定时器任务的静态任务控制块
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];// 定义一个数组，定时器任务的静态任务栈



void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  /* 通过二级指针，将静态TCB的地址返回给FreeRTOS内核 */
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  /* 通过二级指针，将静态栈的地址返回给FreeRTOS内核 */
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  /* 将任务栈深度返回给FreeRTOS内核，单位为StackType_t元素 */
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
  /* 额外初始化代码 */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 创建互斥锁 */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 创建信号量 */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* FreeRTOS软件定时器 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 创建任务消息队列 */
  /* USER CODE END RTOS_QUEUES */

  /*
   * 定义默认启动任务：
   * 入口函数为StartDefaultTask，普通优先级，栈深度128。
   */
  osThreadDef(defaultTask, StartDefaultTask,
              osPriorityNormal, 0, 128);

  /* 创建默认启动任务，不向任务传递参数 */
  defaultTaskHandle =
      osThreadCreate(osThread(defaultTask), NULL);

  /*
   * 静态定义IMU任务：
   * 入口函数为ImuTask_Entry，高优先级，栈深度1024。
   * 使用提前分配的ImuTaskBuffer和ImuTaskControlBlock。
   */
  osThreadStaticDef(ImuTask, ImuTask_Entry,
                    osPriorityHigh, 0, 1024,
                    ImuTaskBuffer,
                    &ImuTaskControlBlock);

  /* 创建IMU任务，不向任务传递参数 */
  ImuTaskHandle =
      osThreadCreate(osThread(ImuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 其他用户任务可在这里创建 */
  /* USER CODE END RTOS_THREADS */
}



/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();//初始化USB设备
  /* USER CODE BEGIN StartDefaultTask */
  /* USB 枚举入口启动后，连接原工程的电机、PWM 和业务任务。 */
  task_Init();

  /* 初始化职责已完成，删除自身*/
  vTaskDelete(NULL);
  /* USER CODE END StartDefaultTask */
}



/* USER CODE BEGIN Header_KeyTask_Entry */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_KeyTask_Entry */
__weak void KeyTask_Entry(void const * argument)
{
  /* USER CODE BEGIN KeyTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END KeyTask_Entry */
}



/* USER CODE BEGIN Header_LcdTask_Entry */
/**
* @brief Function implementing the LcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask_Entry */
__weak void LcdTask_Entry(void const * argument)
{
  /* USER CODE BEGIN LcdTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LcdTask_Entry */
}



/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void const * argument)
{
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask_Entry */
}



/* USER CODE BEGIN Header_FunTest_Entry */
/**
* @brief Function implementing the FunTest thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FunTest_Entry */
__weak void FunTest_Entry(void const * argument)
{
  /* USER CODE BEGIN FunTest_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FunTest_Entry */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

