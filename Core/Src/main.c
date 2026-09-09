/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

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
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);//总时钟初始化函数，函数声明
void MX_FREERTOS_Init(void);//初始化 FreeRTOS，函数声明
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  HAL_Init();//HAL库初始化函数，函数调用

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();//总时钟初始化函数，函数调用

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* 初始化本工程实际需要的外设 */
  /*GPIO   → 按钮、方向、电源使能、片选
    DMA    → 数据自动搬运，目前基本没用
    SPI2   → 板载BMI088
    TIM3   → BMI088加热
    CAN1   → 4个底盘M2006
    CAN2   → 1个伸缩M2006和1个GM6020
    TIM1   → 舵机
    TIM2   → PA0输出升降步进脉冲
    TIM6   → 升降步进位置软件计数*/
  MX_GPIO_Init();    // 初始化GPIO：PE2启动按钮、PE7/PE8步进方向、PC15板载5V使能、BMI088片选等
  MX_DMA_Init();     // 初始化DMA1控制器；来自MC02官方底层，目前本项目业务代码暂未实际使用DMA传输
  MX_OCTOSPI2_Init();// 初始化板载W25Q64，用于保存机构断电前的软件位置
  MX_SPI2_Init();    // 初始化SPI2：与MC02板载BMI088通信，读取加速度和角速度数据
  MX_TIM3_Init();    // 初始化TIM3：CH4输出PWM，控制板载BMI088的恒温加热电路
  MX_FDCAN1_Init();  // 初始化CAN1：控制4个底盘M2006（0x201~0x204）
  MX_FDCAN2_Init();  // 初始化CAN2：控制伸缩M2006（0x202）和GM6020（0x205）
  MX_TIM1_Init();    // 初始化TIM1：CH1从PE9输出50Hz PWM，控制舵机
  MX_TIM2_Init();    // 保留TIM2底层，只有“升降保留步进”版本使用PA0 STEP
  MX_TIM6_Init();    // 保留TIM6底层，只有“升降保留步进”版本启动其计数中断
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();//初始化 FreeRTOS，函数调用，只是把任务创建出来，任务还没有正式运行。

  /* Start scheduler */
  osKernelStart();//启动FreeRTOS任务调度器，让之前创建的任务真正开始运行。

  /* We should never get here as control is now taken by the scheduler */
  /* 程序正常情况下永远不会运行到这里，因为控制权已经交给任务调度器 */

  /* Infinite loop */
  /* 无限循环 */
  
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  vofa_start();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration 配置STM32H723的系统时钟
  * @retval None 无返回值
  */
void SystemClock_Config(void)
{
  //创建两个时钟配置结构体
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};//Oscillator = 振荡器
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};//Clock = 时钟

  /** Supply configuration update enable 使能供电配置更新
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);//表示STM32H723采用内部LDO稳压器供电模式。

  /** Configure the main internal regulator output voltage 配置主内部稳压器的输出电压
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  //等待内部稳压器的电压档位配置完成
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * 根据RCC_OscInitTypeDef结构体中指定的参数，初始化RCC振荡器。
  */
  //指定要配置哪些时钟源
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48//HSI48：STM32内部48MHz振荡器。
                                    |RCC_OSCILLATORTYPE_HSI  //STM32内部64MHz高速振荡器。
                                    |RCC_OSCILLATORTYPE_HSE; //HSE：MC02板载24MHz外部晶振
                                    //注意：选择配置这些振荡器，不代表它们都会作为CPU主时钟。

  RCC_OscInitStruct.HSEState = RCC_HSE_ON;       // 开启MC02板载24MHz外部晶振
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;     // 开启64MHz内部HSI，不分频
  RCC_OscInitStruct.HSICalibrationValue = 64;    // 设置HSI校准参数
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;   // 开启USB所需的48MHz内部时钟

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;         // 开启PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // PLL使用24MHz外部晶振
  RCC_OscInitStruct.PLL.PLLM = 2;                      // 24MHz÷2=12MHz
  RCC_OscInitStruct.PLL.PLLN = 40;                     // 12MHz×40=480MHz
  RCC_OscInitStruct.PLL.PLLP = 1;                      // 480MHz÷1=480MHz，供CPU使用
  RCC_OscInitStruct.PLL.PLLQ = 4;                      // 480MHz÷4=120MHz，供FDCAN等使用
  RCC_OscInitStruct.PLL.PLLR = 2;                      // 480MHz÷2=240MHz
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;   // PLL输入12MHz所在的范围
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;   // 选择宽VCO频率范围
  RCC_OscInitStruct.PLL.PLLFRACN = 0;                  // 不使用小数倍频

  /* 将HSE、HSI、HSI48和PLL参数写入硬件，并检查配置结果 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* 时钟配置失败，进入错误处理，停止继续启动系统 */
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks 
  */
  //选择需要配置的时钟
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
                              //SYSCLK：CPU系统时钟。
                              // HCLK：AHB总线时钟。
                              // PCLK1：APB1时钟。
                              // PCLK2：APB2时钟。
                              // D1PCLK1：D1域中的APB3时钟。
                              // D3PCLK1：D3域中的APB4时钟。

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 系统时钟选择480MHz PLL输出
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;        // CPU不分频：480MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;          // AHB二分频：240MHz
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;         // APB3二分频：120MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;         // APB1二分频：120MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;         // APB2二分频：120MHz
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;         // APB4二分频：120MHz

  /* 将时钟参数写入硬件，并检查配置结果 */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* 时钟配置失败，进入错误处理，停止继续启动系统 */
    Error_Handler();
  }

  //把STM32内部时钟从 MCO1 引脚输出到板外。
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */


/**
 * @brief  Period elapsed callback in non-blocking mode.
 *         非阻塞模式下的定时器周期结束回调函数。
 *
 * @note   This function is called when a TIM23 interrupt takes place,
 *         inside HAL_TIM_IRQHandler().
 *         当TIM23产生中断时，本函数会在HAL_TIM_IRQHandler()内部被调用。
 *
 *         It directly calls HAL_IncTick() to increment the global
 *         variable "uwTick", which is used as the application time base.
 *         本函数直接调用HAL_IncTick()，使全局变量uwTick递增；
 *         uwTick用于提供HAL程序的系统时间基准。
 *
 * @param  htim  TIM handle.
 *               定时器句柄，用于判断产生中断的定时器。
 *
 * @retval None  No return value.
 *               无返回值。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM23) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
  // TIM23计数一个周期
  //       ↓
  // 产生TIM23中断
  //       ↓
  // TIM23_IRQHandler()
  //       ↓
  // HAL_TIM_IRQHandler()
  //       ↓
  // HAL_TIM_PeriodElapsedCallback(&htim23)
  //       ↓
  // 判断htim->Instance是不是TIM23
  //       ↓ 是
  // HAL_IncTick()
  //       ↓
  // uwTick增加1ms


/**
  * @brief  This function is executed in case of error occurrence.  
            当程序发生严重错误时执行本函数。
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  // 用户可以在这里添加错误指示，例如点亮故障LED或记录错误信息。
   /* 关闭CPU的可屏蔽中断，防止其他中断程序继续运行 */
  __disable_irq();
  //RQ = Interrupt Request = 中断请求
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
           报告assert_param参数检查错误所在的源文件名和代码行号。
  * @param  file: pointer to the source file name
  *              指向源文件名的指针
  * @param  line: assert_param error line source number
  *              assert_param参数检查错误所在的代码行号
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  //  * 用户可以在这里输出发生错误的文件名和代码行号，例如：
  //  * printf("参数错误：文件 %s，第 %lu 行\r\n",
  //  *        file, (unsigned long)line);
  //  * 注意：使用printf之前，需要先完成串口或USB CDC重定向。
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
