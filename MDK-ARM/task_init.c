/**
 * @file    task_init.c
 * @brief   保留学长工程 task_Init() 入口，连接 MC02 的 FDCAN、PWM 与控制任务
 */

#include "task_init.h"
#include "PID_task.h"
#include "WatchDog.h"
#include "tim.h"
#include "Auto.h"
#include "can.h"
#include "mechanism_motor.h"

/**
 * @brief 初始化执行机构底层并创建原工程的两个业务任务。
 * @note  由 USB CDC 接收上位机数据，由 BMI088 更新姿态。
 */
void task_Init(void)
{
    /* CAN1连接四个底盘M2006（0x201~0x204）。 */
    CanFilter_Init(&hcan1);
    if (HAL_FDCAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        Error_Handler();
    }

    /* 底盘独立测试时只使用 CAN1，避免机构意外动作。 */
#if !CHASSIS_TEST_MODE
    /* CAN2连接伸缩M2006（0x202）和GM6020（0x205）；0x201预留给以后升级。 */
    CanFilter_Init(&hcan2);
    if (HAL_FDCAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hcan2,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        Error_Handler();
    }

#if !MECH_HEIGHT_USE_M2006
    /* 仅“保留升降步进”版本会启动PA0的STEP脉冲。 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    htim2.Instance->CCR1 = 0;
#endif

    /* TIM1_CH1 输出 50 Hz 舵机信号，初始置于学长工程的释放位置。 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    htim1.Instance->CCR1 = Put_Compare1;

#if !MECH_HEIGHT_USE_M2006
    /* 升降仍为步进电机时，TIM6每1ms累计原StepCounter。 */
    HAL_TIM_Base_Start_IT(&htim6);
#endif

    /* MC02 的 PC15 控制板载 5 V 输出，高电平持续给舵机供电。 */
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);

    /* 驱动器 ENA 已在外部固定为使能，保留原调用以维持代码结构。 */
    __StepMotor1_ON(GPIO_PIN_RESET);
    __StepMotor2_ON(GPIO_PIN_RESET);

    /* 读取上次保存的位置，并准备伸缩M2006闭环。 */
    Mechanism_Init();
#endif

    /* 独立测试不启动自动流程、视觉和机构任务。 */
    vPortEnterCritical();
#if CHASSIS_TEST_MODE
    xTaskCreate(ChassisTest_Task, "Chassis_test", 512, NULL, 2, NULL);
#else
    xTaskCreate(PID_Task, "PID_task", 750, NULL, 2, NULL);
    xTaskCreate(Remote_deal, "Remote_deal", 750, NULL, 2, NULL);
#endif
    vPortExitCritical();
}
