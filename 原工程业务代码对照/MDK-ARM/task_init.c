#include "Task_Init.h"
#include "PID_task.h"
#include "WatchDog.h"
#include "usart.h"
#include "tim.h"
#include "Auto.h"

extern uint8_t usart4_dma_buff[12];
extern uint8_t usart3_dma_buff[64];
extern uint8_t usart2_dma_buff[13];

void task_Init() {
     /*CAN1_Init*/
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    
    /*CAN2_Init*/
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2,usart2_dma_buff,sizeof usart2_dma_buff);
	
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,usart3_dma_buff,sizeof usart3_dma_buff);
		
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart4,usart4_dma_buff,sizeof usart4_dma_buff);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	TIM4->CCR1 = Put_Compare1;
    
	__HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim6);
	
	__StepMotor1_ON(GPIO_PIN_RESET);
	__StepMotor2_ON(GPIO_PIN_RESET);
	
vPortEnterCritical();
    xTaskCreate(PID_Task,
         "PID_task",
          750,
          NULL,
          2,
          NULL); 
//	xTaskCreate(WatchDog_Task, 
//          "WatchDog_task",
//          64,
//          NULL, 
//          4,
//          NULL);
	xTaskCreate(Remote_deal,
          "Remote_deal",
          750,
          NULL,
          2,
          NULL);
vPortExitCritical();
}
