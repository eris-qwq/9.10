/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
//#include "Servos_task.h"

//RoboModule_Typedef putter =  {.ID = 0x110,.mode = Position_Mode,.ExpPWM = 5000};
//RoboModule_Typedef RoboModulelaunch = {.ID = 0x040, .ExpPWM = 5000, .mode =  Velocity_Position_Mode}; 
//GRAB_LAUNCH_Typedef G_L_t = {
//    .pi32ramp = RAMP_selfs,
//    .InPuttor[0].Putter = &putter,
//};

////void setPos_put(Puttor_Handle *m, int32_t pos) {//电推杆位置控制
////     uint16_t  i = 0;
////    i = abs(pos + m->Putter_offest - m->Putter->ExpPosition) / 10; 
////    while(i--){
////          m->Putter->ExpPosition = RAMP_selfs(pos + m->Putter_offest,m->Putter->ExpPosition,10);
////          vTaskDelay(35);
////    }
////}
//int32_t RAMP_selfs( int32_t final, int32_t now, int32_t ramp )
//{
//    float buffer = final - now;
//    
//    if (buffer > 0)
//    {
//        if (buffer > ramp)  
//                now += ramp;  
//        else
//                now += buffer;
//    }		
//    else
//    {
//        if (buffer < -ramp)
//                now += -ramp;
//        else
//                now += buffer;
//    }
//    return now;
//    
//}

//void vBlue_Brick_Init() {
//    

//    
////  CAN_RoboModule_DRV_Reset(&hcan2, &putter);
////  vTaskDelay(1000);
////	CAN_RoboModule_DRV_Reset(&hcan2, &putter);      
////  vTaskDelay(500);
////  CAN_RoboModule_DRV_Mode_Choice(&hcan2, &putter, Position_Mode); 
////  vTaskDelay(500);
////	CAN_RoboModule_DRV_Config(&hcan2, &putter, 2, 0);
////  vTaskDelay(1);
//    CAN_RoboModule_DRV_Reset(&hcan2, &RoboModulelaunch);     
//   	vTaskDelay(1000);	  
//	CAN_RoboModule_DRV_Reset(&hcan2, &RoboModulelaunch);     
//   	vTaskDelay(500);
//  	CAN_RoboModule_DRV_Mode_Choice(&hcan2, &RoboModulelaunch,Velocity_Position_Mode); 
//	vTaskDelay(500);
//	CAN_RoboModule_DRV_Config(&hcan2, &RoboModulelaunch, 2, 0);
//	vTaskDelay(1);
//    
//}

//void launchset(int32_t expect,int32_t speed) {
//  	osDelay(2);
//    RoboModulelaunch.ExpPosition = expect;  //ÆÚÍû	 
//	RoboModulelaunch.ExpVelocity = speed;  //ËÙ¶È
//	osDelay(2);
//}


//TaskHandle_t Lanuch_Handler;
//EventGroupHandle_t pxSTEP_t; 

////TaskHandle_t vPuttor_Handler;
////void vPuttor_task(void *parameters){
////    Puttor_Handle * X = (Puttor_Handle *)parameters;
////    uint16_t i = 100;  
////    while(i--){
////        X->Putter->ExpPosition-=10;
////        osDelay(30);
////    }
////    X->Putter->ExpPosition =  X->Putter->Position;
////    X->Putter_offest =  X->Putter->Position;
////    BaseType_t err = pdFALSE; 
////    for(;;) {
////        err = xSemaphoreTake(X->semaphore,portMAX_DELAY); //等待二值信号量
////        if(err == pdTRUE){
////            setPos_put(X,G_L_t.Puttor_expect);
////        }
////        else  vTaskDelay(10);
////    }
////}

////int textexp;
////void Lanuch_Task(void *pvParameters){
////	
////	G_L_t.InPuttor[0].semaphore = xSemaphoreCreateBinary();
//////	xTaskCreate(vPuttor_task,
//////				"vPuttor_task",
//////				80,
//////				&G_L_t.InPuttor[0].Putter,   
//////				1,
//////				&vPuttor_Handler);
////		portTickType xLastWakeTime = xTaskGetTickCount();
////	for(;;){
////	Shut(&G_L_t,textexp);
////       CAN_RoboModule_DRV_Position_Mode(&hcan2,&putter);
////			vTaskDelay(5);
////		vTaskDelayUntil(&xLastWakeTime,2);
////		
////  }
////}

////void Shut(GRAB_LAUNCH_Typedef *x,int16_t expect) {
////    
////    x->Puttor_expect = expect;  //电推杆期望
////    xSemaphoreGive(x->InPuttor[0].semaphore); //两个电推杆的二值信号量
////    
////}

// TaskHandle_t mylaunch_Handler;
//void mylaunch_Task(void *pvParameters)
//{  
//	for (;;) 
//	{
//		CAN_RoboModule_DRV_Velocity_Position_Mode(&hcan2,&RoboModulelaunch); 	
//		vTaskDelay(2);
//	} 
//}

