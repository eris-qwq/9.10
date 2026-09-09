//#ifndef _SERVOS_TASK_H_
//#define _SERVOS_TASK_H_

//#include "FreeRTOS.h"
//#include "cmsis_os.h"
//#include "task.h"
//#include "RMLibHead.h"
//#include "CANDrive.h"
//#include "RoboModule_DRV.h"
//#include "PID.h"
//#include "motor.h"
//#include "ramp.h"
//#include "Chassis.h"


//typedef int32_t (*RAMP) (int32_t,int32_t,int32_t);

//typedef struct {
//    
//    RoboModule_Typedef *Putter;                            
//    int16_t Putter_offest;
//    SemaphoreHandle_t semaphore;
//    
//}Puttor_Handle;

//typedef struct {
//    RAMP pi32ramp;        //函数指针

//    int16_t Puttor_expect;
//    int16_t Puttor_expect_up;
//    int16_t Puttor_expect_down;
//    Puttor_Handle InPuttor[2]; //两个电推杆
//}GRAB_LAUNCH_Typedef;
//	
//extern GRAB_LAUNCH_Typedef G_L_t;
//extern RoboModule_Typedef putter;
//extern RoboModule_Typedef RoboModulelaunch;

//extern TaskHandle_t Lanuch_Handler;
//void Lanuch_Task(void *pvParameters);
//extern TaskHandle_t vPuttor_Handler;
//void vPuttor_task(void *parameters);
//extern TaskHandle_t mylaunch_Handler;
//void mylaunch_Task(void *pvParameters);	

//extern int textexp;
//void setPos_put(Puttor_Handle *m, int32_t pos);
//int32_t RAMP_selfs( int32_t final, int32_t now, int32_t ramp );
//void Shut(GRAB_LAUNCH_Typedef *x,int16_t expect);
//void vBlue_Brick_Init(void);
//void launchset(int32_t expect,int32_t speed);
//#endif
