#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "queue.h"
#include "event_groups.h" 
#include "cmsis_os.h"

#include "CANDrive.h"
#include "RoboModule_DRV.h"
#include "motor.h"
#include "CRC.h"


void task_Init(void);    
void RGB_init(void);

extern TaskHandle_t vGrab_LunchtaskHandle;


#endif
