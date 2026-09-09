/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
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
