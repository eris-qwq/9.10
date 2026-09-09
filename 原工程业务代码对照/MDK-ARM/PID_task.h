#ifndef _PID_TASK_H_
#define _PID_TASK_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "RMLibHead.h"
#include "RoboModule_DRV.h"
#include "stm32f4xx_hal.h"
#include "wheel.h"
#include "CANDrive.h"
#include "PID.h"
#include "motor.h"
#include "ramp.h"
#include "Chassis.h"
#include "math.h"
#include "JY61.h"
#define PI                3.14159265f
#define Rad2Angle(x) (x * 57.2957795f)
#define Angle2Rad(x) (x * 0.01745329f)
#define SPEED_MODE 1      //速度模式
#define RM3510_MODE 0     //3510模式
#pragma pack(1)

typedef struct
{
   uint8_t Left_Key_Up : 1;         
   uint8_t Left_Key_Down : 1;       
   uint8_t Left_Key_Left : 1;       
   uint8_t Left_Key_Right : 1;
   uint8_t Left_Side_Key : 1;       
   uint8_t Left_Switch_Up : 1;       
   uint8_t Left_Switch_Down: 1;       
   uint8_t UNUSED1 : 1;

   uint8_t Right_Key_Up : 1;        
   uint8_t Right_Key_Down : 1;      
   uint8_t Right_Key_Left : 1;      
   uint8_t Right_Key_Right : 1;
   uint8_t Right_Side_Key : 1;     
   uint8_t Right_Switch_Up : 1;      
   uint8_t Right_Switch_Down : 1;      
   uint8_t UNUSED2 : 1;
} hw_key_t;
  
typedef struct {
	uint8_t head;
	int16_t rocker[4];
	hw_key_t Key;
  uint8_t end;
} UART_DataPack;

typedef struct {
	uint8_t head,
			Fuction;
	uint16_t ID;
	uint16_t DataLen;
	uint8_t Sequence[6];
} Data1Pack;

typedef struct {
	uint8_t head,
			Fuction;
	uint16_t ID;
	uint16_t DataLen;
	int16_t ex,
			ey;
} ;

#pragma pack()
typedef struct {
    int16_t Ex;
    int16_t Ey;
    int16_t Eomega;
    hw_key_t *Key_Control;
    hw_key_t First,Second;
} Remote_Handle_t;

typedef struct{
	uint8_t QR_Yes : 1;
	int16_t Err_x;
	int16_t Err_y;
}Vision_Msg_t;

typedef enum{
	Idle,
	Move,
	Scan,
	Grab,
	Place
}State;

extern float GM6020ExAngle;
extern uint8_t once[3];
extern Chassis_Motor_expect expect_wheel_2006;
extern PID PID_wheel_speed[5];

float RAMP_self(float final, float now, float ramp );
void PID_Task(void *pvParameters);
void Remote_deal(void* param);

#endif
