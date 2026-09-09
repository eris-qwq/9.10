/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
#ifndef _PID_TASK_H_
#define _PID_TASK_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "RMLibHead.h"
#include "RoboModule_DRV.h"
/* F4 HAL 已替换为 MC02 所用的 STM32H723 HAL，业务数据类型保持不变。 */
#include "stm32h7xx_hal.h"
#include "wheel.h"
#include "CANDrive.h"
#include "PID.h"
#include "motor.h"
#include "ramp.h"
#include "Chassis.h"
#include "math.h"
/* 原工程的 JY61 串口解析层已由 App/imu_task.c 的板载 BMI088 读取层替代。
 * 控制层仍只读取 Now_Yaw 与 Now_Omega，因此下面的业务结构和 PID 逻辑不变。 */
#define PI                3.14159265f
#define Rad2Angle(x) (x * 57.2957795f)
#define Angle2Rad(x) (x * 0.01745329f)
#define SPEED_MODE 1      //速度模式
#define RM3510_MODE 0     //3510模式
/*
 * 置 1 后编译为“仅底盘整体运动测试”固件：不创建自动流程与机构控制任务，
 * PE2 长按 1 秒启动；测试过程中再次按下 PE2 会立即急停。
 * 完成实车测试后务必改回 0，再编译正常比赛固件。
 */
#define CHASSIS_TEST_MODE 0
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
} Data2Pack;

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
void ChassisTest_Task(void *pvParameters);

/* 底盘测试任务与原 PID 任务共用的四路反馈和速度环参数。 */
extern M2006_TypeDef motor_wheel_2006[4];
extern PID M2006_Speed[4];
extern PID_Smis PID_wheel_position[5];

#endif

/* ==================== 阅读导航 ====================
 * 下一文件：MDK-ARM/PID_task.c。先读全局数据，再搜索 Remote_deal，最后搜索 PID_Task。
 * ================================================== */
