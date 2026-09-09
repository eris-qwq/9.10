#ifndef _AUTO_H_
#define _AUTO_H_

#include "main.h"
#include "PID_task.h"
#include "arm_math.h"
#include "tim.h"
#include "mechanism_motor.h"

typedef struct {
    uint8_t Sequence[6];
} Auto_t;

extern int8_t Dir[2];
extern Auto_t AutoSequence;
extern PID_Smis PID_wheel_position[5];

/* 学长自动流程使用的机构逻辑位置，单位仍是原StepCounter单位。 */
#define GrabHeight    1950
#define CarDropHeight 1000
#define CarGrabHeight 1200
#define Put2Height    1950
#define GroundHeight  3050

/* GM6020三个取放角度。 */
#define Place1 -275
#define Place2  500
#define Place3  1100

/* 伸缩机构对应三个物料位的逻辑长度。 */
#define DropLen1 340
#define DropLen2 940
#define DropLen3 2720

#define _2006_Speed -4000

/* 舵机PWM比较值；改动前先确认实际舵机安全行程。 */
#define Grab_Compare 2500
#define Put_Compare0 2400
#define Put_Compare1 2200

/* 与car V2上位机统一的颜色编号；目前只有红、蓝、绿配置了实际机构工位。 */
#define RED       1
#define YELLOW    2
#define BLUE      3
#define GREEN     4
#define BLACK     5
#define LIGHTBLUE 6

/* 仅“升降仍用步进”的版本会真正使用DIR1；其余宏名为兼容原工程保留。 */
#define __StepMotor1_ON(x)  ((void)(x))
#define __StepMotor1_Dir(x) HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (x))
#define __StepMotor2_ON(x)  ((void)(x))
#define __StepMotor2_Dir(x) HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (x))

#define __Grab__() do {        \
    TIM1->CCR1 = Grab_Compare; \
    vTaskDelay(250);           \
} while (0)

#define __Release__() do {       \
    TIM1->CCR1 = Put_Compare0;   \
    vTaskDelay(150);             \
    TIM1->CCR1 = Put_Compare1;   \
} while (0)

/* 保留原动作名，内部由机构层选择步进开环或M2006位置闭环。 */
#define __Length__(x)  Mechanism_Length((uint32_t)(x))
#define __Shorten__(x) Mechanism_Shorten((uint32_t)(x))
#define __Up__(x)      Mechanism_Up((uint32_t)(x))
#define __Down__(x)    Mechanism_Down((uint32_t)(x))

uint8_t AutoArm(int16_t Ex_x, int16_t Ex_y, uint8_t is_Valid);
void AutoArmGrabDrop(uint8_t GrabPlace1, uint8_t GroundGrab, uint8_t CarGrab, uint8_t PutPlace);
uint8_t AutoArm1(int16_t Ex_x, int16_t Ex_y, int16_t *ex, uint8_t is_Valid);
uint8_t AutoPosAim(int16_t Ex_x, int16_t Ex_y, int16_t *ex, int16_t *ey, uint8_t is_Valid);
void AutoArmPos(uint8_t Color, uint8_t height);
uint8_t AutoMove(M2006_TypeDef *motor, float *Last, float Des);

#endif
