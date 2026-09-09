#ifndef _AUTO_H_
#define _AUTO_H_

#include "main.h"
#include "PID_task.h"
#include "arm_math.h"
#include "tim.h"

typedef struct{
	uint8_t Sequence[6];//颜色R1G2B3
}Auto_t;

extern int8_t Dir[2];
extern Auto_t AutoSequence;
extern PID_Smis PID_wheel_position[5];
//圆盘抓取累计高度
#define GrabHeight 1950
//放置车上累计高度
#define CarDropHeight 1000
//车上抓取累计高度
#define CarGrabHeight 1200
//叠物料累计高度
#define Put2Height 1950
//放置地面累计高度
#define GroundHeight 3050
//GM6020位置
#define Place1 -275
#define Place2 500
#define Place3 1100
//车放置臂长
#define DropLen1 340  //1000走21mm
#define DropLen2 940
#define DropLen3 2720

#define _2006_Speed -4000

#define Grab_Compare 2500
#define Put_Compare0 2400
#define Put_Compare1 2200
//放置参数//高度  2000
//-1930  90°
//ey = 90  GLen = 800  GAngle = -1930  BLen=1650 BAngle=-2940 RLen=2760 RAngle=-1035
//	 120          1250
//#define RedAngle -1035
//#define RedLengt 2760


//#define GreenAngle -1930
//#define GreenLengt 800

//#define BlueAngle -2940
//#define BlueLengt 1650

#define RED 1
#define GREEN 2
#define BLUE 3

#define __StepMotor1_ON(x) 	  HAL_GPIO_WritePin(EN1_GPIO_Port,EN1_Pin,x)
#define __StepMotor1_Dir(x)   HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin,x)//1下降0上升
#define __StepMotor2_ON(x)    HAL_GPIO_WritePin(EN2_GPIO_Port,EN2_Pin,x)
#define __StepMotor2_Dir(x)   HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin,x)//1收缩0伸出

//夹取
#define __Grab__()  TIM4->CCR1 = Grab_Compare;\
					vTaskDelay(250);
//松开
#define __Release__() TIM4->CCR1 = Put_Compare0;\
					  vTaskDelay(150);			\
					  TIM4->CCR1 = Put_Compare1;
//伸出长度
#define __Length__(x)  __StepMotor2_Dir(0);	 	 \
					   Dir[1] = 1;				 \
					   htim1.Instance->CCR2 = 39;\
					   vTaskDelay(x);  	 		 \
					   htim1.Instance->CCR2 = 0; 
//收缩长度
#define __Shorten__(x) __StepMotor2_Dir(1);		 \
					   Dir[1] = -1;				 \
					   htim1.Instance->CCR2 = 39;\
					   vTaskDelay(x);  	 		 \
					   htim1.Instance->CCR2 = 0; 
//上升
#define __Up__(x) 	   __StepMotor1_Dir(0);		 \
					   Dir[0] = -1;				 \
					   htim1.Instance->CCR1 = 39;\
					   vTaskDelay(x);  	 		 \
					   htim1.Instance->CCR1 = 0; 
//下降
#define __Down__(x)    __StepMotor1_Dir(1);		 \
					   Dir[0] = 1;				 \
					   htim1.Instance->CCR1 = 39;\
					   vTaskDelay(x);  	 		 \
					   htim1.Instance->CCR1 = 0; 
	
uint8_t AutoArm(int16_t Ex_x,int16_t Ex_y,uint8_t is_Valid);
void AutoArmGrabDrop(uint8_t GrabPlace1,uint8_t GroundGrab,uint8_t CarGrab,uint8_t PutPlace);
uint8_t AutoArm1(int16_t Ex_x,int16_t Ex_y,int16_t* ex,uint8_t is_Valid);
uint8_t AutoPosAim(int16_t Ex_x,int16_t Ex_y,int16_t* ex,int16_t* ey,uint8_t is_Valid);
void AutoArmPos(uint8_t Color,uint8_t height);
uint8_t AutoMove(M2006_TypeDef* motor,float* Last,float Des);
#endif