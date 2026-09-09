/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
#ifndef _WHEEL_H_
#define _WHEEL_H_

#define Car_H 0.243//0.243   //前后轮轴距
#define Car_W 0.220//0.220     //左右轮轴距

#include "main.h"
#include "PID_task.h"
/* Linux/GCC 区分文件名大小写，实际保留的业务文件名为 Chassis.h。 */
#include "Chassis.h"
#include "gpio.h"

void wheel_cal(Chassis_Motor_expect *speed,float Vx, float Vy, float Vz);

#endif
