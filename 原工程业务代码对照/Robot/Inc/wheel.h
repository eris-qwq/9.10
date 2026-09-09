#ifndef _WHEEL_H_
#define _WHEEL_H_

#define Car_H 1//0.243   //Öá¾à m
#define Car_W 1//0.220     //ÂÖ¾à m

#include "main.h"
#include "PID_task.h"
#include "chassis.h"
#include "gpio.h"

void wheel_cal(Chassis_Motor_expect *speed,float Vx, float Vy, float Vz);

#endif
