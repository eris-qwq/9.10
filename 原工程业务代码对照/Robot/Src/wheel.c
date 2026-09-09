#include "wheel.h"
/*
	VA轮 = Vx+Vy-Vz*(H/2+W/2)
	VB轮 = Vx-Vy-Vz*(H/2+W/2)
	VC轮 = Vx+Vy+Vz*(H/2+W/2)
	VD轮 = Vx-Vy+Vz*(H/2+W/2)
*/
/*
Vx = ex1 - vy + vz
Vy = vx - ex2 - vz
vz = ex3 -vx -vy
vx = ex4 +vy - vz

*/
void wheel_cal(Chassis_Motor_expect *speed,float Vx, float Vy, float Vz)  //Vx 前后速度 ,Vy 左右速度 , Vz 旋转速度
{
	  speed->expect_1 = Vx+Vy-Vz*(Car_H+Car_W)/2;
	  speed->expect_2 = Vx-Vy-Vz*(Car_H+Car_W)/2;
	  speed->expect_3 = Vx+Vy+Vz*(Car_H+Car_W)/2;
	  speed->expect_4 = Vx-Vy+Vz*(Car_H+Car_W)/2;
}
//3 - 1 = 2 * Vz 