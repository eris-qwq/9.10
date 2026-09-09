/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
#include "wheel.h"
/*
	VA�� = Vx+Vy-Vz*(H/2+W/2)
	VB�� = Vx-Vy-Vz*(H/2+W/2)
	VC�� = Vx+Vy+Vz*(H/2+W/2)
	VD�� = Vx-Vy+Vz*(H/2+W/2)
*/
/*
Vx = ex1 - vy + vz
Vy = vx - ex2 - vz
vz = ex3 -vx -vy
vx = ex4 +vy - vz

*/
void wheel_cal(Chassis_Motor_expect *speed,float Vx, float Vy, float Vz)  //Vx ǰ���ٶ� ,Vy �����ٶ� , Vz ��ת�ٶ�
{
	  speed->expect_1 = Vx+Vy-Vz*(Car_H+Car_W)/2;
	  speed->expect_2 = Vx-Vy-Vz*(Car_H+Car_W)/2;
	  speed->expect_3 = Vx+Vy+Vz*(Car_H+Car_W)/2;
	  speed->expect_4 = Vx-Vy+Vz*(Car_H+Car_W)/2;
}
//3 - 1 = 2 * Vz 