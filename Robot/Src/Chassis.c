/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
/**
 * @file    Chassis.c
 * @author  yao
 * @date    1-May-2020
 * @brief   底盘驱动模块
 */

#include "Chassis.h"


void ChassisMotorSpeedClean(ChassisSpeed_Ref_t *ref) {
    ref->forward_back_ref = 0;
    ref->left_right_ref = 0;
    ref->rotate_ref = 0;
}

__weak void PID_Expect(Chassis_Motor_Speed *motor, ChassisSpeed_Ref_t *ref) {
    motor->speed_3 = -ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_2 = ref->left_right_ref + ref->rotate_ref;                            

    motor->speed_1 = ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;
}
void Run_Speed(Chassis_Motor_expect *motor,int32_t Ex, int32_t Ey, int32_t Angle) {
        motor->expect_1 +=  -(Ey * 0.86603f  - Ex * 0.5f + Angle)*0.5f;
        motor->expect_2 += -(Ex + Angle)*0.5f;
        motor->expect_3 += -(-Ey * 0.86603f - Ex * 0.5f + Angle)*0.5f;

}

/* ==================== 阅读导航 ====================
 * 下一文件：Robot/Src/motor.c。理解 CAN 字节、电机角度/速度和发送控制量。
 * ================================================== */
