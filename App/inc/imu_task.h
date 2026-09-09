/**
 * @file    imu_task.h
 * @brief   MC02 板载 BMI088 姿态任务接口与坐标方向配置
 */

#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "cmsis_os.h"

#define INS_YAW_ADDRESS_OFFSET   0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET  2

/*
 * 原 JY61 工程按“顺时针为正”使用航向，BMI088 的右手系 Z 轴通常相反，
 * 因此默认取 -1。装车后若原地转向闭环方向相反，只修改这两个宏即可。
 */
#define IMU_YAW_SIGN  (-1.0f)
#define IMU_GYRO_SIGN (-1.0f)

extern float gyro[3];
extern float acc[3];
extern float imuQuat[4];
extern float imuAngle[3];
extern uint8_t ImuReady;

void AHRS_init(float quat[4]);
void AHRS_update(float quat[4], float gyro_data[3], float accel_data[3]);
void GetAngle(float q[4], float *yaw, float *pitch, float *roll);
void ImuTask_Entry(void const *argument);
float IMU_GetTemperature(void);
uint16_t IMU_GetHeaterPwm(void);
uint8_t IMU_GetCalibrationState(void);
uint8_t IMU_GetCalibrationProgress(void);
float IMU_GetGyroBiasZDegPerSecond(void);

#endif /* __IMU_TASK_H__ */
