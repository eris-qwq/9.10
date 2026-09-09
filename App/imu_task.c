/**
 * @file    imu_task.c
 * @brief   读取 MC02 板载 BMI088，并保持 Now_Yaw/Now_Omega 控制层接口
 */

#include "imu_task.h"

#include <math.h>

#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "tim.h"

#define DES_TEMP 40.0f
#define TEMP_KP  300.0f
#define TEMP_KI  40.0f
#define TEMP_CONTROL_PERIOD_S 0.001f
#define TEMP_MAX_OUT 500.0f
#define TEMP_INTEGRAL_MAX 1500.0f
#define TEMP_HARD_CUTOFF 45.0f
#define GYRO_CALIBRATION_TEMP 39.0f
#define GYRO_CALIBRATION_SAMPLES 3000U
#define GYRO_CALIBRATION_STILL_LIMIT 0.05f

#define RAD_TO_CENTIDEG 5729.5779513f
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};
uint8_t ImuReady;
static float temperature;
static uint16_t heater_pwm;
static float gyro_bias[3];
static uint8_t calibration_state;
static uint8_t calibration_progress;
extern float Now_Yaw;
extern int16_t Now_Omega;

void AHRS_init(float quat[4]);
void AHRS_update(float quat[4], float gyro_data[3], float accel_data[3]);
void GetAngle(float q[4], float *yaw, float *pitch, float *roll);
static int16_t FloatToInt16Saturated(float value);
static void UpdateHeater(float current_temperature);



//ImuTask**********************************************************************
//ImuTask**********************************************************************
//ImuTask**********************************************************************
/**
 * @brief BMI088 姿态任务入口，每 1 ms 更新一次姿态和控制层接口。
 * @param argument FreeRTOS 任务参数，本工程未使用。
 * @note
 * Now_Yaw 单位固定为 0.01°并做多圈展开；Now_Omega 单位固定为 0.01°/s。
 * 默认符号取反以模拟原 JY61“顺时针为正”的输出，可在 imu_task.h 修改。
 */
void ImuTask_Entry(void const *argument)
{
    //局部变量
    float last_yaw = 0.0f;
    float continuous_yaw = 0.0f;
    uint8_t first_sample = 1U;
    float gyro_sum[3] = {0.0f};
    uint32_t calibration_samples = 0U;

    (void)argument;

    //等设备稳定
    osDelay(10U);
    //加热PWM
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* 传感器未就绪时周期重试，禁止把无效姿态送入 PID。 */
    while (BMI088_init() != 0U)
    {
        ImuReady = 0U;
        osDelay(100U);
    }


    AHRS_init(imuQuat);
    ImuReady = 0U;
    calibration_state = 0U;
    calibration_progress = 0U;

    for (;;)
    {
        float delta_yaw;

        BMI088_read(gyro, acc, &temperature);
        UpdateHeater(temperature);

        /* 温度到39℃后静置3秒，测量本次上电的三轴零偏。 */
        if (calibration_state != 2U)
        {
            if (temperature < GYRO_CALIBRATION_TEMP)
            {
                calibration_state = 0U;
                calibration_progress = 0U;
                calibration_samples = 0U;
                gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0f;
            }
            else if ((fabsf(gyro[0]) < GYRO_CALIBRATION_STILL_LIMIT) &&
                     (fabsf(gyro[1]) < GYRO_CALIBRATION_STILL_LIMIT) &&
                     (fabsf(gyro[2]) < GYRO_CALIBRATION_STILL_LIMIT))
            {
                calibration_state = 1U;
                gyro_sum[0] += gyro[0];
                gyro_sum[1] += gyro[1];
                gyro_sum[2] += gyro[2];
                ++calibration_samples;
                calibration_progress = (uint8_t)((calibration_samples * 100U) /
                                                   GYRO_CALIBRATION_SAMPLES);
                if (calibration_samples >= GYRO_CALIBRATION_SAMPLES)
                {
                    gyro_bias[0] = gyro_sum[0] / (float)GYRO_CALIBRATION_SAMPLES;
                    gyro_bias[1] = gyro_sum[1] / (float)GYRO_CALIBRATION_SAMPLES;
                    gyro_bias[2] = gyro_sum[2] / (float)GYRO_CALIBRATION_SAMPLES;
                    calibration_state = 2U;
                    calibration_progress = 100U;
                    AHRS_init(imuQuat);
                    continuous_yaw = 0.0f;
                    first_sample = 1U;
                }
            }
            else
            {
                /* 三秒内被移动就从0%重新开始。 */
                calibration_state = 1U;
                calibration_progress = 0U;
                calibration_samples = 0U;
                gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0f;
            }
            osDelay(1U);
            continue;
        }

        gyro[0] -= gyro_bias[0];
        gyro[1] -= gyro_bias[1];
        gyro[2] -= gyro_bias[2];

        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat,
                 imuAngle + INS_YAW_ADDRESS_OFFSET,
                 imuAngle + INS_PITCH_ADDRESS_OFFSET,
                 imuAngle + INS_ROLL_ADDRESS_OFFSET);

        if (first_sample != 0U)
        {
            /* 第一帧作为零点，与原 JY61 的 GyroOffset 行为一致。 */
            last_yaw = imuAngle[INS_YAW_ADDRESS_OFFSET];
            first_sample = 0U;
            ImuReady = 1U;
        }

        delta_yaw = imuAngle[INS_YAW_ADDRESS_OFFSET] - last_yaw;

        //修正-180，+180跳变
        if (delta_yaw > PI_F)
        {
            delta_yaw -= TWO_PI_F;
        }
        else if (delta_yaw < -PI_F)
        {
            delta_yaw += TWO_PI_F;
        }
        continuous_yaw += delta_yaw;
        last_yaw = imuAngle[INS_YAW_ADDRESS_OFFSET];

        /* 对控制层继续输出原变量名和原类型，避免改动自动流程与 Yaw PID。 */
        //Now_Yaw：航向角从上电累积值，0.01度，逆时针正向
        Now_Yaw = IMU_YAW_SIGN * continuous_yaw * RAD_TO_CENTIDEG;
        //Now_Yaw：航向角角速度，0.01度，逆时针正向
        Now_Omega = FloatToInt16Saturated(IMU_GYRO_SIGN
                                          * gyro[2]
                                          * RAD_TO_CENTIDEG);

        osDelay(1U);
    }
}



































/**
 * @brief 把四元数初始化为无旋转姿态。
 * @param quat 四元数数组，顺序为 w、x、y、z。
 */
void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

/**
 * @brief 使用角速度和加速度更新 Mahony 姿态解算。
 * @param quat 当前四元数，函数会原地更新。
 * @param gyro_data 三轴角速度，单位 rad/s。
 * @param accel_data 三轴加速度，单位 m/s²。
 */
void AHRS_update(float quat[4], float gyro_data[3], float accel_data[3])
{
    MahonyAHRSupdateIMU(quat,
                        gyro_data[0], gyro_data[1], gyro_data[2],
                        accel_data[0], accel_data[1], accel_data[2]);
}

/**
 * @brief 把四元数换算为偏航、俯仰和横滚欧拉角。
 * @param q 四元数，顺序为 w、x、y、z。
 * @param yaw 输出偏航角，单位 rad。
 * @param pitch 输出俯仰角，单位 rad。
 * @param roll 输出横滚角，单位 rad。
 */
void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                  2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

/**
 * @brief 把浮点角速度限制并转换为原工程的 int16_t 接口。
 * @param value 单位为 0.01°/s 的浮点数。
 * @return 限制到 int16_t 范围后的数值。
 */
static int16_t FloatToInt16Saturated(float value)
{
    if (value > 32767.0f)
    {
        value = 32767.0f;
    }
    else if (value < -32768.0f)
    {
        value = -32768.0f;
    }
    return (int16_t)value;
}

/**
 * @brief 根据温度误差调节 BMI088 加热片 PWM。
 * @param current_temperature BMI088 当前温度，单位 ℃。
 */
static void UpdateHeater(float current_temperature)
{
    static float integral_output;
    const float error = DES_TEMP - current_temperature;
    float output;

    /* 真正按时间积分的PI；积分项限幅，避免升温阶段饱和后过冲。 */
    integral_output += TEMP_KI * error * TEMP_CONTROL_PERIOD_S;
    if (integral_output > TEMP_INTEGRAL_MAX)
    {
        integral_output = TEMP_INTEGRAL_MAX;
    }
    else if (integral_output < 0.0f)
    {
        integral_output = 0.0f;
    }
    output = TEMP_KP * error + integral_output;

    /* 任何算法或温度读数异常时，45℃都强制关闭加热。 */
    if (current_temperature >= TEMP_HARD_CUTOFF)
    {
        output = 0.0f;
        integral_output = 0.0f;
    }

    if (output > TEMP_MAX_OUT)
    {
        output = TEMP_MAX_OUT;
    }
    else if (output < 0.0f)
    {
        output = 0.0f;
    }
    heater_pwm = (uint16_t)output;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, heater_pwm);
}

float IMU_GetTemperature(void)
{
    return temperature;
}

uint16_t IMU_GetHeaterPwm(void)
{
    return heater_pwm;
}

uint8_t IMU_GetCalibrationState(void)
{
    return calibration_state;
}

uint8_t IMU_GetCalibrationProgress(void)
{
    return calibration_progress;
}

float IMU_GetGyroBiasZDegPerSecond(void)
{
    return gyro_bias[2] * 57.2957795f;
}
