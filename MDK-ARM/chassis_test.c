/**
 * @file chassis_test.c
 * @brief 四轮底盘角度位置串级 PID 自动测试
 *
 * 测试流程：
 *
 * 1. 上电后等待一段时间
 * 2. 记录四个轮子的当前角度作为起始位置
 * 3. 前进指定角度
 * 4. 停止
 * 5. 后退相同角度
 * 6. 停止
 * 7. 左移指定角度
 * 8. 停止
 * 9. 右移相同角度
 * 10. 原地旋转指定角度
 * 11. 停止
 * 12. 测试结束
 *
 * 控制方式：
 *
 * 目标角度
 *     ↓
 * PID_Control_Smis()
 *     ↓
 * 目标角速度
 *     ↓
 * PID_Control()
 *     ↓
 * M2006电流

 */

#include "PID_task.h"
#include <math.h>

#if CHASSIS_TEST_MODE


/* ============================================================
 *                      测试参数
 * ============================================================ */

/* 每个动作目标角度 */
#define TEST_MOVE_ANGLE_DEG       360.0f

/* 原地旋转时使用的轮子角度 */
#define TEST_ROTATE_ANGLE_DEG     360.0f

/* 最大目标轮速，单位根据原工程 Speed 的单位，一般为 RPM */
#define TEST_MAX_SPEED_RPM        1000.0f

/* 最大电流 */
#define TEST_CURRENT_LIMIT        2500

/* 认为已经到达目标的角度误差 */
#define TEST_ANGLE_ERROR_DEG      3.0f

/* 认为已经基本停止的速度 */
#define TEST_SPEED_ERROR_RPM      30.0f

/* 上电后等待时间 */
#define TEST_START_DELAY_MS       2000U

/* 每个动作最大允许时间 */
#define TEST_ACTION_TIMEOUT_MS    5000U

/* 到达目标后保持停止时间 */
#define TEST_STOP_TIME_MS         1000U


/* ============================================================
 *                   轮子方向修正
 * ============================================================
 *
 * 如果以后发现：
 *
 * 目标角度增加，但是某个轮子实际角度减少，
 * 就把对应的 TEST_WHEEL_DIR 改成 -1。
 *
 * 正常情况下先全部使用 +1。
 */

#define TEST_WHEEL_DIR_1          1.0f
#define TEST_WHEEL_DIR_2          1.0f
#define TEST_WHEEL_DIR_3          1.0f
#define TEST_WHEEL_DIR_4          1.0f


/* ============================================================
 *                      测试状态
 * ============================================================ */

typedef enum
{
    TEST_START = 0,

    TEST_FORWARD,
    TEST_FORWARD_STOP,

    TEST_BACKWARD,
    TEST_BACKWARD_STOP,

    TEST_LEFT,
    TEST_LEFT_STOP,

    TEST_RIGHT,
    TEST_RIGHT_STOP,

    TEST_ROTATE_CCW,
    TEST_ROTATE_CCW_STOP,

    TEST_ROTATE_CW,
    TEST_ROTATE_CW_STOP,

    TEST_FINISHED

} ChassisTestState_t;


/* ============================================================
 *                   测试变量
 * ============================================================ */

static ChassisTestState_t test_state = TEST_START;

/* 四个轮子开始测试时的角度 */
static float test_start_angle[4] = {0};

/* 当前动作目标角度 */
static float test_target_angle[4] = {0};

/* 当前动作开始时间 */
static TickType_t test_state_tick = 0;


/* ============================================================
 *                      工具函数
 * ============================================================ */

/**
 * @brief 获取当前轮子角度
 */
static void Test_GetCurrentAngle(float *angle)
{
    angle[0] = motor_wheel_2006[0].Angle_DEG;
    angle[1] = motor_wheel_2006[1].Angle_DEG;
    angle[2] = motor_wheel_2006[2].Angle_DEG;
    angle[3] = motor_wheel_2006[3].Angle_DEG;
}


/**
 * @brief 设置目标角度
 */
static void Test_SetTargetAngle(float angle1,float angle2,float angle3,float angle4)
{
    test_target_angle[0] = angle1;
    test_target_angle[1] = angle2;
    test_target_angle[2] = angle3;
    test_target_angle[3] = angle4;
}


/**
 * @brief 判断当前位置是否到达目标
 */
static uint8_t Test_Arrived(void)
{
    float error0;
    float error1;
    float error2;
    float error3;

    error0 = fabsf(test_target_angle[0] -motor_wheel_2006[0].Angle_DEG);
    error1 = fabsf(test_target_angle[1] -motor_wheel_2006[1].Angle_DEG);
    error2 = fabsf(test_target_angle[2] -motor_wheel_2006[2].Angle_DEG);
    error3 = fabsf(test_target_angle[3] -motor_wheel_2006[3].Angle_DEG);


    /*
     * 角度已经比较接近目标，
     * 同时四个轮子的速度也已经比较低。
     */
    if ((error0 < TEST_ANGLE_ERROR_DEG) &&
        (error1 < TEST_ANGLE_ERROR_DEG) &&
        (error2 < TEST_ANGLE_ERROR_DEG) &&
        (error3 < TEST_ANGLE_ERROR_DEG))
    {
        if ((fabsf(motor_wheel_2006[0].Speed) < TEST_SPEED_ERROR_RPM) &&
            (fabsf(motor_wheel_2006[1].Speed) < TEST_SPEED_ERROR_RPM) &&
            (fabsf(motor_wheel_2006[2].Speed) < TEST_SPEED_ERROR_RPM) &&
            (fabsf(motor_wheel_2006[3].Speed) < TEST_SPEED_ERROR_RPM))
        {
            return 1;
        }
    }

    return 0;
}


/**
 * @brief 清零四个轮子的控制量
 */
static void Test_StopMotor(void)
{
    int16_t current[4] = {0, 0, 0, 0};

    /*
     * 清除位置 PID 的积分和输出，
     * 防止下一个动作开始时带着上一次动作的残留量。
     */
    PID_wheel_position[0].error_inter = 0;
    PID_wheel_position[0].pid_out = 0;

    PID_wheel_position[1].error_inter = 0;
    PID_wheel_position[1].pid_out = 0;

    PID_wheel_position[2].error_inter = 0;
    PID_wheel_position[2].pid_out = 0;

    PID_wheel_position[3].error_inter = 0;
    PID_wheel_position[3].pid_out = 0;


    /*
     * 速度 PID 也清掉积分。
     */
    M2006_Speed[0].error_inter = 0;
    M2006_Speed[1].error_inter = 0;
    M2006_Speed[2].error_inter = 0;
    M2006_Speed[3].error_inter = 0;

    M2006_Speed[0].pid_out = 0;
    M2006_Speed[1].pid_out = 0;
    M2006_Speed[2].pid_out = 0;
    M2006_Speed[3].pid_out = 0;


    MotorSend(&hcan1, 0x200U, current);
}


/**
 * @brief 进入下一个动作
 */
static void Test_NextState(ChassisTestState_t next_state)
{
    test_state = next_state;
    test_state_tick = xTaskGetTickCount();

    /*
     * 进入新动作前清 PID。
     */
    PID_wheel_position[0].error_inter = 0;
    PID_wheel_position[1].error_inter = 0;
    PID_wheel_position[2].error_inter = 0;
    PID_wheel_position[3].error_inter = 0;

    M2006_Speed[0].error_inter = 0;
    M2006_Speed[1].error_inter = 0;
    M2006_Speed[2].error_inter = 0;
    M2006_Speed[3].error_inter = 0;
}


/* ============================================================
 *                    设置各个动作目标
 * ============================================================ */


/**
 * @brief 前进
 *
 * 四个轮子目标角度全部增加。
 */
static void Test_SetForwardTarget(void)
{
    Test_SetTargetAngle(

        test_start_angle[0] +
        TEST_WHEEL_DIR_1 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[1] +
        TEST_WHEEL_DIR_2 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[2] +
        TEST_WHEEL_DIR_3 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[3] +
        TEST_WHEEL_DIR_4 * TEST_MOVE_ANGLE_DEG
    );
}


/**
 * @brief 后退
 */
static void Test_SetBackwardTarget(void)
{
    Test_SetTargetAngle(

        test_start_angle[0] -
        TEST_WHEEL_DIR_1 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[1] -
        TEST_WHEEL_DIR_2 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[2] -
        TEST_WHEEL_DIR_3 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[3] -
        TEST_WHEEL_DIR_4 * TEST_MOVE_ANGLE_DEG
    );
}


/**
 * @brief 左移
 *
 * 与 wheel_cal() 中 Ey 的正方向保持一致。
 */
static void Test_SetLeftTarget(void)
{
    Chassis_Motor_expect speed;

    /*
     * 这里只利用原来的 wheel_cal()，
     * 不重新写麦轮运动学。
     */
    wheel_cal(&speed, 0, TEST_MAX_SPEED_RPM, 0);

    Test_SetTargetAngle(

        test_start_angle[0] +
        TEST_WHEEL_DIR_1 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[1] +
        TEST_WHEEL_DIR_2 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[2] +
        TEST_WHEEL_DIR_3 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[3] +
        TEST_WHEEL_DIR_4 * TEST_MOVE_ANGLE_DEG
    );
}


/**
 * @brief 右移
 */
static void Test_SetRightTarget(void)
{
    Test_SetTargetAngle(

        test_start_angle[0] -
        TEST_WHEEL_DIR_1 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[1] -
        TEST_WHEEL_DIR_2 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[2] -
        TEST_WHEEL_DIR_3 * TEST_MOVE_ANGLE_DEG,

        test_start_angle[3] -
        TEST_WHEEL_DIR_4 * TEST_MOVE_ANGLE_DEG
    );
}


/**
 * @brief 原地逆时针
 *
 * 这里使用原来的 wheel_cal：
 *
 * wz > 0
 *
 * 得到：
 * 1、2轮负
 * 3、4轮正
 */
static void Test_SetRotateCCWTarget(void)
{
    Chassis_Motor_expect speed;

    wheel_cal(&speed, 0, 0, TEST_MAX_SPEED_RPM);

    Test_SetTargetAngle(

        test_start_angle[0] - TEST_WHEEL_DIR_1 * TEST_ROTATE_ANGLE_DEG,

        test_start_angle[1] - TEST_WHEEL_DIR_2 * TEST_ROTATE_ANGLE_DEG,

        test_start_angle[2] + TEST_WHEEL_DIR_3 * TEST_ROTATE_ANGLE_DEG,

        test_start_angle[3] + TEST_WHEEL_DIR_4 * TEST_ROTATE_ANGLE_DEG
    );
}


/**
 * @brief 原地顺时针
 */
static void Test_SetRotateCWTarget(void)
{
    Test_SetTargetAngle(

        test_start_angle[0] + TEST_WHEEL_DIR_1 * TEST_ROTATE_ANGLE_DEG,

        test_start_angle[1] + TEST_WHEEL_DIR_2 * TEST_ROTATE_ANGLE_DEG,

        test_start_angle[2] - TEST_WHEEL_DIR_3 * TEST_ROTATE_ANGLE_DEG,

        test_start_angle[3] - TEST_WHEEL_DIR_4 * TEST_ROTATE_ANGLE_DEG
    );
}


/* ============================================================
 *                    位置串级 PID
 * ============================================================ */

static void Test_PositionCascadePID(int16_t *current)
{
    float target_speed[4];


    /*
     * ========================================================
     * 第一层：位置环
     *
     * 目标：
     *
     *     目标角度 - 当前角度
     *
     * PID_Control_Smis()
     *
     * 输出：
     *
     *     目标角速度
     *
     * ========================================================
     */

    PID_Control_Smis(

        motor_wheel_2006[0].Angle_DEG,
        test_target_angle[0],
        &PID_wheel_position[0],
        motor_wheel_2006[0].Speed
    );

    PID_Control_Smis(

        motor_wheel_2006[1].Angle_DEG,
        test_target_angle[1],
        &PID_wheel_position[1],
        motor_wheel_2006[1].Speed
    );

    PID_Control_Smis(

        motor_wheel_2006[2].Angle_DEG,
        test_target_angle[2],
        &PID_wheel_position[2],
        motor_wheel_2006[2].Speed
    );

    PID_Control_Smis(

        motor_wheel_2006[3].Angle_DEG,
        test_target_angle[3],
        &PID_wheel_position[3],
        motor_wheel_2006[3].Speed
    );


    /*
     * PID_Control_Smis 的输出就是目标角速度。
     */
    target_speed[0] = PID_wheel_position[0].pid_out;
    target_speed[1] = PID_wheel_position[1].pid_out;
    target_speed[2] = PID_wheel_position[2].pid_out;
    target_speed[3] = PID_wheel_position[3].pid_out;


    /*
     * ========================================================
     * 限制位置环输出的最大目标速度
     *
     * 这不是新的 PID。
     * 只是防止位置误差太大时目标速度过高。
     * ========================================================
     */

    limit(target_speed[0],
          TEST_MAX_SPEED_RPM,
          -TEST_MAX_SPEED_RPM);

    limit(target_speed[1],
          TEST_MAX_SPEED_RPM,
          -TEST_MAX_SPEED_RPM);

    limit(target_speed[2],
          TEST_MAX_SPEED_RPM,
          -TEST_MAX_SPEED_RPM);

    limit(target_speed[3],
          TEST_MAX_SPEED_RPM,
          -TEST_MAX_SPEED_RPM);


    /*
     * ========================================================
     * 第二层：速度环
     *
     * 当前速度：
     *     motor_wheel_2006[i].Speed
     *
     * 目标速度：
     *     position PID 的输出
     *
     * 输出：
     *     M2006 电流
     * ========================================================
     */

    PID_Control(

        motor_wheel_2006[0].Speed,
        target_speed[0],
        &M2006_Speed[0]
    );

    PID_Control(

        motor_wheel_2006[1].Speed,
        target_speed[1],
        &M2006_Speed[1]
    );

    PID_Control(

        motor_wheel_2006[2].Speed,
        target_speed[2],
        &M2006_Speed[2]
    );

    PID_Control(motor_wheel_2006[3].Speed,target_speed[3],&M2006_Speed[3]);

    /*
     * ========================================================
     * 电流限制
     * ========================================================
     */

    limit(M2006_Speed[0].pid_out,TEST_CURRENT_LIMIT,-TEST_CURRENT_LIMIT);
    limit(M2006_Speed[1].pid_out,TEST_CURRENT_LIMIT,-TEST_CURRENT_LIMIT);
    limit(M2006_Speed[2].pid_out,TEST_CURRENT_LIMIT,-TEST_CURRENT_LIMIT);
    limit(M2006_Speed[3].pid_out,TEST_CURRENT_LIMIT,-TEST_CURRENT_LIMIT);

    current[0] = (int16_t)M2006_Speed[0].pid_out;
    current[1] = (int16_t)M2006_Speed[1].pid_out;
    current[2] = (int16_t)M2006_Speed[2].pid_out;
    current[3] = (int16_t)M2006_Speed[3].pid_out;
}


/* ============================================================
 *                       主测试任务
 * ============================================================ */

void ChassisTest_Task(void *pvParameters)
{
    TickType_t last_wake;
    TickType_t now;
    uint32_t elapsed_ms;

    int16_t current[4] = {0, 0, 0, 0};


    (void)pvParameters;


    /*
     * 等待底盘 CAN 和电机反馈稳定。
     */
    vTaskDelay(pdMS_TO_TICKS(TEST_START_DELAY_MS));


    /*
     * ========================================================
     * 记录测试开始时四个轮子的角度
     * ========================================================
     */

    Test_GetCurrentAngle(test_start_angle);


    /*
     * 第一个目标：前进
     */
    Test_SetForwardTarget();

    test_state = TEST_FORWARD;
    test_state_tick = xTaskGetTickCount();


    last_wake = xTaskGetTickCount();


    for (;;)
    {
        now = xTaskGetTickCount();

        elapsed_ms =
            (uint32_t)((now - test_state_tick) *
                       portTICK_PERIOD_MS);


        /* ====================================================
         * 状态机
         * ==================================================== */

        switch (test_state)
        {

        /* ----------------------------------------------------
         * 前进
         * ---------------------------------------------------- */

        case TEST_FORWARD:

            if (Test_Arrived() ||
                elapsed_ms >= TEST_ACTION_TIMEOUT_MS)
            {
                Test_StopMotor();

                Test_NextState(TEST_FORWARD_STOP);
            }

            break;


        /* ----------------------------------------------------
         * 前进停止
         * ---------------------------------------------------- */

        case TEST_FORWARD_STOP:

            Test_StopMotor();

            if (elapsed_ms >= TEST_STOP_TIME_MS)
            {
                Test_GetCurrentAngle(test_start_angle);

                Test_SetBackwardTarget();

                Test_NextState(TEST_BACKWARD);
            }

            break;


        /* ----------------------------------------------------
         * 后退
         * ---------------------------------------------------- */

        case TEST_BACKWARD:

            if (Test_Arrived() ||
                elapsed_ms >= TEST_ACTION_TIMEOUT_MS)
            {
                Test_StopMotor();

                Test_NextState(TEST_BACKWARD_STOP);
            }

            break;


        /* ----------------------------------------------------
         * 后退停止
         * ---------------------------------------------------- */

        case TEST_BACKWARD_STOP:

            Test_StopMotor();

            if (elapsed_ms >= TEST_STOP_TIME_MS)
            {
                Test_GetCurrentAngle(test_start_angle);

                Test_SetLeftTarget();

                Test_NextState(TEST_LEFT);
            }

            break;


        /* ----------------------------------------------------
         * 左移
         * ---------------------------------------------------- */

        case TEST_LEFT:

            if (Test_Arrived() ||
                elapsed_ms >= TEST_ACTION_TIMEOUT_MS)
            {
                Test_StopMotor();

                Test_NextState(TEST_LEFT_STOP);
            }

            break;


        /* ----------------------------------------------------
         * 左移停止
         * ---------------------------------------------------- */

        case TEST_LEFT_STOP:

            Test_StopMotor();

            if (elapsed_ms >= TEST_STOP_TIME_MS)
            {
                Test_GetCurrentAngle(test_start_angle);

                Test_SetRightTarget();

                Test_NextState(TEST_RIGHT);
            }

            break;


        /* ----------------------------------------------------
         * 右移
         * ---------------------------------------------------- */

        case TEST_RIGHT:

            if (Test_Arrived() ||
                elapsed_ms >= TEST_ACTION_TIMEOUT_MS)
            {
                Test_StopMotor();

                Test_NextState(TEST_RIGHT_STOP);
            }

            break;


        /* ----------------------------------------------------
         * 右移停止
         * ---------------------------------------------------- */

        case TEST_RIGHT_STOP:

            Test_StopMotor();

            if (elapsed_ms >= TEST_STOP_TIME_MS)
            {
                Test_GetCurrentAngle(test_start_angle);

                Test_SetRotateCCWTarget();

                Test_NextState(TEST_ROTATE_CCW);
            }

            break;


        /* ----------------------------------------------------
         * 原地逆时针
         * ---------------------------------------------------- */

        case TEST_ROTATE_CCW:

            if (Test_Arrived() ||
                elapsed_ms >= TEST_ACTION_TIMEOUT_MS)
            {
                Test_StopMotor();

                Test_NextState(TEST_ROTATE_CCW_STOP);
            }

            break;


        /* ----------------------------------------------------
         * 原地逆时针停止
         * ---------------------------------------------------- */

        case TEST_ROTATE_CCW_STOP:

            Test_StopMotor();

            if (elapsed_ms >= TEST_STOP_TIME_MS)
            {
                Test_GetCurrentAngle(test_start_angle);

                Test_SetRotateCWTarget();

                Test_NextState(TEST_ROTATE_CW);
            }

            break;


        /* ----------------------------------------------------
         * 原地顺时针
         * ---------------------------------------------------- */

        case TEST_ROTATE_CW:

            if (Test_Arrived() ||
                elapsed_ms >= TEST_ACTION_TIMEOUT_MS)
            {
                Test_StopMotor();

                Test_NextState(TEST_ROTATE_CW_STOP);
            }

            break;


        /* ----------------------------------------------------
         * 原地顺时针停止
         * ---------------------------------------------------- */

        case TEST_ROTATE_CW_STOP:

            Test_StopMotor();

            if (elapsed_ms >= TEST_STOP_TIME_MS)
            {
                Test_NextState(TEST_FINISHED);
            }

            break;


        /* ----------------------------------------------------
         * 测试结束
         * ---------------------------------------------------- */

        case TEST_FINISHED:

            Test_StopMotor();

            /*
             * 一直保持 0 电流。
             */
            vTaskDelayUntil(&last_wake,
                            pdMS_TO_TICKS(2U));

            continue;


        default:

            Test_StopMotor();

            Test_NextState(TEST_FINISHED);

            break;
        }


        /* ====================================================
         * 如果当前是运动状态，运行串级 PID
         * ==================================================== */

        if ((test_state == TEST_FORWARD) ||
            (test_state == TEST_BACKWARD) ||
            (test_state == TEST_LEFT) ||
            (test_state == TEST_RIGHT) ||
            (test_state == TEST_ROTATE_CCW) ||
            (test_state == TEST_ROTATE_CW))
        {
            Test_PositionCascadePID(current);

            MotorSend(&hcan1, 0x200U, current);
        }
        else
        {
            /*
             * STOP 状态直接发送 0。
             */
            Test_StopMotor();
        }


        /*
         * 控制周期保持原来的 2 ms。
         */
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(2U));
    }
}

#endif