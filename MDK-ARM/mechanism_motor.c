#include "mechanism_motor.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "CRC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
#include "Auto.h"
#include "w25q64.h"

extern int32_t StepCounter[2];
extern int8_t Dir[2];

typedef struct {
    M2006_TypeDef motor;
    float boot_motor_deg;
    float boot_saved_unit;
    volatile float current_unit;
    volatile float target_unit;
    float integral;
    float previous_speed_error;
    volatile uint8_t feedback_ready;
    volatile uint16_t stable_count;
} MechanismMotorState_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    int32_t position_unit[2];
    uint16_t crc16;
    uint16_t reserved2;
} MechanismPositionRecord_t;

#define MECH_POSITION_MAGIC   0x4D433032UL /* ASCII含义：MC02 */
#define MECH_POSITION_VERSION 1U

static MechanismMotorState_t state[MECH_AXIS_COUNT];
static uint8_t flash_ready;

/* 判断某一机构轴在本版本中是否由M2006驱动。 */
static uint8_t AxisUsesM2006(MechanismAxis_t axis)
{
    return (axis == MECH_AXIS_HEIGHT) ? MECH_HEIGHT_USE_M2006 : MECH_LENGTH_USE_M2006;
}

/* 返回逻辑正方向对应的编码器方向。 */
static float AxisSign(MechanismAxis_t axis)
{
    return (axis == MECH_AXIS_HEIGHT) ? MECH_HEIGHT_MOTOR_SIGN : MECH_LENGTH_MOTOR_SIGN;
}

/* 返回M2006转子角度到原StepCounter单位的标定比例。 */
static float AxisDegreePerUnit(MechanismAxis_t axis)
{
    return (axis == MECH_AXIS_HEIGHT) ? MECH_HEIGHT_DEG_PER_LEGACY_UNIT
                                      : MECH_LENGTH_DEG_PER_LEGACY_UNIT;
}

/* 返回某轴的软件最小位置。 */
static float AxisMin(MechanismAxis_t axis)
{
    return (axis == MECH_AXIS_HEIGHT) ? MECH_HEIGHT_MIN_UNIT : MECH_LENGTH_MIN_UNIT;
}

/* 返回某轴的软件最大位置。 */
static float AxisMax(MechanismAxis_t axis)
{
    return (axis == MECH_AXIS_HEIGHT) ? MECH_HEIGHT_MAX_UNIT : MECH_LENGTH_MAX_UNIT;
}

/* 返回某轴位置外环的比例系数。 */
static float AxisPositionKp(MechanismAxis_t axis)
{
    return (axis == MECH_AXIS_HEIGHT) ? MECH_HEIGHT_POSITION_KP : MECH_LENGTH_POSITION_KP;
}

/* 返回某轴允许的最高电机转速。 */
static float AxisMaxSpeed(MechanismAxis_t axis)
{
    return (axis == MECH_AXIS_HEIGHT) ? MECH_HEIGHT_MAX_SPEED_RPM : MECH_LENGTH_MAX_SPEED_RPM;
}

/* 把数值限制在给定上下界内。 */
static float ClampFloat(float value, float upper, float lower)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}

/* 从板载W25Q64恢复上一次正常保存的机构逻辑位置。 */
static void LoadSavedPosition(void)
{
    MechanismPositionRecord_t record;

    StepCounter[0] = 0;
    StepCounter[1] = 0;
#if MECH_POSITION_PERSIST_ENABLE
    flash_ready = (OSPI_W25Qxx_Init() == OSPI_W25Qxx_OK);
    if (!flash_ready ||
        OSPI_W25Qxx_ReadBuffer((uint8_t *)&record,
                               MECH_POSITION_FLASH_ADDRESS,
                               sizeof(record)) != OSPI_W25Qxx_OK) {
        return;
    }
    if ((record.magic != MECH_POSITION_MAGIC) ||
        (record.version != MECH_POSITION_VERSION) ||
        (record.crc16 != Verify_CRC16_Check_Sum((uint8_t *)&record,
                                                offsetof(MechanismPositionRecord_t, crc16)))) {
        return;
    }
    StepCounter[0] = record.position_unit[0];
    StepCounter[1] = record.position_unit[1];
#else
    flash_ready = 0U;
#endif
}

/* 初始化两个机构轴的状态、目标和断电位置基准。 */
void Mechanism_Init(void)
{
    memset(state, 0, sizeof(state));
    LoadSavedPosition();
    for (uint32_t i = 0; i < MECH_AXIS_COUNT; ++i) {
        state[i].current_unit = (float)StepCounter[i];
        state[i].target_unit = state[i].current_unit;
        state[i].boot_saved_unit = state[i].current_unit;
    }
}

/* 接收某个机构M2006的反馈，并换算成连续的原StepCounter逻辑位置。 */
void Mechanism_OnM2006Feedback(MechanismAxis_t axis, const uint8_t data[8])
{
    MechanismMotorState_t *s;
    uint16_t angle;

    if ((axis >= MECH_AXIS_COUNT) || !AxisUsesM2006(axis)) return;
    s = &state[axis];
    if (!s->feedback_ready) {
        /* 第一帧只建立编码器基准，避免把上电机械角误判成跨圈。 */
        angle = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
        s->motor.MchanicalAngle = angle;
        s->motor.LsatAngle = angle;
        s->motor.Speed = (int16_t)(((uint16_t)data[2] << 8U) | data[3]);
        s->motor.r = 0;
        s->motor.Angle = angle;
        s->motor.Angle_DEG = (float)angle * 0.0439453125f;
        s->boot_motor_deg = s->motor.Angle_DEG;
        s->boot_saved_unit = (float)StepCounter[axis];
        s->current_unit = s->boot_saved_unit;
        s->target_unit = s->current_unit;
        s->feedback_ready = 1U;
        return;
    }

    M2006_Receive(&s->motor, (uint8_t *)data);
    s->current_unit = s->boot_saved_unit +
                      AxisSign(axis) * (s->motor.Angle_DEG - s->boot_motor_deg) /
                      AxisDegreePerUnit(axis);
    StepCounter[axis] = (int32_t)lroundf(s->current_unit);
}

/* 每2ms执行一次位置外环和速度内环，输出CAN2四路电流命令。 */
void Mechanism_ControlTick(int16_t can2_current[4])
{
    for (uint32_t i = 0; i < MECH_AXIS_COUNT; ++i) {
        MechanismMotorState_t *s = &state[i];
        float position_error;
        float target_speed;
        float speed_error;
        float output;

        if (!AxisUsesM2006((MechanismAxis_t)i) || !s->feedback_ready) continue;

        position_error = s->target_unit - s->current_unit;
        target_speed = ClampFloat(position_error * AxisPositionKp((MechanismAxis_t)i),
                                  AxisMaxSpeed((MechanismAxis_t)i),
                                  -AxisMaxSpeed((MechanismAxis_t)i));
        /* 电机方向符号也要作用到目标转速，否则位置反馈正方向改变后闭环会反向。 */
        target_speed *= AxisSign((MechanismAxis_t)i);
        speed_error = target_speed - (float)s->motor.Speed;
        if (MECH_SPEED_KI > 0.000001f) {
            s->integral = ClampFloat(s->integral + speed_error * 0.002f,
                                     MECH_CURRENT_LIMIT / MECH_SPEED_KI,
                                     -MECH_CURRENT_LIMIT / MECH_SPEED_KI);
        } else {
            /* 允许调试时把Ki设为0，不产生除零。 */
            s->integral = 0.0f;
        }
        output = MECH_SPEED_KP * speed_error + MECH_SPEED_KI * s->integral +
                 MECH_SPEED_KD * (speed_error - s->previous_speed_error) / 0.002f;
        s->previous_speed_error = speed_error;
        output = ClampFloat(output, MECH_CURRENT_LIMIT, -MECH_CURRENT_LIMIT);

        can2_current[i] = (int16_t)output; /* 0x201对应下标0，0x202对应下标1。 */
        if ((fabsf(position_error) <= MECH_POSITION_TOLERANCE_UNIT) &&
            (abs(s->motor.Speed) <= (int)MECH_SPEED_TOLERANCE_RPM)) {
            if (s->stable_count < MECH_STABLE_CYCLES) ++s->stable_count;
        } else {
            s->stable_count = 0U;
        }
    }
}

/* 查询指定机构轴是否已经收到有效M2006反馈。 */
uint8_t Mechanism_FeedbackReady(MechanismAxis_t axis)
{
    if (axis >= MECH_AXIS_COUNT) return 0U;
    return AxisUsesM2006(axis) ? state[axis].feedback_ready : 1U;
}

/* 把单个机构轴移动到绝对逻辑位置，并等待稳定到位或超时。 */
uint8_t Mechanism_MoveTo(MechanismAxis_t axis, float target_unit)
{
    TickType_t start_tick;
    MechanismMotorState_t *s;

    if (axis >= MECH_AXIS_COUNT) return 0U;
    if (!AxisUsesM2006(axis)) {
        const float safe_target = ClampFloat(target_unit, AxisMax(axis), AxisMin(axis));
        int32_t delta = (int32_t)lroundf(safe_target) - StepCounter[axis];
        if (axis == MECH_AXIS_HEIGHT) {
            if (delta > 0) Mechanism_Down((uint32_t)delta);
            else if (delta < 0) Mechanism_Up((uint32_t)(-delta));
        }
        return 1U;
    }

    s = &state[axis];
    start_tick = xTaskGetTickCount();
    while (!s->feedback_ready) {
        if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(MECH_FEEDBACK_WAIT_MS)) return 0U;
        vTaskDelay(pdMS_TO_TICKS(2U));
    }

    s->target_unit = ClampFloat(target_unit, AxisMax(axis), AxisMin(axis));
    s->stable_count = 0U;
    start_tick = xTaskGetTickCount();
    while (s->stable_count < MECH_STABLE_CYCLES) {
        if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(MECH_MOVE_TIMEOUT_MS)) {
            s->target_unit = s->current_unit;
            return 0U;
        }
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
    Mechanism_SavePosition();
    return 1U;
}

/* 以当前逻辑位置为基准，移动指定的相对量。 */
uint8_t Mechanism_MoveRelative(MechanismAxis_t axis, float delta_unit)
{
    float origin = AxisUsesM2006(axis) ? state[axis].current_unit : (float)StepCounter[axis];
    return Mechanism_MoveTo(axis, origin + delta_unit);
}

/* 按原Auto.c语义控制升降和伸缩两个轴到位。 */
void Mechanism_MovePairTo(float height_unit, float length_unit)
{
    TickType_t start_tick;

    if (MECH_HEIGHT_USE_M2006 && MECH_LENGTH_USE_M2006) {
        /* 原步进代码会让两轴同时动作；这里也同时下发两个目标，再统一等待到位。 */
        start_tick = xTaskGetTickCount();
        while (!state[MECH_AXIS_HEIGHT].feedback_ready ||
               !state[MECH_AXIS_LENGTH].feedback_ready) {
            if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(MECH_FEEDBACK_WAIT_MS)) return;
            vTaskDelay(pdMS_TO_TICKS(2U));
        }
        state[MECH_AXIS_HEIGHT].target_unit = ClampFloat(height_unit,
                                                         AxisMax(MECH_AXIS_HEIGHT),
                                                         AxisMin(MECH_AXIS_HEIGHT));
        state[MECH_AXIS_LENGTH].target_unit = ClampFloat(length_unit,
                                                         AxisMax(MECH_AXIS_LENGTH),
                                                         AxisMin(MECH_AXIS_LENGTH));
        state[MECH_AXIS_HEIGHT].stable_count = 0U;
        state[MECH_AXIS_LENGTH].stable_count = 0U;
        start_tick = xTaskGetTickCount();
        while ((state[MECH_AXIS_HEIGHT].stable_count < MECH_STABLE_CYCLES) ||
               (state[MECH_AXIS_LENGTH].stable_count < MECH_STABLE_CYCLES)) {
            if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(MECH_MOVE_TIMEOUT_MS)) {
                state[MECH_AXIS_HEIGHT].target_unit = state[MECH_AXIS_HEIGHT].current_unit;
                state[MECH_AXIS_LENGTH].target_unit = state[MECH_AXIS_LENGTH].current_unit;
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(2U));
        }
        Mechanism_SavePosition();
    } else if (!MECH_HEIGHT_USE_M2006 && MECH_LENGTH_USE_M2006) {
        /* 混合版本先给伸缩M2006目标；升降步进阻塞运行时，PID任务仍会让伸缩并行动作。 */
        start_tick = xTaskGetTickCount();
        while (!state[MECH_AXIS_LENGTH].feedback_ready) {
            if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(MECH_FEEDBACK_WAIT_MS)) return;
            vTaskDelay(pdMS_TO_TICKS(2U));
        }
        state[MECH_AXIS_LENGTH].target_unit = ClampFloat(length_unit,
                                                         AxisMax(MECH_AXIS_LENGTH),
                                                         AxisMin(MECH_AXIS_LENGTH));
        state[MECH_AXIS_LENGTH].stable_count = 0U;
        (void)Mechanism_MoveTo(MECH_AXIS_HEIGHT, height_unit);
        start_tick = xTaskGetTickCount();
        while (state[MECH_AXIS_LENGTH].stable_count < MECH_STABLE_CYCLES) {
            if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(MECH_MOVE_TIMEOUT_MS)) {
                state[MECH_AXIS_LENGTH].target_unit = state[MECH_AXIS_LENGTH].current_unit;
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(2U));
        }
        Mechanism_SavePosition();
    } else {
        (void)Mechanism_MoveTo(MECH_AXIS_HEIGHT, height_unit);
        (void)Mechanism_MoveTo(MECH_AXIS_LENGTH, length_unit);
    }
}

/* 视觉对准期间，小步修改伸缩M2006的位置目标。 */
void Mechanism_JogLength(int8_t direction)
{
    if (!state[MECH_AXIS_LENGTH].feedback_ready) return;
    if (direction > 0) state[MECH_AXIS_LENGTH].target_unit += MECH_JOG_UNIT_PER_CALL;
    else if (direction < 0) state[MECH_AXIS_LENGTH].target_unit -= MECH_JOG_UNIT_PER_CALL;
    state[MECH_AXIS_LENGTH].target_unit = ClampFloat(state[MECH_AXIS_LENGTH].target_unit,
                                                     AxisMax(MECH_AXIS_LENGTH),
                                                     AxisMin(MECH_AXIS_LENGTH));
}

/* 执行保留版本的升降步进动作，并继续维护StepCounter。 */
static void RunHeightStepper(GPIO_PinState direction, int8_t count_direction, uint32_t amount)
{
    __StepMotor1_Dir(direction);
    Dir[0] = count_direction;
    htim2.Instance->CCR1 = 39U;
    vTaskDelay(pdMS_TO_TICKS(amount));
    htim2.Instance->CCR1 = 0U;
    Mechanism_SavePosition();
}

/* 保留学长“升高”动作名的实现。 */
void Mechanism_Up(uint32_t amount)
{
    if (MECH_HEIGHT_USE_M2006) (void)Mechanism_MoveRelative(MECH_AXIS_HEIGHT, -(float)amount);
    else RunHeightStepper(GPIO_PIN_RESET, -1, amount);
}

/* 保留学长“下降”动作名的实现。 */
void Mechanism_Down(uint32_t amount)
{
    if (MECH_HEIGHT_USE_M2006) (void)Mechanism_MoveRelative(MECH_AXIS_HEIGHT, (float)amount);
    else RunHeightStepper(GPIO_PIN_SET, 1, amount);
}

/* 保留学长“伸长”动作名的实现。 */
void Mechanism_Length(uint32_t amount)
{
    (void)Mechanism_MoveRelative(MECH_AXIS_LENGTH, (float)amount);
}

/* 保留学长“缩短”动作名的实现。 */
void Mechanism_Shorten(uint32_t amount)
{
    (void)Mechanism_MoveRelative(MECH_AXIS_LENGTH, -(float)amount);
}

/* 将两个StepCounter逻辑位置连同CRC写入板载W25Q64。 */
void Mechanism_SavePosition(void)
{
#if MECH_POSITION_PERSIST_ENABLE
    MechanismPositionRecord_t record = {0};
    if (!flash_ready) return;
    record.magic = MECH_POSITION_MAGIC;
    record.version = MECH_POSITION_VERSION;
    record.position_unit[0] = StepCounter[0];
    record.position_unit[1] = StepCounter[1];
    record.crc16 = Verify_CRC16_Check_Sum((uint8_t *)&record,
                                          offsetof(MechanismPositionRecord_t, crc16));
    if (OSPI_W25Qxx_SectorErase(MECH_POSITION_FLASH_ADDRESS) == OSPI_W25Qxx_OK) {
        (void)OSPI_W25Qxx_WriteBuffer((uint8_t *)&record,
                                      MECH_POSITION_FLASH_ADDRESS,
                                      sizeof(record));
    }
#endif
}

/* 原点开关以后触发时，把指定轴当前位置重新定义为逻辑0。 */
void Mechanism_ApplyOrigin(MechanismAxis_t axis)
{
    if (axis >= MECH_AXIS_COUNT) return;
    /* 后续原点开关触发时调用：当前机械点被定义为逻辑0，并立即保存。 */
    StepCounter[axis] = 0;
    state[axis].current_unit = 0.0f;
    state[axis].target_unit = 0.0f;
    state[axis].boot_saved_unit = 0.0f;
    state[axis].boot_motor_deg = state[axis].motor.Angle_DEG;
    Mechanism_SavePosition();
}
