#ifndef MECHANISM_MOTOR_H
#define MECHANISM_MOTOR_H

#include <stdint.h>
#include "motor.h"
#include "mechanism_config.h"

typedef enum {
    MECH_AXIS_HEIGHT = 0,
    MECH_AXIS_LENGTH = 1,
    MECH_AXIS_COUNT = 2
} MechanismAxis_t;

void Mechanism_Init(void);
void Mechanism_ControlTick(int16_t can2_current[4]);
void Mechanism_OnM2006Feedback(MechanismAxis_t axis, const uint8_t data[8]);

uint8_t Mechanism_MoveTo(MechanismAxis_t axis, float target_unit);
uint8_t Mechanism_MoveRelative(MechanismAxis_t axis, float delta_unit);
void Mechanism_MovePairTo(float height_unit, float length_unit);
void Mechanism_JogLength(int8_t direction);

void Mechanism_Up(uint32_t amount);
void Mechanism_Down(uint32_t amount);
void Mechanism_Length(uint32_t amount);
void Mechanism_Shorten(uint32_t amount);

void Mechanism_SavePosition(void);
void Mechanism_ApplyOrigin(MechanismAxis_t axis);
uint8_t Mechanism_FeedbackReady(MechanismAxis_t axis);

#endif
