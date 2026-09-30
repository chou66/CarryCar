#include "manipulator.h"
#include "emm_adapter.h"
#include "robot_config.h"

static uint8_t sign_dir(float x)
{
    return (x >= 0.0f) ? 0U : 1U;
}

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

HAL_StatusTypeDef Manipulator_Gimbal_Velocity(uint8_t dir, uint16_t rpm, uint8_t acc)
{
    return emm_vel(MOTOR_GIMBAL_ID, dir, rpm, acc, false);
}

HAL_StatusTypeDef Manipulator_Slide_Velocity(uint8_t dir, uint16_t rpm, uint8_t acc)
{
    return emm_vel(MOTOR_SLIDE_ID, dir, rpm, acc, false);
}

HAL_StatusTypeDef Manipulator_Gimbal_MovePulses(uint8_t dir, uint16_t rpm,
                                                uint8_t acc, uint32_t pulses)
{
    return emm_pos(MOTOR_GIMBAL_ID, dir, rpm, acc, pulses, false, false);
}

HAL_StatusTypeDef Manipulator_Slide_MovePulses(uint8_t dir, uint16_t rpm,
                                               uint8_t acc, uint32_t pulses)
{
    return emm_pos(MOTOR_SLIDE_ID, dir, rpm, acc, pulses, false, false);
}

bool Manipulator_Gimbal_RotateDeg(float degree, uint16_t rpm, uint8_t acc)
{
    uint32_t pulses;
    if (GIMBAL_PULSES_PER_DEG <= 0.0f)
        return false;
    pulses = (uint32_t)(absf_local(degree) * GIMBAL_PULSES_PER_DEG + 0.5f);
    return Manipulator_Gimbal_MovePulses(sign_dir(degree), rpm, acc, pulses) == HAL_OK;
}

bool Manipulator_Slide_MoveMm(float mm, uint16_t rpm, uint8_t acc)
{
    uint32_t pulses;
    if (SLIDE_PULSES_PER_MM <= 0.0f)
        return false;
    pulses = (uint32_t)(absf_local(mm) * SLIDE_PULSES_PER_MM + 0.5f);
    return Manipulator_Slide_MovePulses(sign_dir(mm), rpm, acc, pulses) == HAL_OK;
}

void Manipulator_StopAll(void)
{
    (void)emm_stop(MOTOR_GIMBAL_ID, false);
    (void)emm_stop(MOTOR_SLIDE_ID, false);
}
