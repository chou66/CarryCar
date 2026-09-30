#include "servo.h"
#include "tim.h"
#include "robot_config.h"

static uint16_t clamp_us(uint16_t us)
{
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

void Servo_Init(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_SAFE_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_SAFE_US);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void GripperServo_SetUs(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, clamp_us(us));
}

void GripperServo_Open(void)  { GripperServo_SetUs(GRIPPER_OPEN_US); }
void GripperServo_Close(void) { GripperServo_SetUs(GRIPPER_CLOSE_US); }
void GripperServo_Mid(void)   { GripperServo_SetUs(GRIPPER_MID_US); }

void CarouselServo_SetUs(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, clamp_us(us));
}

void CarouselServo_SetAngle(uint16_t angle_deg)
{
    uint32_t span = (uint32_t)SERVO_MAX_US - (uint32_t)SERVO_MIN_US;
    uint32_t offset;
    uint16_t us;

    if (angle_deg > CAROUSEL_TRAVEL_DEG)
        angle_deg = CAROUSEL_TRAVEL_DEG;

    offset = ((uint32_t)angle_deg * span) / CAROUSEL_TRAVEL_DEG;

#if CAROUSEL_REVERSE
    us = (uint16_t)((uint32_t)SERVO_MAX_US - offset);
#else
    us = (uint16_t)((uint32_t)SERVO_MIN_US + offset);
#endif
    CarouselServo_SetUs(us);
}

void CarouselServo_GotoSlot(uint8_t slot)
{
    uint16_t angle;
    if (slot > 2U) return;

    angle = (uint16_t)(CAROUSEL_SLOT0_DEG +
                       (uint16_t)slot * CAROUSEL_SLOT_STEP_DEG);
    while (angle >= 360U) angle = (uint16_t)(angle - 360U);

    /* With the default 270-degree positional servo all 0/120/240 slots fit. */
    if (angle <= CAROUSEL_TRAVEL_DEG)
        CarouselServo_SetAngle(angle);
}

void Servo_SetUs(uint16_t us) { GripperServo_SetUs(us); }
void Servo_Open(void)         { GripperServo_Open(); }
void Servo_Close(void)        { GripperServo_Close(); }
void Servo_Mid(void)          { GripperServo_Mid(); }
