#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SERVO_POSITION_MIN_US
#define SERVO_POSITION_MIN_US 500U
#endif

#ifndef SERVO_POSITION_MAX_US
#define SERVO_POSITION_MAX_US 2500U
#endif

/* Original carrycar high-level API. */
void Servo_Init(void);

void Servo_SetUs(uint16_t us);
void Servo_Open(void);
void Servo_Close(void);
void Servo_Mid(void);

void DiscServo_SetUs(uint16_t us);
void DiscServo_ToRear(void);
void DiscServo_ToFront(void);
void DiscServo_ToLinePickup(void);

void CarouselServo_SetUs(uint16_t us);
void CarouselServo_GotoSlot(uint8_t slot);
void CarouselServo_GotoIntakeSlot(uint8_t slot);
void CarouselServo_GotoReleaseSlot(uint8_t slot);
void CarouselServo_GotoColorSlot(uint8_t slot);

/*
 * Teammate-tested 360-degree servo API only.
 *
 * TIM4 CH3 / PD14: whole-disc pivot servo
 * TIM4 CH4 / PD15: internal carousel indexing servo
 */


void Servo_Pivot_180_Init();
void Servo_Spin_180_Init();
void Servo_Pivot_360_Init();
void Servo_Spin_360_Init();
void Servo_Pivot_180_SetAngle(uint8_t angle);
void Servo_Spin_180_SetAngle(uint8_t angle);
void Servo_Pivot_270_Init(void);
void Servo_Spin_270_Init(void);
void Servo_Pivot_270_SetAngle(uint16_t angle);
void Servo_Spin_270_SetAngle(uint16_t angle);
void Servo_Pivot_360_SetAngle(uint16_t angle);
void Servo_Spin_360_SetAngle(uint16_t angle);
void Servo_Pivot_360_SetAngle_Inverse(uint16_t angle);
void Servo_Spin_360_SetAngle_Inverse(uint16_t angle);


#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
