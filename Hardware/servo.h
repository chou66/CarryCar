#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdint.h>

void Servo_Init(void);

void GripperServo_SetUs(uint16_t us);
void GripperServo_Open(void);
void GripperServo_Close(void);
void GripperServo_Mid(void);

void CarouselServo_SetUs(uint16_t us);
void CarouselServo_SetAngle(uint16_t angle_deg);
void CarouselServo_GotoSlot(uint8_t slot);

/* Legacy aliases for old competition code */
void Servo_SetUs(uint16_t us);
void Servo_Open(void);
void Servo_Close(void);
void Servo_Mid(void);


void TraySelectorServo_SetUs(uint16_t us);
void Pwm4Servo_SetUs(uint16_t us);

#endif
