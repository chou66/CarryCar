#ifndef __K230_H
#define __K230_H

#include "main.h"
#include <stdint.h>

#define K230_DEFAULT_TARGET_X          100.0f
#define K230_DEFAULT_TARGET_Y           59.0f
#define K230_DEFAULT_TOLERANCE_X         2.0f
#define K230_DEFAULT_TOLERANCE_Y         2.0f
#define K230_DEFAULT_STABLE_FRAMES       5U
#define K230_DEFAULT_DATA_TIMEOUT_MS   500U

typedef struct
{
    float center_x;
    float center_y;
    float radius;
    float dx;
    float dy;
    uint32_t timestamp_ms;
    uint32_t frame_id;
    uint8_t has_radius;
    uint8_t valid;
    uint8_t in_tolerance;
    uint8_t stable_count;
    uint8_t aligned;
} K230_Result_t;

void K230_Init(void);
void K230_FeedByte(uint8_t byte);
void K230_Process(void);
void K230_SetAlignmentConfig(float target_x, float target_y,
                             float tolerance_x, float tolerance_y,
                             uint8_t stable_frames);
void K230_ResetAlignment(void);
uint8_t K230_GetLatestResult(K230_Result_t *out);
uint8_t K230_IsAligned(void);
uint8_t K230_IsOnline(uint32_t now_ms);
HAL_StatusTypeDef K230_SendString(const char *text);

#endif
