#ifndef __HWT101_COMPAT_H
#define __HWT101_COMPAT_H

#include <stdint.h>

typedef struct
{
    float roll;
    float pitch;
    float yaw_raw;
    float yaw;
    uint8_t valid;
    uint32_t last_update_ms;
} HWT101_Data_t;

void HWT101_ZeroYaw(void);
float HWT101_GetYaw(void);
HWT101_Data_t HWT101_GetData(void);
uint8_t HWT101_IsOnline(void);

#endif
