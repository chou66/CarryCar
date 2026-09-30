#pragma once

#include <stdbool.h>

typedef struct {
    float kp;
    float kd;
    float max_w_radps;
    float tolerance_deg;
    float target_deg;
    float last_error_deg;
    bool enabled;
} HeadingController;

void heading_init(HeadingController *ctrl, float kp, float kd, float max_w_radps, float tolerance_deg);
void heading_set_target(HeadingController *ctrl, float target_deg);
/* dt_s must be the fixed scheduler period (recommended 0.01 s). */
float heading_update(HeadingController *ctrl, float yaw_deg, float gyro_z_dps, float dt_s);
bool heading_arrived(const HeadingController *ctrl, float yaw_deg);
