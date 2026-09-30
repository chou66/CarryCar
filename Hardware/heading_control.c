#include "heading_control.h"

static float wrap_deg(float x)
{
    while (x > 180.0f) x -= 360.0f;
    while (x < -180.0f) x += 360.0f;
    return x;
}

void heading_init(HeadingController *ctrl, float kp, float kd, float max_w, float tolerance)
{
    *ctrl = (HeadingController){.kp=kp, .kd=kd, .max_w_radps=max_w, .tolerance_deg=tolerance};
}

void heading_set_target(HeadingController *ctrl, float target_deg)
{
    ctrl->target_deg = wrap_deg(target_deg);
    ctrl->last_error_deg = 0.0f;
    ctrl->enabled = true;
}

float heading_update(HeadingController *ctrl, float yaw_deg, float gyro_z_dps, float dt_s)
{
    float err, w;
    (void)dt_s; /* gyro already gives the derivative in deg/s. */
    if (!ctrl->enabled) return 0.0f;
    err = wrap_deg(ctrl->target_deg - yaw_deg);
    if (err < ctrl->tolerance_deg && err > -ctrl->tolerance_deg) return 0.0f;
    w = (ctrl->kp * err - ctrl->kd * gyro_z_dps) * 0.01745329252f;
    if (w > ctrl->max_w_radps) w = ctrl->max_w_radps;
    if (w < -ctrl->max_w_radps) w = -ctrl->max_w_radps;
    ctrl->last_error_deg = err;
    return w;
}

bool heading_arrived(const HeadingController *ctrl, float yaw_deg)
{
    float e = wrap_deg(ctrl->target_deg - yaw_deg);
    return e < ctrl->tolerance_deg && e > -ctrl->tolerance_deg;
}
