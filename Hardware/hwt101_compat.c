#include "main.h"
#include "hwt101_compat.h"
#include "app_uart.h"
#include "hwt101_uart.h"

void HWT101_ZeroYaw(void)
{
    hwt101_set_zero(AppUart_GetImu());
}

float HWT101_GetYaw(void)
{
    return hwt101_yaw_relative_deg(AppUart_GetImu());
}

HWT101_Data_t HWT101_GetData(void)
{
    Hwt101 *imu = AppUart_GetImu();
    HWT101_Data_t out;
    out.roll = imu->roll_deg;
    out.pitch = imu->pitch_deg;
    out.yaw_raw = imu->yaw_deg;
    out.yaw = hwt101_yaw_relative_deg(imu);
    out.valid = imu->has_angle ? 1U : 0U;
    out.last_update_ms = imu->last_update_ms;
    return out;
}

uint8_t HWT101_IsOnline(void)
{
    return hwt101_is_fresh(AppUart_GetImu(), HAL_GetTick(), 500U) ? 1U : 0U;
}
