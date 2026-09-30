#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile float roll_deg;
    volatile float pitch_deg;
    volatile float yaw_deg;
    volatile float gyro_z_dps;
    volatile uint32_t last_update_ms;
    float yaw_zero_deg;
    bool has_angle;
} Hwt101;

void hwt101_init(Hwt101 *imu);
/* Call once for every byte received by the USART RX interrupt/DMA parser. */
void hwt101_feed_byte(Hwt101 *imu, uint8_t byte, uint32_t now_ms);
void hwt101_set_zero(Hwt101 *imu);
float hwt101_yaw_relative_deg(const Hwt101 *imu);
bool hwt101_is_fresh(const Hwt101 *imu, uint32_t now_ms, uint32_t timeout_ms);
