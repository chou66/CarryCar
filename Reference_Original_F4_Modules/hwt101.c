#include "hwt101.h"
#include <string.h>

static uint8_t rx_buf[11];
static uint8_t rx_index = 0;

static HWT101_Data_t hwt_data;
static float yaw_offset = 0.0f;

/**
 * @brief 执行HWT101陀螺仪相关功能。
 * @param angle 函数输入参数，具体取值请结合调用处和头文件定义使用。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 根据程序流程在需要时调用。
 */
static float AngleLimit(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

/**
 * @brief 初始化HWT101陀螺仪相关功能。
 * @note 通常在系统初始化阶段调用一次。
 */
void HWT101_Init(void)
{
    memset(&hwt_data, 0, sizeof(hwt_data));
    memset(rx_buf, 0, sizeof(rx_buf));

    rx_index = 0;
    yaw_offset = 0.0f;
}

/**
 * @brief 检测HWT101陀螺仪相关功能。
 * @param buf 函数输入参数，具体取值请结合调用处和头文件定义使用。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 根据程序流程在需要时调用。
 */
static uint8_t CheckSum(const uint8_t *buf)
{
    uint8_t sum = 0;

    for (uint8_t i = 0; i < 10; i++)
    {
        sum += buf[i];
    }

    return sum;
}

/**
 * @brief 解析HWT101陀螺仪相关功能。
 * @param buf 函数输入参数，具体取值请结合调用处和头文件定义使用。
 * @note 根据程序流程在需要时调用。
 */
static void ParseFrame(const uint8_t *buf)
{
    if (buf[0] != 0x55)
    {
        return;
    }

    if (CheckSum(buf) != buf[10])
    {
        return;
    }

    if (buf[1] == 0x53)
    {
        int16_t roll_raw;
        int16_t pitch_raw;
        int16_t yaw_raw;

        roll_raw  = (int16_t)((buf[3] << 8) | buf[2]);
        pitch_raw = (int16_t)((buf[5] << 8) | buf[4]);
        yaw_raw   = (int16_t)((buf[7] << 8) | buf[6]);

        hwt_data.roll =
            (float)roll_raw / 32768.0f * 180.0f;

        hwt_data.pitch =
            (float)pitch_raw / 32768.0f * 180.0f;

        hwt_data.yaw_raw =
            (float)yaw_raw / 32768.0f * 180.0f;

        hwt_data.yaw =
            AngleLimit(hwt_data.yaw_raw - yaw_offset);

        hwt_data.valid = 1;
        hwt_data.last_update_ms = HAL_GetTick();
    }
}

/**
 * @brief 接收HWT101陀螺仪相关功能。
 * @param byte 函数输入参数，具体取值请结合调用处和头文件定义使用。
 * @note 根据程序流程在需要时调用。
 */
void HWT101_RxByte(uint8_t byte)
{
    if (rx_index == 0)
    {
        if (byte == 0x55)
        {
            rx_buf[rx_index++] = byte;
        }

        return;
    }

    rx_buf[rx_index++] = byte;

    if (rx_index >= 11)
    {
        ParseFrame(rx_buf);
        rx_index = 0;
    }
}

/**
 * @brief 获取HWT101陀螺仪相关功能。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 可在主循环或任务周期中按需要调用。
 */
HWT101_Data_t HWT101_GetData(void)
{
    HWT101_Data_t data = hwt_data;

    data.yaw = AngleLimit(data.yaw_raw - yaw_offset);

    return data;
}

/**
 * @brief 获取HWT101陀螺仪相关功能。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 可在主循环或任务周期中按需要调用。
 */
float HWT101_GetYaw(void)
{
    /**
     * @brief 执行HWT101陀螺仪相关功能。
     * @param yaw_offset 函数输入参数，具体取值请结合调用处和头文件定义使用。
     * @retval 返回函数执行结果或读取到的数据。
     * @note 根据程序流程在需要时调用。
     */
    return AngleLimit(hwt_data.yaw_raw - yaw_offset);
}

/**
 * @brief 执行HWT101陀螺仪相关功能。
 * @note 根据程序流程在需要时调用。
 */
void HWT101_ZeroYaw(void)
{
    yaw_offset = hwt_data.yaw_raw;
    hwt_data.yaw = 0.0f;
}

/**
 * @brief 执行HWT101陀螺仪相关功能。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 根据程序流程在需要时调用。
 */
uint8_t HWT101_IsOnline(void)
{
    if (!hwt_data.valid)
    {
        return 0;
    }

    if (HAL_GetTick() - hwt_data.last_update_ms > 500)
    {
        return 0;
    }

    return 1;
}
