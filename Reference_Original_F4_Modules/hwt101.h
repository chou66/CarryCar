#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"
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

/**
 * @brief 初始化HWT101陀螺仪相关功能。
 * @note 通常在系统初始化阶段调用一次。
 */
void HWT101_Init(void);
/**
 * @brief 接收HWT101陀螺仪相关功能。
 * @param byte 函数输入参数，具体取值请结合调用处和头文件定义使用。
 * @note 根据程序流程在需要时调用。
 */
void HWT101_RxByte(uint8_t byte);

/**
 * @brief 获取HWT101陀螺仪相关功能。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 可在主循环或任务周期中按需要调用。
 */
HWT101_Data_t HWT101_GetData(void);
/**
 * @brief 获取HWT101陀螺仪相关功能。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 可在主循环或任务周期中按需要调用。
 */
float HWT101_GetYaw(void);

/**
 * @brief 执行HWT101陀螺仪相关功能。
 * @note 根据程序流程在需要时调用。
 */
void HWT101_ZeroYaw(void);
/**
 * @brief 执行HWT101陀螺仪相关功能。
 * @retval 返回函数执行结果或读取到的数据。
 * @note 根据程序流程在需要时调用。
 */
uint8_t HWT101_IsOnline(void);
uint8_t HWT101_HeadingHoldStart(void);
void HWT101_HeadingHoldStop(void);
float HWT101_HeadingHoldGetCorrection(void);
float HWT101_HeadingHoldGetTarget(void);
uint8_t HWT101_HeadingHoldIsEnabled(void);

#endif
