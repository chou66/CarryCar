#ifndef __K230_H
#define __K230_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * 默认视觉对齐参数
 *
 * 当前K230检测图像尺寸为200×120，实测视觉中心约为(100, 59)。
 * ========================================================= */
#define K230_DEFAULT_TARGET_X             100.0f
#define K230_DEFAULT_TARGET_Y              59.0f
#define K230_DEFAULT_TOLERANCE_X            2.0f
#define K230_DEFAULT_TOLERANCE_Y            2.0f
#define K230_DEFAULT_STABLE_FRAMES          5U
#define K230_DEFAULT_DATA_TIMEOUT_MS      500U

#define K230_RX_RING_SIZE                 256U
#define K230_FRAME_MAX_LENGTH              63U

/*
 * 设为1：HAL_UART_RxCpltCallback和HAL_UART_ErrorCallback
 * 由k230.c统一定义。
 *
 * 使用此模式时，工程其他文件中不能再定义这两个HAL回调。
 *
 * 设为0：由工程已有的唯一HAL回调调用：
 * K230_RxCpltCallback(huart);
 * K230_ErrorCallback(huart);
 */
#ifndef K230_OWNS_HAL_UART_CALLBACKS
#define K230_OWNS_HAL_UART_CALLBACKS        0U
#endif


/* 保留原比赛任务使用的数据结构。 */
typedef struct
{
    float center_x;
    float center_y;
    float radius;

    /* 1：本帧包含半径；0：本帧只有x、y。 */
    uint8_t has_radius;

    uint8_t valid;
} K230_Circle_t;


/* 当前视觉坐标和对齐状态。 */
typedef struct
{
    float center_x;
    float center_y;
    float radius;

    float dx;
    float dy;

    uint8_t has_radius;
    uint8_t valid;
    uint8_t in_tolerance;
    uint8_t stable_count;
    uint8_t aligned;

    uint32_t frame_id;
    uint32_t timestamp_ms;
} K230_Result_t;


/* 串口接收统计，方便Keil Watch观察。 */
typedef struct
{
    uint32_t rx_byte_count;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t ring_overflow_count;
    uint32_t uart_error_count;
    uint32_t restart_error_count;
} K230_Stats_t;


/**
 * @brief 初始化K230串口模块并启动USART1单字节中断接收。
 *
 * 默认：
 * - K230串口：huart1
 * - USB转TTL调试输出：huart4
 * - 目标中心：(100, 59)
 * - 容差：X/Y各2像素
 * - 连续5帧对齐才确认
 */
HAL_StatusTypeDef K230_Init(void);


/**
 * @brief 在主循环中持续调用。
 *
 * 功能：
 * - 处理USART1中断收到的数据；
 * - 支持 @x;y\r\n 或 @x;y\n；
 * - 支持 @x;y;r\r\n 或 @x;y;r\n；
 * - 计算DX、DY；
 * - 完成连续稳定帧对齐判断；
 * - 可将原始数据和解析结果输出到UART4。
 */
void K230_Process(void);


/**
 * @brief 配置视觉目标中心、容差和连续稳定帧数量。
 */
void K230_SetAlignmentConfig(float target_x,
                             float target_y,
                             float tolerance_x,
                             float tolerance_y,
                             uint8_t stable_frames);


/**
 * @brief 设置视觉数据失效超时。
 *
 * 超过此时间未收到新帧，K230_IsAligned()将返回0。
 */
void K230_SetDataTimeout(uint32_t timeout_ms);


/**
 * @brief 设置UART4调试输出。
 *
 * @param enable_result_print 1：输出解析结果；0：关闭。
 * @param enable_raw_forward 1：将K230原始数据转发到UART4；0：关闭。
 */


/**
 * @brief 获取最近一帧结果。
 *
 * @return 1：存在有效结果；0：尚无有效视觉帧。
 */
uint8_t K230_GetLatestResult(K230_Result_t *result);


/**
 * @brief 判断是否收到新的有效视觉帧。
 *
 * 每帧只会通过此接口返回一次。
 */
uint8_t K230_GetNewResult(K230_Result_t *result);


/**
 * @brief 当前是否已稳定对齐。
 *
 * 必须同时满足：
 * - 最近数据未超时；
 * - 连续稳定帧达到设定数量。
 */
uint8_t K230_IsAligned(void);


/**
 * @brief 清除当前稳定计数和对齐状态。
 */
void K230_ResetAlignment(void);


/**
 * @brief STM32通过USART1向K230发送字符串。
 */
HAL_StatusTypeDef K230_SendString(const char *text);


/**
 * @brief 获取接收统计信息。
 */
const volatile K230_Stats_t *K230_GetStats(void);


/**
 * @brief USART1接收完成分发函数。
 *
 * 当K230_OWNS_HAL_UART_CALLBACKS为0时，
 * 在工程唯一的HAL_UART_RxCpltCallback中调用。
 */
void K230_RxCpltCallback(UART_HandleTypeDef *huart);


/**
 * @brief USART1错误分发函数。
 *
 * 当K230_OWNS_HAL_UART_CALLBACKS为0时，
 * 在工程唯一的HAL_UART_ErrorCallback中调用。
 */
void K230_ErrorCallback(UART_HandleTypeDef *huart);


/**
 * @brief 保留原比赛任务接口：等待下一帧有效视觉坐标。
 *
 * 支持：
 * - @x;y\r\n：radius=0，has_radius=0；
 * - @x;y;r\r\n：返回完整圆数据。
 *
 * @return 1：成功取得新帧；0：超时。
 */
uint8_t K230_FindSmallCircle(K230_Circle_t *circle,
                             uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
