#ifndef __MJ6000_H
#define __MJ6000_H

#include "main.h"
#include <stdint.h>

#define MJ6000_OK             1U
#define MJ6000_TIMEOUT        0U
#define MJ6000_PARAM_ERROR    2U
#define MJ6000_OVERFLOW       3U

/**
 * @brief 清除USART2中残留的数据
 */
void MJ6000_ClearRx(void);

/**
 * @brief 读取一次二维码数据
 * @param buf        接收缓冲区
 * @param max_len    缓冲区总长度
 * @param timeout_ms 接收超时时间
 * @retval MJ6000_OK          接收成功
 *         MJ6000_TIMEOUT     超时未收到数据
 *         MJ6000_PARAM_ERROR 参数错误
 *         MJ6000_OVERFLOW    数据超过缓冲区
 */
uint8_t MJ6000_ReadCode(char *buf,
                        uint16_t max_len,
                        uint32_t timeout_ms);

#endif
