#include "mj6000.h"
#include "app_control.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

/**
 * @brief 清除USART2接收寄存器中的残留数据
 */
void MJ6000_ClearRx(void)
{
    uint8_t temp;

    while (HAL_UART_Receive(&huart2,
                            &temp,
                            1,
                            1) == HAL_OK)
    {
    }

    /*
     * 清除UART可能出现的溢出错误。
     */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

/**
 * @brief 阻塞等待并读取一次二维码数据
 */
uint8_t MJ6000_ReadCode(char *buf,
                        uint16_t max_len,
                        uint32_t timeout_ms)
{
    uint8_t ch;
    uint16_t index = 0;
    uint32_t start_tick;
    uint32_t last_byte_tick = 0;
    uint8_t received_any = 0;

    if ((buf == NULL) || (max_len < 2U))
    {
        return MJ6000_PARAM_ERROR;
    }

    memset(buf, 0, max_len);

    start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        AppControl_Service();
        if (AppControl_StopRequested() != 0U)
        {
            return MJ6000_TIMEOUT;
        }

        if (HAL_UART_Receive(&huart2,
                             &ch,
                             1,
                             10) == HAL_OK)
        {
            received_any = 1U;
            last_byte_tick = HAL_GetTick();

            /*
             * 收到回车、换行或0，认为一帧数据结束。
             */
            if ((ch == '\r') ||
                (ch == '\n') ||
                (ch == '\0'))
            {
                if (index > 0U)
                {
                    buf[index] = '\0';
                    return MJ6000_OK;
                }

                /*
                 * 忽略二维码数据前面的回车或换行。
                 */
                continue;
            }

            if (index < (uint16_t)(max_len - 1U))
            {
                buf[index] = (char)ch;
                index++;
            }
            else
            {
                buf[max_len - 1U] = '\0';
                return MJ6000_OVERFLOW;
            }
        }
        else
        {
            /*
             * 有些模块没有输出回车换行。
             * 已收到数据后，连续50ms没有新字节，
             * 认为本次二维码数据接收结束。
             */
            if ((received_any != 0U) &&
                ((HAL_GetTick() - last_byte_tick) >= 50U))
            {
                buf[index] = '\0';

                if (index > 0U)
                {
                    return MJ6000_OK;
                }
            }
        }
    }

    /*
     * 超时前已经收到部分完整数据，也作为成功返回。
     */
    if (index > 0U)
    {
        buf[index] = '\0';
        return MJ6000_OK;
    }

    return MJ6000_TIMEOUT;
}
