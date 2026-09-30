#include "k230.h"

#include <string.h>

/* CubeMX生成的串口句柄。 */
extern UART_HandleTypeDef huart1;

#define K230_FRAME_HEAD          '@'
#define K230_FIELD_SEPARATOR     ';'


/* =========================================================
 * 串口接收缓冲区
 * ========================================================= */
static uint8_t s_rx_byte = 0U;

static volatile uint8_t s_rx_ring[K230_RX_RING_SIZE];
static volatile uint16_t s_rx_write = 0U;
static volatile uint16_t s_rx_read = 0U;


/* =========================================================
 * 帧接收状态
 * ========================================================= */
static char s_frame[K230_FRAME_MAX_LENGTH + 1U];
static uint16_t s_frame_index = 0U;
static uint8_t s_receiving = 0U;
static uint8_t s_got_cr = 0U;


/* =========================================================
 * 对齐配置
 * ========================================================= */
static float s_target_x = K230_DEFAULT_TARGET_X;
static float s_target_y = K230_DEFAULT_TARGET_Y;

static float s_tolerance_x = K230_DEFAULT_TOLERANCE_X;
static float s_tolerance_y = K230_DEFAULT_TOLERANCE_Y;

static uint8_t s_required_stable_frames =
    K230_DEFAULT_STABLE_FRAMES;

static uint32_t s_data_timeout_ms =
    K230_DEFAULT_DATA_TIMEOUT_MS;


/* =========================================================
 * 数据与调试状态
 * ========================================================= */
static K230_Result_t s_result;
static uint32_t s_last_read_frame_id = 0U;

static volatile K230_Stats_t s_stats;


/* =========================================================
 * 内部工具函数
 * ========================================================= */
static float K230_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}


static void K230_ResetFrameReceiver(void)
{
    s_frame_index = 0U;
    s_receiving = 0U;
    s_got_cr = 0U;
    memset(s_frame, 0, sizeof(s_frame));
}


static void K230_ResetAllState(void)
{
    s_rx_write = 0U;
    s_rx_read = 0U;

    memset((void *)s_rx_ring, 0, sizeof(s_rx_ring));
    memset(&s_result, 0, sizeof(s_result));
    memset((void *)&s_stats, 0, sizeof(s_stats));

    s_last_read_frame_id = 0U;

    K230_ResetFrameReceiver();
}


static HAL_StatusTypeDef K230_StartReceive(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive_IT(
        &huart1,
        &s_rx_byte,
        1U
    );

    if (status != HAL_OK)
    {
        s_stats.restart_error_count++;
    }

    return status;
}


static void K230_PushByte(uint8_t data)
{
    uint16_t next_write;

    next_write =
        (uint16_t)((s_rx_write + 1U) % K230_RX_RING_SIZE);

    if (next_write == s_rx_read)
    {
        s_stats.ring_overflow_count++;
        return;
    }

    s_rx_ring[s_rx_write] = data;
    s_rx_write = next_write;

    s_stats.rx_byte_count++;
}


static uint8_t K230_PopByte(uint8_t *data)
{
    if ((data == NULL) || (s_rx_read == s_rx_write))
    {
        return 0U;
    }

    *data = s_rx_ring[s_rx_read];

    s_rx_read =
        (uint16_t)((s_rx_read + 1U) % K230_RX_RING_SIZE);

    return 1U;
}


/*
 * 支持以下两种格式：
 *
 * @x;y\r\n  or @x;y\n
 * @x;y;r\r\n or @x;y;r\n
 */
static uint8_t K230_ParseUnsignedDecimal(const char **cursor,
                                            float *value)
{
    const char *p;
    uint32_t integer_part = 0U;
    uint32_t fractional_part = 0U;
    uint32_t fractional_scale = 1U;
    uint8_t integer_digits = 0U;

    if ((cursor == NULL) || (*cursor == NULL) || (value == NULL))
    {
        return 0U;
    }

    p = *cursor;

    while ((*p >= '0') && (*p <= '9'))
    {
        integer_part = integer_part * 10U + (uint32_t)(*p - '0');
        p++;
        integer_digits++;
        if (integer_digits > 6U) return 0U;
    }

    if (*p == '.')
    {
        uint8_t fraction_digits = 0U;
        p++;

        while ((*p >= '0') && (*p <= '9'))
        {
            if (fraction_digits < 3U)
            {
                fractional_part =
                    fractional_part * 10U + (uint32_t)(*p - '0');
                fractional_scale *= 10U;
            }
            fraction_digits++;
            p++;
            if (fraction_digits > 6U) return 0U;
        }

        if ((integer_digits == 0U) && (fraction_digits == 0U))
        {
            return 0U;
        }
    }
    else if (integer_digits == 0U)
    {
        return 0U;
    }

    *value = (float)integer_part;
    if (fractional_scale > 1U)
    {
        *value += (float)fractional_part / (float)fractional_scale;
    }

    *cursor = p;
    return 1U;
}


/*
 * K230 frame payload:
 *   x;y
 *   x;y;r
 * Example: @103.00;57.00
 *
 * Coordinates/radius are non-negative.  A compact parser is used instead of the standard floating-point text parser, avoiding the large scanf/strtod library.
 */
static uint8_t K230_ParsePayload(const char *payload,
                                 float *x,
                                 float *y,
                                 float *radius,
                                 uint8_t *has_radius)
{
    const char *p = payload;
    float parsed_x;
    float parsed_y;
    float parsed_radius = 0.0f;
    uint8_t parsed_has_radius = 0U;

    if ((payload == NULL) ||
        (x == NULL) ||
        (y == NULL) ||
        (radius == NULL) ||
        (has_radius == NULL))
    {
        return 0U;
    }

    if (K230_ParseUnsignedDecimal(&p, &parsed_x) == 0U)
    {
        return 0U;
    }

    if (*p != K230_FIELD_SEPARATOR)
    {
        return 0U;
    }
    p++;

    if (K230_ParseUnsignedDecimal(&p, &parsed_y) == 0U)
    {
        return 0U;
    }

    if (*p == '\0')
    {
        parsed_has_radius = 0U;
    }
    else if (*p == K230_FIELD_SEPARATOR)
    {
        p++;
        if ((K230_ParseUnsignedDecimal(&p, &parsed_radius) == 0U) ||
            (*p != '\0') ||
            (parsed_radius <= 0.0f))
        {
            return 0U;
        }
        parsed_has_radius = 1U;
    }
    else
    {
        return 0U;
    }

    *x = parsed_x;
    *y = parsed_y;
    *radius = parsed_radius;
    *has_radius = parsed_has_radius;
    return 1U;
}


static void K230_UpdateAlignment(float x,
                                 float y,
                                 float radius,
                                 uint8_t has_radius)
{
    s_result.center_x = x;
    s_result.center_y = y;
    s_result.radius = radius;
    s_result.has_radius = has_radius;

    s_result.dx = x - s_target_x;
    s_result.dy = y - s_target_y;

    s_result.valid = 1U;
    s_result.timestamp_ms = HAL_GetTick();
    s_result.frame_id++;

    if ((K230_AbsFloat(s_result.dx) <= s_tolerance_x) &&
        (K230_AbsFloat(s_result.dy) <= s_tolerance_y))
    {
        s_result.in_tolerance = 1U;

        if (s_result.stable_count <
            s_required_stable_frames)
        {
            s_result.stable_count++;
        }

        if (s_result.stable_count >=
            s_required_stable_frames)
        {
            s_result.aligned = 1U;
        }
        else
        {
            s_result.aligned = 0U;
        }
    }
    else
    {
        s_result.in_tolerance = 0U;
        s_result.stable_count = 0U;
        s_result.aligned = 0U;
    }
}


static void K230_CompleteFrame(void)
{
    float x;
    float y;
    float radius;
    uint8_t has_radius;

    s_frame[s_frame_index] = '\0';

    if (K230_ParsePayload(
            s_frame,
            &x,
            &y,
            &radius,
            &has_radius) != 0U)
    {
        s_stats.valid_frame_count++;

        K230_UpdateAlignment(
            x,
            y,
            radius,
            has_radius
        );
    }
    else
    {
        s_stats.invalid_frame_count++;
    }

    K230_ResetFrameReceiver();
}


static void K230_HandleByte(uint8_t data)
{

    if (s_receiving == 0U)
    {
        if (data == (uint8_t)K230_FRAME_HEAD)
        {
            s_receiving = 1U;
            s_got_cr = 0U;
            s_frame_index = 0U;
            memset(s_frame, 0, sizeof(s_frame));
        }

        return;
    }

    if (s_got_cr != 0U)
    {
        if (data == '\n')
        {
            K230_CompleteFrame();
        }
        else
        {
            K230_ResetFrameReceiver();

            if (data == (uint8_t)K230_FRAME_HEAD)
            {
                s_receiving = 1U;
            }
        }

        return;
    }

    if (data == '\r')
    {
        s_got_cr = 1U;
        return;
    }

    if (data == '\n')
    {
        /*
         * Be tolerant of K230 scripts that terminate frames with LF only.
         * CRLF is still supported by the s_got_cr branch above.
         */
        K230_CompleteFrame();
        return;
    }

    if (data == (uint8_t)K230_FRAME_HEAD)
    {
        s_frame_index = 0U;
        s_got_cr = 0U;
        memset(s_frame, 0, sizeof(s_frame));
        return;
    }

    if (s_frame_index < K230_FRAME_MAX_LENGTH)
    {
        s_frame[s_frame_index++] = (char)data;
    }
    else
    {
        s_stats.invalid_frame_count++;
        K230_ResetFrameReceiver();
    }
}


/* =========================================================
 * 对外接口
 * ========================================================= */
HAL_StatusTypeDef K230_Init(void)
{
    HAL_StatusTypeDef status;

    K230_ResetAllState();

    /*
     * 清除USART1可能残留的错误状态。
     * CubeMX中仍需开启USART1 global interrupt。
     */
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    status = K230_StartReceive();

    return status;
}


void K230_Process(void)
{
    uint8_t data;

    while (K230_PopByte(&data) != 0U)
    {
        K230_HandleByte(data);
    }

    /*
     * 数据超时后清除稳定状态，
     * 防止底盘使用很久以前的ALIGNED结果。
     */
    if ((s_result.valid != 0U) &&
        ((HAL_GetTick() - s_result.timestamp_ms) >
         s_data_timeout_ms))
    {
        s_result.in_tolerance = 0U;
        s_result.stable_count = 0U;
        s_result.aligned = 0U;
    }
}


void K230_SetAlignmentConfig(float target_x,
                             float target_y,
                             float tolerance_x,
                             float tolerance_y,
                             uint8_t stable_frames)
{
    s_target_x = target_x;
    s_target_y = target_y;

    s_tolerance_x =
        (tolerance_x >= 0.0f) ? tolerance_x : -tolerance_x;

    s_tolerance_y =
        (tolerance_y >= 0.0f) ? tolerance_y : -tolerance_y;

    s_required_stable_frames =
        (stable_frames == 0U) ? 1U : stable_frames;

    K230_ResetAlignment();
}


void K230_SetDataTimeout(uint32_t timeout_ms)
{
    s_data_timeout_ms =
        (timeout_ms == 0U) ?
        K230_DEFAULT_DATA_TIMEOUT_MS :
        timeout_ms;
}




uint8_t K230_GetLatestResult(K230_Result_t *result)
{
    if ((result == NULL) || (s_result.valid == 0U))
    {
        return 0U;
    }

    *result = s_result;
    return 1U;
}


uint8_t K230_GetNewResult(K230_Result_t *result)
{
    if ((result == NULL) ||
        (s_result.valid == 0U) ||
        (s_result.frame_id == s_last_read_frame_id))
    {
        return 0U;
    }

    *result = s_result;
    s_last_read_frame_id = s_result.frame_id;

    return 1U;
}


uint8_t K230_IsAligned(void)
{
    if (s_result.valid == 0U)
    {
        return 0U;
    }

    if ((HAL_GetTick() - s_result.timestamp_ms) >
        s_data_timeout_ms)
    {
        return 0U;
    }

    return s_result.aligned;
}


void K230_ResetAlignment(void)
{
    s_result.in_tolerance = 0U;
    s_result.stable_count = 0U;
    s_result.aligned = 0U;
}


HAL_StatusTypeDef K230_SendString(const char *text)
{
    if (text == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(
        &huart1,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        100U
    );
}


const volatile K230_Stats_t *K230_GetStats(void)
{
    return &s_stats;
}


void K230_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) ||
        (huart->Instance != USART1))
    {
        return;
    }

    K230_PushByte(s_rx_byte);

    (void)K230_StartReceive();
}


void K230_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) ||
        (huart->Instance != USART1))
    {
        return;
    }

    s_stats.uart_error_count++;

    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    (void)HAL_UART_AbortReceive(&huart1);
    (void)K230_StartReceive();
}


uint8_t K230_FindSmallCircle(K230_Circle_t *circle,
                             uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint32_t start_frame_id;

    if (circle == NULL)
    {
        return 0U;
    }

    memset(circle, 0, sizeof(*circle));

    /*
     * K230 is a continuous transmitter in this project.
     * Process anything already buffered first.
     */
    K230_Process();

    /*
     * If the latest frame is fresh, use it immediately.
     * This is especially useful after a short chassis move: USART1
     * interrupts may already have buffered a post-move coordinate.
     */
    if ((s_result.valid != 0U) &&
        ((HAL_GetTick() - s_result.timestamp_ms) <=
         s_data_timeout_ms))
    {
        circle->center_x = s_result.center_x;
        circle->center_y = s_result.center_y;
        circle->radius = s_result.radius;
        circle->has_radius = s_result.has_radius;
        circle->valid = 1U;
        return 1U;
    }

    /*
     * Otherwise wait for the next streamed frame.
     * No command is transmitted to the K230.
     */
    start_ms = HAL_GetTick();
    start_frame_id = s_result.frame_id;

    while ((HAL_GetTick() - start_ms) < timeout_ms)
    {
        K230_Process();

        if ((s_result.valid != 0U) &&
            (s_result.frame_id != start_frame_id))
        {
            circle->center_x = s_result.center_x;
            circle->center_y = s_result.center_y;
            circle->radius = s_result.radius;
            circle->has_radius = s_result.has_radius;
            circle->valid = 1U;
            return 1U;
        }

        HAL_Delay(1U);
    }

    return 0U;
}


/* =========================================================
 * 可选：由k230.c直接拥有HAL UART回调
 * ========================================================= */
//#if (K230_OWNS_HAL_UART_CALLBACKS != 0U)

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    /*
//     * UART5：HWT101陀螺仪。
//     * 每收到1字节就交给HWT101协议解析，然后立即启动下一字节接收。
//     */
//    if (huart->Instance == UART5)
//    {
//        HWT101_RxByte(g_hwt101_rx_byte);

//        (void)HAL_UART_Receive_IT(
//            &huart5,
//            &g_hwt101_rx_byte,
//            1U
//        );

//        return;
//    }

//    /* USART1：K230原有接收逻辑继续保留 */
//    K230_RxCpltCallback(huart);
//}



//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    K230_ErrorCallback(huart);
//}

//#endif
