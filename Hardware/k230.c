#include "k230.h"
#include "usart.h"
#include <string.h>

#define K230_RING_SIZE 256U
#define K230_FRAME_MAX 63U

static volatile uint8_t s_ring[K230_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;

static char s_frame[K230_FRAME_MAX + 1U];
static uint8_t s_frame_len;
static uint8_t s_in_frame;

static K230_Result_t s_result;
static float s_target_x = K230_DEFAULT_TARGET_X;
static float s_target_y = K230_DEFAULT_TARGET_Y;
static float s_tol_x = K230_DEFAULT_TOLERANCE_X;
static float s_tol_y = K230_DEFAULT_TOLERANCE_Y;
static uint8_t s_stable_need = K230_DEFAULT_STABLE_FRAMES;

static float absf_local(float x) { return x >= 0.0f ? x : -x; }

static uint8_t parse_number(const char **pp, float *out)
{
    const char *p = *pp;
    float v = 0.0f;
    float frac = 0.1f;
    uint8_t neg = 0U;
    uint8_t any = 0U;

    if (*p == '-') { neg = 1U; ++p; }
    while (*p >= '0' && *p <= '9')
    {
        any = 1U;
        v = v * 10.0f + (float)(*p - '0');
        ++p;
    }
    if (*p == '.')
    {
        ++p;
        while (*p >= '0' && *p <= '9')
        {
            any = 1U;
            v += (float)(*p - '0') * frac;
            frac *= 0.1f;
            ++p;
        }
    }
    if (!any) return 0U;
    *out = neg ? -v : v;
    *pp = p;
    return 1U;
}

static void process_frame(void)
{
    const char *p = s_frame;
    float x, y, r = 0.0f;
    uint8_t has_r = 0U;

    if (*p == '@') ++p;
    if (!parse_number(&p, &x)) return;
    if (*p != ';' && *p != ',') return;
    ++p;
    if (!parse_number(&p, &y)) return;

    if (*p == ';' || *p == ',')
    {
        ++p;
        if (!parse_number(&p, &r)) return;
        has_r = 1U;
    }
    if (*p != '\0') return;

    s_result.center_x = x;
    s_result.center_y = y;
    s_result.radius = r;
    s_result.has_radius = has_r;
    s_result.dx = x - s_target_x;
    s_result.dy = y - s_target_y;
    s_result.timestamp_ms = HAL_GetTick();
    s_result.frame_id++;
    s_result.valid = 1U;

    if (absf_local(s_result.dx) <= s_tol_x &&
        absf_local(s_result.dy) <= s_tol_y)
    {
        s_result.in_tolerance = 1U;
        if (s_result.stable_count < s_stable_need)
            s_result.stable_count++;
        s_result.aligned = (s_result.stable_count >= s_stable_need) ? 1U : 0U;
    }
    else
    {
        s_result.in_tolerance = 0U;
        s_result.stable_count = 0U;
        s_result.aligned = 0U;
    }
}

void K230_Init(void)
{
    s_head = s_tail = 0U;
    s_frame_len = 0U;
    s_in_frame = 0U;
    memset(&s_result, 0, sizeof(s_result));
}

void K230_FeedByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_head + 1U) & (K230_RING_SIZE - 1U));
    if (next == s_tail)
        return;
    s_ring[s_head] = byte;
    s_head = next;
}

void K230_Process(void)
{
    while (s_tail != s_head)
    {
        uint8_t c = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) & (K230_RING_SIZE - 1U));

        if (!s_in_frame)
        {
            if (c == '@')
            {
                s_in_frame = 1U;
                s_frame_len = 0U;
                s_frame[s_frame_len++] = '@';
            }
            continue;
        }

        if (c == '\r' || c == '\n')
        {
            if (s_frame_len > 1U)
            {
                s_frame[s_frame_len] = '\0';
                process_frame();
            }
            s_frame_len = 0U;
            s_in_frame = 0U;
            continue;
        }

        if (s_frame_len < K230_FRAME_MAX)
            s_frame[s_frame_len++] = (char)c;
        else
        {
            s_frame_len = 0U;
            s_in_frame = 0U;
        }
    }
}

void K230_SetAlignmentConfig(float target_x, float target_y,
                             float tolerance_x, float tolerance_y,
                             uint8_t stable_frames)
{
    s_target_x = target_x;
    s_target_y = target_y;
    s_tol_x = tolerance_x > 0.0f ? tolerance_x : K230_DEFAULT_TOLERANCE_X;
    s_tol_y = tolerance_y > 0.0f ? tolerance_y : K230_DEFAULT_TOLERANCE_Y;
    s_stable_need = stable_frames ? stable_frames : 1U;
    K230_ResetAlignment();
}

void K230_ResetAlignment(void)
{
    s_result.stable_count = 0U;
    s_result.in_tolerance = 0U;
    s_result.aligned = 0U;
}

uint8_t K230_GetLatestResult(K230_Result_t *out)
{
    if (!out || !s_result.valid) return 0U;
    *out = s_result;
    return 1U;
}

uint8_t K230_IsAligned(void)
{
    return s_result.aligned;
}

uint8_t K230_IsOnline(uint32_t now_ms)
{
    return (s_result.valid &&
            (uint32_t)(now_ms - s_result.timestamp_ms) <=
            K230_DEFAULT_DATA_TIMEOUT_MS) ? 1U : 0U;
}

HAL_StatusTypeDef K230_SendString(const char *text)
{
    uint16_t len = 0U;
    if (!text) return HAL_ERROR;
    while (text[len] != '\0') ++len;
    return HAL_UART_Transmit(&huart4, (uint8_t *)text, len, 100U);
}
