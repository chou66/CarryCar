#include "qr_scanner.h"
#include <string.h>

static char s_build[QR_CODE_MAX + 1U];
static char s_latest[QR_CODE_MAX + 1U];
static uint8_t s_len;
static volatile uint8_t s_updated;
static uint32_t s_last_byte_ms;
static uint32_t s_last_frame_ms;

static void finish_frame(uint32_t now_ms)
{
    if (s_len == 0U) return;
    s_build[s_len] = '\0';
    memcpy(s_latest, s_build, (size_t)s_len + 1U);
    s_updated = 1U;
    s_last_frame_ms = now_ms;
    s_len = 0U;
}

void QRScanner_Init(void)
{
    s_len = 0U;
    s_updated = 0U;
    s_last_byte_ms = 0U;
    s_last_frame_ms = 0U;
    s_build[0] = '\0';
    s_latest[0] = '\0';
}

void QRScanner_FeedByte(uint8_t byte, uint32_t now_ms)
{
    s_last_byte_ms = now_ms;

    if (byte == '\r' || byte == '\n' || byte == '\0')
    {
        finish_frame(now_ms);
        return;
    }

    if (s_len < QR_CODE_MAX)
        s_build[s_len++] = (char)byte;
    else
        s_len = 0U;
}

void QRScanner_Update(uint32_t now_ms)
{
    /* Same idea as the tested first project: modules without CR/LF end after 50 ms silence. */
    if (s_len > 0U && (uint32_t)(now_ms - s_last_byte_ms) >= 50U)
        finish_frame(now_ms);
}

static uint8_t copy_code(char *out, uint16_t max_len)
{
    uint16_t i = 0U;
    if (!out || max_len < 2U || s_latest[0] == '\0') return 0U;
    while (i + 1U < max_len && s_latest[i] != '\0')
    {
        out[i] = s_latest[i];
        ++i;
    }
    out[i] = '\0';
    return 1U;
}

uint8_t QRScanner_GetNewCode(char *out, uint16_t max_len)
{
    if (!s_updated) return 0U;
    if (!copy_code(out, max_len)) return 0U;
    s_updated = 0U;
    return 1U;
}

uint8_t QRScanner_GetLatestCode(char *out, uint16_t max_len)
{
    return copy_code(out, max_len);
}

uint8_t QRScanner_IsOnline(uint32_t now_ms)
{
    return (s_last_frame_ms != 0U &&
            (uint32_t)(now_ms - s_last_frame_ms) <= 2000U) ? 1U : 0U;
}
