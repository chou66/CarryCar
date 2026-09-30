#ifndef __QR_SCANNER_H
#define __QR_SCANNER_H

#include <stdint.h>

#define QR_CODE_MAX 63U

void QRScanner_Init(void);
void QRScanner_FeedByte(uint8_t byte, uint32_t now_ms);
void QRScanner_Update(uint32_t now_ms);
uint8_t QRScanner_GetNewCode(char *out, uint16_t max_len);
uint8_t QRScanner_GetLatestCode(char *out, uint16_t max_len);
uint8_t QRScanner_IsOnline(uint32_t now_ms);

#endif
