#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"

#define CAN_MOTOR_MAX_ID 6U

extern volatile uint32_t can_error_step;
extern volatile uint32_t can_error_code;
extern volatile uint32_t can_error_count;
extern volatile uint32_t can_psr;
extern volatile uint32_t can_psr_tec;
extern volatile uint32_t can_busoff_cnt;

extern volatile uint8_t can_rx_flag;
extern FDCAN_RxHeaderTypeDef can_rx_header;
extern uint8_t can_rx_data[8];

uint8_t can_SendCmd(__IO uint8_t *cmd, uint8_t len);
void fdcan2_UserInit(void);
uint8_t fdcan2_recover(void);

uint8_t can_send_status_query(uint8_t id);
void can_motor_status_clear(uint8_t id);
uint8_t can_motor_status_get(uint8_t id, uint8_t *status, uint32_t *tick_ms);

uint8_t Emm_V5_Read_Status(uint8_t id, uint8_t *status, uint32_t timeout_ms);
uint8_t Emm_V5_Is_Reached(uint8_t id);

#ifdef __cplusplus
}
#endif
#endif
