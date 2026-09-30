#ifndef __EMM_ADAPTER_H
#define __EMM_ADAPTER_H

#include "main.h"
#include <stdbool.h>


HAL_StatusTypeDef emm_vel(
    uint8_t id,
    uint8_t dir,
    uint16_t rpm,
    uint8_t acc,
    bool sync
);


HAL_StatusTypeDef emm_pos(
    uint8_t id,
    uint8_t dir,
    uint16_t speed,
    uint8_t acc,
    uint32_t clk,
    bool raF,
    bool snF
);


HAL_StatusTypeDef emm_stop(
    uint8_t id,
    bool sync
);


HAL_StatusTypeDef emm_read_status(
    uint8_t id
);


HAL_StatusTypeDef emm_sync_start(void);


void emm_prepare_position_feedback(
    uint8_t id
);


bool emm_all_position_reached_after(
    uint8_t mask,
    uint32_t start_ms
);


bool emm_any_position_fault_after(
    uint8_t mask,
    uint32_t start_ms
);


#endif