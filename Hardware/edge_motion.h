#ifndef __EDGE_MOTION_H
#define __EDGE_MOTION_H

#include "chassis.h"
#include "hmi_navigation.h"
#include <stdint.h>

typedef enum
{
    EDGE_MOTION_IDLE = 0,
    EDGE_MOTION_MOVING,
    EDGE_MOTION_PAUSE_ABORT,
    EDGE_MOTION_ERROR
} EdgeMotionState_t;

typedef struct
{
    Chassis *chassis;
    HMI_Navigation_t *hmi_nav;
    EdgeMotionState_t state;
    uint8_t from_node;
    uint8_t to_node;
} EdgeMotion_t;

void EdgeMotion_Init(EdgeMotion_t *em, Chassis *chassis, HMI_Navigation_t *hmi_nav);
void EdgeMotion_Update(EdgeMotion_t *em, uint32_t now_ms);
void EdgeMotion_EStop(EdgeMotion_t *em);
EdgeMotionState_t EdgeMotion_GetState(const EdgeMotion_t *em);

#endif
