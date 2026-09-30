#ifndef __ROBOT_APP_H
#define __ROBOT_APP_H

#include <stdint.h>
#include "chassis.h"
#include "hmi_navigation.h"
#include "edge_motion.h"

void RobotApp_Init(void);
void RobotApp_Update(uint32_t now_ms);

uint8_t RobotApp_GotoNode(uint8_t target_node);
uint8_t RobotApp_GotoReturnNode(void);
void RobotApp_EStop(void);

Chassis *RobotApp_GetChassis(void);
HMI_Navigation_t *RobotApp_GetNavigation(void);

#endif
