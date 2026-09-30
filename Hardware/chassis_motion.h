#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "chassis.h"

/*
 * 比赛任务层的基础动作接口。
 * 坐标约定和 mecanum_kinematics 保持一致：
 *   +X = 右，-X = 左
 *   +Y = 前，-Y = 后
 *   +theta = 逆时针，-theta = 顺时针
 *
 * 未来节点寻路请优先调用 signed core 接口：
 *   chassis_motion_translate(dx, dy, ...)
 * 而不是先判断正负再选择 forward/backward/left/right。
 */

void chassis_motion_move(Chassis *chassis,
                         float dx_m, float dy_m, float dtheta_rad,
                         float speed_mps, uint32_t now_ms);

void chassis_motion_translate(Chassis *chassis,
                              float dx_m, float dy_m,
                              float speed_mps, uint32_t now_ms);

void chassis_motion_move_x(Chassis *chassis,
                           float dx_m, float speed_mps, uint32_t now_ms);

void chassis_motion_move_y(Chassis *chassis,
                           float dy_m, float speed_mps, uint32_t now_ms);

void chassis_motion_rotate(Chassis *chassis,
                           float dtheta_rad, float speed_mps, uint32_t now_ms);

void chassis_motion_rotate_deg(Chassis *chassis,
                               float angle_deg, float speed_mps, uint32_t now_ms);

/* 人工调车快捷函数：内部仍回到同一个 signed core，不会绕开校准。 */
void chassis_motion_forward(Chassis *chassis, float distance_m,
                            float speed_mps, uint32_t now_ms);
void chassis_motion_backward(Chassis *chassis, float distance_m,
                             float speed_mps, uint32_t now_ms);
void chassis_motion_strafe_right(Chassis *chassis, float distance_m,
                                 float speed_mps, uint32_t now_ms);
void chassis_motion_strafe_left(Chassis *chassis, float distance_m,
                                float speed_mps, uint32_t now_ms);
void chassis_motion_rotate_ccw_deg(Chassis *chassis, float angle_deg,
                                   float speed_mps, uint32_t now_ms);
void chassis_motion_rotate_cw_deg(Chassis *chassis, float angle_deg,
                                  float speed_mps, uint32_t now_ms);

bool chassis_motion_busy(const Chassis *chassis);
bool chassis_motion_done(const Chassis *chassis);
bool chassis_motion_failed(const Chassis *chassis);
void chassis_motion_stop(Chassis *chassis);
