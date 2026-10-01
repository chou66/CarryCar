#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/* ---------- CAN motor IDs ---------- */
#define MOTOR_FL_ID                 1U
#define MOTOR_RL_ID                 2U
#define MOTOR_RR_ID                 3U
#define MOTOR_FR_ID                 4U
#define MOTOR_GIMBAL_ID             5U
#define MOTOR_SLIDE_ID              6U

/* ---------- Route geometry ---------- */
/* Real field measurement: adjacent navigation-node centers are 850 mm apart.
 * The yellow forbidden square is 450 x 450 mm; that is NOT the node spacing.
 */
#define ROUTE_GRID_STEP_M           0.850f
#define ROUTE_EDGE_SPEED_MPS        0.150f

/* ---------- Servo calibration ----------
 * TIM3 CH1 / PB4 = gripper
 * TIM3 CH2 / PE3 = 3-slot carousel
 *
 * These pulse values are safe starting calibration values, NOT a mechanical truth.
 * First power-up should be performed with mechanism unloaded.
 */
#define SERVO_MIN_US                500U
#define SERVO_MAX_US                2500U
#define SERVO_SAFE_US               1500U

#define GRIPPER_OPEN_US             1000U
#define GRIPPER_CLOSE_US            1374U
#define GRIPPER_MID_US              1500U

/* The old tested code supports 270/360-degree positional servo mapping.
 * This project defaults to 270 degrees because three 120-degree positions
 * fit at 0/120/240 degrees. Change this to the actual servo travel.
 */
#define CAROUSEL_TRAVEL_DEG         270U
#define CAROUSEL_SLOT0_DEG          0U
#define CAROUSEL_SLOT_STEP_DEG      120U
#define CAROUSEL_REVERSE            0U

/* 转盘舵机实测工作位置 */
#define CAROUSEL_PICKUP_US          2120U  /* 夹取物料：逆时针约104度 */
#define CAROUSEL_PLACE_US           1390U  /* 向三工位料盘放料：顺时针约20度 */
#define CAROUSEL_MID_US             1500U  /* 安全中位 */

/* ---------- Motor 5/6 mechanical calibration ----------
 * Leave 0.0 until measured. Raw pulse APIs remain usable.
 */
#define GIMBAL_PULSES_PER_DEG       0.0f
#define SLIDE_PULSES_PER_MM         0.0f

#define MANIP_DEFAULT_RPM           150U
#define MANIP_DEFAULT_ACC           20U



#endif
