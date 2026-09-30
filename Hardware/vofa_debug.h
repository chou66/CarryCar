#ifndef __VOFA_DEBUG_H
#define __VOFA_DEBUG_H
#include "main.h"
#include <stdint.h>

#define VOFA_DEBUG_PERIOD_MS 40U
#define VOFA_DEBUG_MAX_RPM       500U
#define VOFA_CHASSIS_MAX_RPM     100U

/* Formal chassis-motion validation.
 * 0.20 m/s is approximately 50 wheel RPM for the current 77 mm wheel.
 */
#define VOFA_FORMAL_MOVE_SPEED_MPS  0.20f
#define VOFA_FORMAL_MOVE_ACC        10U
#define VOFA_FORMAL_DISTANCE_M      0.45f
#define VOFA_FORMAL_ANGLE_DEG       90.0f

/*
 * HWT101 automatic heading correction after every navigation edge.
 *
 * Operation:
 *   1) Before an 85 cm edge starts, current yaw is stored as software zero.
 *   2) The chassis executes the edge.
 *   3) After a short settling time, relative yaw is checked.
 *   4) If |yaw| > deadband, ONE small opposite rotation is executed.
 *   5) Only then is Navigation_EdgeReached() called.
 *
 * This is deliberately one-shot for the first real-car test to avoid
 * oscillating back and forth around 0 degrees.
 */
#define VOFA_NAV_YAW_DEADBAND_DEG        0.80f
#define VOFA_NAV_YAW_MAX_START_DEG        8.00f
#define VOFA_NAV_YAW_CORRECT_RPM          20U
#define VOFA_NAV_YAW_CORRECT_ACC          10U
#define VOFA_NAV_YAW_SETTLE_MS            150U
#define VOFA_NAV_IMU_TIMEOUT_MS           300U
#define VOFA_NAV_YAW_CORRECT_TIMEOUT_MS  1800U

/* RX frame from VOFA, HEX mode:
 * AA 55 CMD P1 P2 P3 P4
 *
 * 01 stop all motors
 * 02 zero HWT101 yaw
 * 10 motor enable: P1=id, P2=0/1
 * 11 motor velocity: P1=id, P2=dir, P3=rpm/10, P4=acc
 * 12 stop one motor: P1=id
 * 20 gripper: P1=0 open,1 mid,2 close
 * 21 carousel: P1=slot 0..2
 * 22 raw servo us: P1=0 gripper/1 carousel, P2:P3=us
 *
 * Chassis bench commands:
 * 30 chassis enable: P1=1 enable / 0 disable
 * 31 forward:       P1=rpm/10, P2=acc
 * 32 backward:      P1=rpm/10, P2=acc
 * 33 strafe left:   P1=rpm/10, P2=acc
 * 34 strafe right:  P1=rpm/10, P2=acc
 * 35 rotate CCW:    P1=rpm/10, P2=acc
 * 36 rotate CW:     P1=rpm/10, P2=acc
 * 37 chassis stop
 *
 * Formal chassis_motion/chassis_set_position validation:
 * 40 forward 450 mm
 * 41 backward 450 mm
 * 42 strafe left 450 mm
 * 43 strafe right 450 mm
 * 44 rotate CCW 90 deg
 * 45 rotate CW 90 deg
 * 46 stop formal position motion
 *
 * Navigation + Dijkstra bench:
 * 50 init/reset logical start node: P1=start node 1..9
 * 51 plan + physically drive to target: P1=target node 1..9
 * 52 abort navigation + stop chassis
 * 53 set static obstacle mask:
 *    mask = P1 | (P2<<8) | (P3<<16) | (P4<<24)
 *
 * Map orientation:
 *   7 -- 8 -- 9
 *   |    |    |
 *   4 -- 5 -- 6
 *   |    |    |
 *   1 -- 2 -- 3
 *
 * Robot body coordinate alignment for this test:
 *   vehicle FRONT points from node 1 toward node 4 (+Y)
 *   vehicle RIGHT points from node 1 toward node 2 (+X)
 *
 * CH9 / last_result:
 * 0=no command, 1=accepted/running, 2=bad parameter or busy,
 * 3=unknown command, 4=formal position done, 5=formal position failed,
 * 6=navigation target reached, 7=navigation/no-path logic error,
 * 8=navigation physical edge motion failed,
 * 9=waiting for fresh HWT101 data,
 * 10=automatic edge-end yaw correction is running,
 * 11=yaw error too large to auto-correct safely,
 * 12=yaw correction timeout.
 */
void VofaDebug_Init(void);
void VofaDebug_FeedByte(uint8_t byte);
void VofaDebug_Update(uint32_t now_ms);
#endif
