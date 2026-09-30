#include "robot_app.h"
#include "robot_config.h"
#include "chassis_config.h"
#include "app_uart.h"
#include "hwt101_uart.h"
#include "hmi_display.h"
#include "hmi_protocol.h"
#include "k230.h"
#include "qr_scanner.h"
#include "servo.h"
#include "manipulator.h"
#include "navigation.h"

static Chassis s_chassis;
static HMI_Navigation_t s_nav;
static EdgeMotion_t s_edge;

void RobotApp_Init(void)
{
    MecanumConfig cfg;

    chassis_config_init_mecanum(&cfg);
    chassis_init(&s_chassis, &cfg);

    AppUart_Init();
    HMI_Display_Init();
    HMI_Navigation_Init(&s_nav);
    EdgeMotion_Init(&s_edge, &s_chassis, &s_nav);

    /*
     * Start PWM at neutral only. No automatic gripper/carousel/motor movement
     * occurs at boot.
     */
    Servo_Init();
}

void RobotApp_Update(uint32_t now_ms)
{
    Hwt101 *imu = AppUart_GetImu();
    uint32_t events;

    K230_Process();
    QRScanner_Update(now_ms);

    events = HMI_Navigation_Process(&s_nav);

    if (events & (HMI_NAV_EVENT_MAP_INIT |
                  HMI_NAV_EVENT_MAP_UPDATED |
                  HMI_NAV_EVENT_REPLANNED))
    {
        HMI_Navigation_UpdateDebugDisplay(&s_nav);
    }

    /*
     * If obstacle information changes while the robot is between two confirmed
     * nodes, the navigation layer can only replan from the last confirmed node.
     * Do not keep driving a now-invalid physical edge.
     */
    if ((events & (HMI_NAV_EVENT_REPLANNED | HMI_NAV_EVENT_NO_PATH)) &&
        EdgeMotion_GetState(&s_edge) == EDGE_MOTION_MOVING)
    {
        EdgeMotion_EStop(&s_edge);
    }

    EdgeMotion_Update(&s_edge, now_ms);
    chassis_update(&s_chassis, now_ms);

    HMI_Display_Periodic(
        now_ms,
        hwt101_yaw_relative_deg(imu),
        0.0f,
        0.0f);
}

uint8_t RobotApp_GotoNode(uint8_t target_node)
{
    if (!HMI_Navigation_PlanTo(&s_nav, target_node))
        return 0U;
    HMI_Navigation_UpdateDebugDisplay(&s_nav);
    return 1U;
}

uint8_t RobotApp_GotoReturnNode(void)
{
    uint8_t n = HMI_Navigation_GetReturnNode(&s_nav);
    return n ? RobotApp_GotoNode(n) : 0U;
}

void RobotApp_EStop(void)
{
    EdgeMotion_EStop(&s_edge);
    Manipulator_StopAll();
}

Chassis *RobotApp_GetChassis(void) { return &s_chassis; }
HMI_Navigation_t *RobotApp_GetNavigation(void) { return &s_nav; }
