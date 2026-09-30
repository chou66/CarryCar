#include "edge_motion.h"
#include "chassis_motion.h"
#include "map_graph.h"
#include "navigation.h"
#include "robot_config.h"

void EdgeMotion_Init(EdgeMotion_t *em, Chassis *chassis, HMI_Navigation_t *hmi_nav)
{
    if (!em) return;
    em->chassis = chassis;
    em->hmi_nav = hmi_nav;
    em->state = EDGE_MOTION_IDLE;
    em->from_node = 0U;
    em->to_node = 0U;
}

static uint8_t node_ok(uint8_t n)
{
    return (n >= 1U && n <= NODE_NUM) ? 1U : 0U;
}

void EdgeMotion_Update(EdgeMotion_t *em, uint32_t now_ms)
{
    NavigationState_t nav_state;

    if (!em || !em->chassis || !em->hmi_nav ||
        !HMI_Navigation_IsInitialized(em->hmi_nav))
        return;

    nav_state = Navigation_GetState(&em->hmi_nav->nav);

    /* A true PAUSE must stop physical motion. Position mode cannot know the
       remaining distance after an asynchronous stop, so require replan/restart
       from the last confirmed node rather than blindly resuming. */
    if (nav_state == NAV_STATE_PAUSED)
    {
        if (em->state == EDGE_MOTION_MOVING)
        {
            chassis_motion_stop(em->chassis);
            em->state = EDGE_MOTION_PAUSE_ABORT;
        }
        return;
    }

    if (em->state == EDGE_MOTION_PAUSE_ABORT)
        return;

    if (em->state == EDGE_MOTION_MOVING)
    {
        if (chassis_motion_failed(em->chassis))
        {
            em->state = EDGE_MOTION_ERROR;
            return;
        }

        if (chassis_motion_done(em->chassis))
        {
            if (!HMI_Navigation_EdgeReached(em->hmi_nav))
            {
                em->state = EDGE_MOTION_ERROR;
                return;
            }
            HMI_Navigation_UpdateDebugDisplay(em->hmi_nav);
            em->state = EDGE_MOTION_IDLE;
        }
        return;
    }

    if (em->state != EDGE_MOTION_IDLE || nav_state != NAV_STATE_RUNNING)
        return;

    if (HMI_Navigation_GetCurrentEdge(em->hmi_nav,
                                      &em->from_node, &em->to_node))
    {
        float dx, dy;

        if (!node_ok(em->from_node) || !node_ok(em->to_node))
        {
            em->state = EDGE_MOTION_ERROR;
            return;
        }

        dx = g_nodes[em->to_node].x - g_nodes[em->from_node].x;
        dy = g_nodes[em->to_node].y - g_nodes[em->from_node].y;

        chassis_motion_translate(em->chassis, dx, dy,
                                 ROUTE_EDGE_SPEED_MPS, now_ms);
        em->state = EDGE_MOTION_MOVING;
    }
}

void EdgeMotion_EStop(EdgeMotion_t *em)
{
    if (!em || !em->chassis) return;
    chassis_motion_stop(em->chassis);
    em->state = EDGE_MOTION_ERROR;
}

EdgeMotionState_t EdgeMotion_GetState(const EdgeMotion_t *em)
{
    return em ? em->state : EDGE_MOTION_ERROR;
}
