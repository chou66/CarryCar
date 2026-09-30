#include "hmi_navigation.h"
#include "hmi_display.h"
#include <string.h>

static uint8_t nav_active(NavigationState_t s)
{
    return (s == NAV_STATE_RUNNING || s == NAV_STATE_PAUSED) ? 1U : 0U;
}

void HMI_Navigation_Init(HMI_Navigation_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

uint32_t HMI_Navigation_Process(HMI_Navigation_t *ctx)
{
    uint32_t events = HMI_NAV_EVENT_NONE;
    HMI_MapConfig_t incoming;

    if (!ctx) return HMI_NAV_EVENT_NONE;

    if (HMI_MapConfig_TakeUpdatedSnapshot(&incoming))
    {
        uint32_t old_mask = ctx->map.obstacle_mask;
        uint8_t had_init = ctx->initialized;
        NavigationState_t old_state = had_init ?
            Navigation_GetState(&ctx->nav) : NAV_STATE_IDLE;

        ctx->map = incoming;
        ctx->return_node = incoming.return_node;

        if (!had_init)
        {
            if (Navigation_Init(&ctx->nav,
                                incoming.start_node,
                                incoming.obstacle_mask))
            {
                ctx->initialized = 1U;
                events |= HMI_NAV_EVENT_MAP_INIT;
            }
        }
        else
        {
            /*
             * 运行后的 HMI start_node 不覆盖真实 current_node。
             * start_node 只用于第一份地图初始化。
             */
            Navigation_SetObstacleMask(&ctx->nav, incoming.obstacle_mask);
            events |= HMI_NAV_EVENT_MAP_UPDATED;

            /*
             * 只有障碍真的变化，并且当前有正在执行/暂停的目标时才重规划。
             * PAUSED 时 Navigation_Replan() 会得到 RUNNING，因此下面恢复暂停状态。
             */
            if ((old_mask != incoming.obstacle_mask) && nav_active(old_state))
            {
                if (Navigation_Replan(&ctx->nav))
                {
                    events |= HMI_NAV_EVENT_REPLANNED;

                    if (old_state == NAV_STATE_PAUSED)
                    {
                        (void)Navigation_Pause(&ctx->nav);
                    }
                }
                else
                {
                    events |= HMI_NAV_EVENT_NO_PATH;
                }
            }
        }
    }

    if (HMI_Control_GetPauseRequest())
    {
        HMI_Control_ClearPauseRequest();

        if (ctx->initialized && Navigation_Pause(&ctx->nav))
            events |= HMI_NAV_EVENT_PAUSED;
    }

    if (HMI_Control_GetResumeRequest())
    {
        HMI_Control_ClearResumeRequest();

        if (ctx->initialized && Navigation_Resume(&ctx->nav))
            events |= HMI_NAV_EVENT_RESUMED;
    }

    return events;
}

uint8_t HMI_Navigation_PlanTo(HMI_Navigation_t *ctx, uint8_t target_node)
{
    if (!ctx || !ctx->initialized) return 0U;
    return Navigation_PlanTo(&ctx->nav, target_node);
}

uint8_t HMI_Navigation_GetCurrentEdge(const HMI_Navigation_t *ctx,
                                      uint8_t *from_node,
                                      uint8_t *to_node)
{
    if (!ctx || !ctx->initialized) return 0U;
    return Navigation_GetCurrentEdge(&ctx->nav, from_node, to_node);
}

uint8_t HMI_Navigation_EdgeReached(HMI_Navigation_t *ctx)
{
    if (!ctx || !ctx->initialized) return 0U;
    return Navigation_EdgeReached(&ctx->nav);
}

void HMI_Navigation_UpdateDebugDisplay(const HMI_Navigation_t *ctx)
{
    const PathResult_t *path;
    uint8_t target;

    if (!ctx || !ctx->initialized) return;

    HMI_Debug_SetCurrentNode(Navigation_GetCurrentNode(&ctx->nav));

    target = Navigation_GetTargetNode(&ctx->nav);
    if (target != 0U)
        HMI_Debug_SetTargetNode(target);

    path = Navigation_GetPath(&ctx->nav);

    if (path && path->valid && path->length > 0U)
        HMI_Debug_SetPath(path->node, path->length);
    else
        HMI_Debug_SetPath(NULL, 0U);
}

uint8_t HMI_Navigation_GetReturnNode(const HMI_Navigation_t *ctx)
{
    return ctx ? ctx->return_node : 0U;
}

uint8_t HMI_Navigation_IsInitialized(const HMI_Navigation_t *ctx)
{
    return (ctx && ctx->initialized) ? 1U : 0U;
}
