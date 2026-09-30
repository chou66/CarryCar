#ifndef __HMI_NAVIGATION_H
#define __HMI_NAVIGATION_H

#include <stdint.h>
#include "hmi_protocol.h"
#include "navigation.h"

/*
 * HMI <-> Navigation 桥接层
 *
 * 职责：
 * 1. 消费 hmi_protocol.c 已解析好的地图/暂停/继续事件；
 * 2. 将地图配置同步到 Navigation；
 * 3. 在导航运行中地图改变时，从“最后确认到达节点”重新规划；
 * 4. 将 Navigation 的节点/目标/路径同步到 page5 调试页。
 *
 * 不负责：
 * - UART 字节接收（hmi_protocol.c）
 * - Dijkstra（navigation/path_planner）
 * - 真实车辆运动（未来 edge_motion）
 * - 比赛阶段调度（未来 mission）
 */

typedef struct
{
    Navigation_t nav;

    /* HMI 最近一次合法地图配置。 */
    HMI_MapConfig_t map;

    /* 是否已经用第一份 HMI 地图配置初始化 Navigation。 */
    uint8_t initialized;

    /* HMI 配置中的最终返回节点，仅保存，不自动导航过去。 */
    uint8_t return_node;

} HMI_Navigation_t;

/**
 * @brief 初始化 HMI-Navigation 桥接对象。
 *
 * @param ctx 桥接对象。
 *
 * @note
 * 系统启动时调用一次。此函数不会凭空创建导航起点；
 * 必须等淘晶驰发送第一份合法 MAP_CONFIG 后，
 * HMI_Navigation_Process() 才会用 start_node 初始化 Navigation。
 */
void HMI_Navigation_Init(HMI_Navigation_t *ctx);

/**
 * @brief 处理 HMI 已解析事件，并同步到 Navigation。
 *
 * @param ctx 桥接对象。
 *
 * @return 本轮发生的事件位，可按 HMI_NAV_EVENT_xxx 判断。
 *
 * @note 主循环中反复调用即可。
 *
 * 第一份地图：
 *   HMI start_node + obstacle_mask -> Navigation_Init()
 *
 * 后续地图更新：
 *   新 obstacle_mask -> Navigation_SetObstacleMask()
 *   若导航正在 RUNNING/PAUSED 且障碍发生变化 -> Navigation_Replan()
 *
 * 暂停/继续：
 *   HMI pause  -> Navigation_Pause()
 *   HMI resume -> Navigation_Resume()
 *
 * @warning
 * 地图更新不会把 Navigation 当前节点强行改回 HMI 的 start_node。
 * 实车运行后，当前位置必须以 Navigation 最后确认到达的节点为准。
 */
uint32_t HMI_Navigation_Process(HMI_Navigation_t *ctx);

/**
 * @brief 规划前往某节点。
 *
 * @param ctx 桥接对象。
 * @param target_node 目标节点 1~9。
 * @return 1 成功；0 尚未初始化、无路或参数非法。
 *
 * @note
 * 比赛任务层以后主要调用这个接口，不需要直接调用 Dijkstra。
 */
uint8_t HMI_Navigation_PlanTo(HMI_Navigation_t *ctx, uint8_t target_node);

/**
 * @brief 获取当前待执行地图边。
 *
 * @return 1 有边，例如 from=9,to=6；0 当前没有待执行边。
 *
 * @note 未来 edge_motion 层消费此接口。
 */
uint8_t HMI_Navigation_GetCurrentEdge(const HMI_Navigation_t *ctx,
                                      uint8_t *from_node,
                                      uint8_t *to_node);

/**
 * @brief 通知：当前地图边已经被真实车辆完成。
 *
 * @warning
 * 实车只有确认到达下一节点后才能调用。
 * PC Test 可直接调用来模拟“车辆已经走完这一边”。
 */
uint8_t HMI_Navigation_EdgeReached(HMI_Navigation_t *ctx);

/**
 * @brief 将 Navigation 当前节点、目标节点和路径发送到 page5。
 *
 * @note
 * 建议在首次规划、重规划、EdgeReached 后调用；
 * 不必每个主循环都发送，避免占用串口。
 */
void HMI_Navigation_UpdateDebugDisplay(const HMI_Navigation_t *ctx);

/** @brief 获取 HMI 配置中的最终返回节点。 */
uint8_t HMI_Navigation_GetReturnNode(const HMI_Navigation_t *ctx);

/** @brief 是否已收到第一份合法地图并完成 Navigation 初始化。 */
uint8_t HMI_Navigation_IsInitialized(const HMI_Navigation_t *ctx);


/* HMI_Navigation_Process() 事件位 */
#define HMI_NAV_EVENT_NONE          0x00000000UL
#define HMI_NAV_EVENT_MAP_INIT      0x00000001UL
#define HMI_NAV_EVENT_MAP_UPDATED   0x00000002UL
#define HMI_NAV_EVENT_REPLANNED     0x00000004UL
#define HMI_NAV_EVENT_NO_PATH       0x00000008UL
#define HMI_NAV_EVENT_PAUSED        0x00000010UL
#define HMI_NAV_EVENT_RESUMED       0x00000020UL

#endif
