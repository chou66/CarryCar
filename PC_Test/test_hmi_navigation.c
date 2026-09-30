#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "hmi_protocol.h"
#include "hmi_navigation.h"
#include "hmi_display.h"

static int pass_count = 0;
static int fail_count = 0;

#define CHECK(name, cond) do { \
    if (cond) { printf("[PASS] %s\n", name); pass_count++; } \
    else { printf("[FAIL] %s\n", name); fail_count++; } \
} while (0)

/* ---------- HMI display stubs ----------
 * 本测试只验证“桥接层是否发出了正确显示数据”，
 * 不需要真实 UART / HAL。
 */
static uint8_t disp_current = 0;
static uint8_t disp_target = 0;
static uint8_t disp_path[PATH_MAX_LEN];
static uint8_t disp_path_len = 0;

void HMI_Debug_SetCurrentNode(uint8_t node) { disp_current = node; }
void HMI_Debug_SetTargetNode(uint8_t node) { disp_target = node; }
void HMI_Debug_SetPath(const uint8_t *path, uint8_t length)
{
    disp_path_len = 0;
    if (!path || !length) return;
    if (length > PATH_MAX_LEN) length = PATH_MAX_LEN;
    memcpy(disp_path, path, length);
    disp_path_len = length;
}

/* hmi_display.h 中其他函数本测试不会调用，不需要实现。 */

static void feed(const uint8_t *p, size_t n)
{
    size_t i;
    for (i = 0; i < n; ++i) HMI_Protocol_RxByte(p[i]);
}

static void send_map(uint8_t start, uint8_t ret, uint32_t mask)
{
    uint8_t f[HMI_MAP_FRAME_LEN] = {0};
    int i;
    f[0] = HMI_HEAD_1; f[1] = HMI_HEAD_2; f[2] = HMI_CMD_MAP_CONFIG;
    f[3] = start; f[4] = ret;
    for (i = 0; i < HMI_OBSTACLE_NUM; ++i)
        f[5+i] = (mask & (1UL << i)) ? 1U : 0U;
    f[26] = HMI_TAIL_1; f[27] = HMI_TAIL_2;
    feed(f, sizeof(f));
}

static void send_ctrl(uint8_t cmd)
{
    uint8_t f[HMI_CTRL_FRAME_LEN] =
        {HMI_HEAD_1, HMI_HEAD_2, HMI_CMD_RUN_CONTROL, cmd,
         HMI_TAIL_1, HMI_TAIL_2};
    feed(f, sizeof(f));
}

static int path_is(const PathResult_t *p, const uint8_t *nodes, uint8_t n)
{
    return p && p->valid && p->length == n &&
           memcmp(p->node, nodes, n) == 0;
}

int main(void)
{
    HMI_Navigation_t ctx;
    uint32_t ev;
    uint8_t from, to;
    const PathResult_t *p;

    const uint8_t normal[]  = {9,6,3};
    const uint8_t reroute[] = {9,8,5,2,3};

    printf("\n========================================\n");
    printf("  HMI <-> Navigation Bridge PC Test\n");
    printf("========================================\n\n");

    HMI_Protocol_Init();
    HMI_Navigation_Init(&ctx);

    CHECK("Not initialized before MAP_CONFIG",
          !HMI_Navigation_IsInitialized(&ctx));

    /* 第一份地图：N9 出发，N3 返回，无障碍 */
    send_map(9, 3, 0);
    ev = HMI_Navigation_Process(&ctx);

    CHECK("MAP_CONFIG initializes navigation",
          (ev & HMI_NAV_EVENT_MAP_INIT) != 0);
    CHECK("Bridge initialized", HMI_Navigation_IsInitialized(&ctx));
    CHECK("Current node from HMI = N9",
          Navigation_GetCurrentNode(&ctx.nav) == 9);
    CHECK("Return node from HMI = N3",
          HMI_Navigation_GetReturnNode(&ctx) == 3);

    CHECK("Plan N9 -> N3", HMI_Navigation_PlanTo(&ctx, 3));
    p = Navigation_GetPath(&ctx.nav);
    CHECK("Path = 9->6->3", path_is(p, normal, 3));

    CHECK("Current edge = 9->6",
          HMI_Navigation_GetCurrentEdge(&ctx, &from, &to) &&
          from == 9 && to == 6);

    HMI_Navigation_UpdateDebugDisplay(&ctx);
    CHECK("Display current node N9", disp_current == 9);
    CHECK("Display target node N3", disp_target == 3);
    CHECK("Display path 9->6->3",
          disp_path_len == 3 && memcmp(disp_path, normal, 3) == 0);

    /* HMI 暂停 / 继续 */
    send_ctrl(HMI_CTRL_PAUSE);
    ev = HMI_Navigation_Process(&ctx);
    CHECK("HMI pause -> Navigation PAUSED",
          (ev & HMI_NAV_EVENT_PAUSED) &&
          Navigation_GetState(&ctx.nav) == NAV_STATE_PAUSED);

    CHECK("Paused navigation has no executable edge",
          !HMI_Navigation_GetCurrentEdge(&ctx, &from, &to));

    send_ctrl(HMI_CTRL_RESUME);
    ev = HMI_Navigation_Process(&ctx);
    CHECK("HMI resume -> Navigation RUNNING",
          (ev & HMI_NAV_EVENT_RESUMED) &&
          Navigation_GetState(&ctx.nav) == NAV_STATE_RUNNING);

    /* z21 = bit20，封锁 N6-N9；应自动从当前确认节点 N9 重规划 */
    send_map(9, 3, (1UL << 20));
    ev = HMI_Navigation_Process(&ctx);
    p = Navigation_GetPath(&ctx.nav);

    CHECK("Obstacle update consumed",
          (ev & HMI_NAV_EVENT_MAP_UPDATED) != 0);
    CHECK("Obstacle update triggers replan",
          (ev & HMI_NAV_EVENT_REPLANNED) != 0);
    CHECK("Reroute = 9->8->5->2->3",
          path_is(p, reroute, 5));

    /* 模拟真实车辆逐边到达 */
    while (HMI_Navigation_GetCurrentEdge(&ctx, &from, &to))
    {
        printf("Edge : N%u -> N%u\n", from, to);
        CHECK("EdgeReached accepted", HMI_Navigation_EdgeReached(&ctx));
    }

    CHECK("Arrive N3",
          Navigation_GetCurrentNode(&ctx.nav) == 3);
    CHECK("Navigation DONE",
          Navigation_GetState(&ctx.nav) == NAV_STATE_DONE);

    HMI_Navigation_UpdateDebugDisplay(&ctx);
    CHECK("Display current node updated to N3", disp_current == 3);

    /*
     * 再发地图时 start_node 故意仍写 N9。
     * 桥接层不应把已经实际到达的 current_node=N3 重置成 N9。
     */
    send_map(9, 9, (1UL << 20));
    ev = HMI_Navigation_Process(&ctx);
    CHECK("Later MAP_CONFIG does not reset real current node",
          Navigation_GetCurrentNode(&ctx.nav) == 3);
    CHECK("Return node can update independently",
          HMI_Navigation_GetReturnNode(&ctx) == 9);

    printf("\n========================================\n");
    printf("PASS : %d\nFAIL : %d\nTOTAL: %d\n",
           pass_count, fail_count, pass_count + fail_count);
    printf("========================================\n");
    printf(fail_count ? "HMI-NAV BRIDGE TEST FAILED\n"
                      : "ALL HMI-NAV BRIDGE TESTS PASSED\n");

    return fail_count ? 1 : 0;
}
