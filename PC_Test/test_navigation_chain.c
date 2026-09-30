#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "hmi_protocol.h"
#include "map_graph.h"
#include "path_planner.h"
#include "path_executor.h"


static int g_pass = 0;
static int g_fail = 0;


/* =========================================================
 * Test helpers
 * ========================================================= */

static void test_pass(const char *name)
{
    printf("[PASS] %s\n", name);
    g_pass++;
}


static void test_fail(const char *name)
{
    printf("[FAIL] %s\n", name);
    g_fail++;
}


static void check(int condition, const char *name)
{
    if (condition)
    {
        test_pass(name);
    }
    else
    {
        test_fail(name);
    }
}


static void print_path(const PathResult_t *path)
{
    uint8_t i;

    if ((path == NULL) || (path->valid == 0U))
    {
        printf("NO PATH");
        return;
    }

    for (i = 0U; i < path->length; i++)
    {
        printf("%u", path->node[i]);

        if (i + 1U < path->length)
        {
            printf("->");
        }
    }
}


/* =========================================================
 * Build a real HMI MAP_CONFIG frame
 *
 * AA 55 01 SS RR O1 ... O21 55 AA
 * ========================================================= */

static void make_map_frame(uint8_t *frame,
                           uint8_t start,
                           uint8_t return_node,
                           uint32_t obstacle_mask)
{
    uint8_t i;

    memset(frame, 0, HMI_MAP_FRAME_LEN);

    frame[0] = HMI_HEAD_1;
    frame[1] = HMI_HEAD_2;
    frame[2] = HMI_CMD_MAP_CONFIG;

    frame[3] = start;
    frame[4] = return_node;

    /*
     * obstacle[0] = z01
     * ...
     * obstacle[20] = z21
     */
    for (i = 0U; i < HMI_OBSTACLE_NUM; i++)
    {
        frame[5U + i] =
            (uint8_t)((obstacle_mask >> i) & 0x01U);
    }

    frame[26] = HMI_TAIL_1;
    frame[27] = HMI_TAIL_2;
}


/* =========================================================
 * Feed one complete frame byte-by-byte.
 *
 * This deliberately simulates UART RX behavior rather than
 * directly modifying g_hmi_map.
 * ========================================================= */

static void feed_hmi_frame(const uint8_t *frame,
                           uint8_t length)
{
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        HMI_Protocol_RxByte(frame[i]);
    }
}


/* =========================================================
 * Execute one PathResult completely.
 *
 * No real chassis exists on PC.
 *
 * Therefore:
 *
 * current -> next
 *      ↓
 * pretend vehicle reached next
 *      ↓
 * PathExecutor_EdgeReached()
 * ========================================================= */

static uint8_t simulate_executor(PathExecutor_t *exec,
                                 const PathResult_t *path)
{
    uint8_t current;
    uint8_t next;

    if (!PathExecutor_Start(exec, path))
    {
        printf("Executor refused path\n");
        return 0U;
    }

    /*
     * start == target:
     * Executor should already be DONE.
     */
    if (PathExecutor_IsDone(exec))
    {
        printf("Exec : already at N%u\n",
               PathExecutor_GetCurrentNode(exec));

        return 1U;
    }

    while (!PathExecutor_IsDone(exec))
    {
        current = PathExecutor_GetCurrentNode(exec);
        next    = PathExecutor_GetNextNode(exec);

        if ((current == 0U) || (next == 0U))
        {
            printf("Exec : invalid edge %u -> %u\n",
                   current,
                   next);

            return 0U;
        }

        printf("Exec : N%u -> N%u\n",
               current,
               next);

        /*
         * PC simulation:
         *
         * pretend the physical vehicle has actually
         * reached next_node.
         */
        if (!PathExecutor_EdgeReached(exec))
        {
            printf("Executor failed to advance\n");
            return 0U;
        }
    }

    return 1U;
}


/* =========================================================
 * Plan + execute ONE navigation stage.
 *
 * Important:
 *
 * start is always supplied by the previous Executor result.
 * ========================================================= */

static uint8_t run_stage(PathExecutor_t *exec,
                         uint8_t stage,
                         uint8_t start,
                         uint8_t target)
{
    PathResult_t path;
    uint8_t final_node;

    printf("\n");
    printf("========== STAGE %u ==========\n", stage);

    printf("Current : N%u\n", start);
    printf("Target  : N%u\n", target);

    path = PathPlanner_Dijkstra(start, target);

    printf("Plan    : ");
    print_path(&path);
    printf("\n");

    if (!path.valid)
    {
        printf("[STAGE FAILED] NO PATH\n");
        return 0U;
    }

    /*
     * Planner result itself must start/end correctly.
     */
    if ((path.length == 0U) ||
        (path.node[0] != start) ||
        (path.node[path.length - 1U] != target))
    {
        printf("[STAGE FAILED] Bad planner result\n");
        return 0U;
    }

    if (!simulate_executor(exec, &path))
    {
        printf("[STAGE FAILED] Executor error\n");
        return 0U;
    }

    final_node = PathExecutor_GetCurrentNode(exec);

    printf("Arrived : N%u\n", final_node);

    if (final_node != target)
    {
        printf("[STAGE FAILED] Wrong final node\n");
        return 0U;
    }

    printf("[STAGE PASS]\n");

    return 1U;
}


/* =========================================================
 * TEST 1
 *
 * Real HMI frame
 *      ↓
 * snapshot
 *      ↓
 * MapGraph
 *      ↓
 * Dijkstra
 *      ↓
 * Executor
 * ========================================================= */

static void test_hmi_to_executor(void)
{
    uint8_t frame[HMI_MAP_FRAME_LEN];

    HMI_MapConfig_t map;
    PathResult_t path;
    PathExecutor_t exec;

    printf("\n");
    printf("========================================\n");
    printf(" TEST 1: HMI -> Map -> Planner -> Exec\n");
    printf("========================================\n");

    HMI_Protocol_Init();

    /*
     * Empty map:
     *
     * start  = N9
     * return = N3
     */
    make_map_frame(frame,
                   9U,
                   3U,
                   0U);

    feed_hmi_frame(frame,
                   HMI_MAP_FRAME_LEN);

    check(HMI_MapConfig_IsValid(),
          "HMI map valid");

    check(HMI_MapConfig_IsUpdated(),
          "HMI map updated");

    memset(&map, 0, sizeof(map));

    check(HMI_MapConfig_TakeUpdatedSnapshot(&map),
          "Take HMI snapshot");

    check(map.start_node == 9U,
          "Snapshot start N9");

    check(map.return_node == 3U,
          "Snapshot return N3");

    check(map.obstacle_mask == 0U,
          "Snapshot empty obstacle mask");

    MapGraph_Build(map.obstacle_mask);

    path =
        PathPlanner_Dijkstra(map.start_node,
                             map.return_node);

    printf("Planner : ");
    print_path(&path);
    printf("\n");

    check(path.valid,
          "Dijkstra path exists");

    PathExecutor_Init(&exec);

    check(simulate_executor(&exec, &path),
          "Executor consumes Dijkstra path");

    check(PathExecutor_GetCurrentNode(&exec) ==
              map.return_node,
          "Executor arrives N3");

    check(PathExecutor_IsDone(&exec),
          "Executor DONE");
}


/* =========================================================
 * TEST 2
 *
 * Continuous multi-stage navigation.
 *
 * These targets DO NOT represent actual competition
 * functional-area mapping.
 *
 * They only simulate:
 *
 * Start -> A -> B -> C -> ... -> End
 *
 * Critical rule:
 *
 * next stage start =
 * PathExecutor_GetCurrentNode()
 * ========================================================= */

static void test_multi_stage_navigation(void)
{
    PathExecutor_t exec;

    /*
     * Simulated mission checkpoints.
     *
     * N9 -> N8 -> N7 -> N2 -> N6
     *    -> N8 -> N2 -> N6 -> N3
     *
     * They are NOT yet QR/raw/rough/fine mappings.
     */
    static const uint8_t target[] =
    {
        8U,
        7U,
        2U,
        6U,
        8U,
        2U,
        6U,
        3U
    };

    uint8_t current;
    uint8_t i;
    uint8_t mission_ok = 1U;

    printf("\n");
    printf("========================================\n");
    printf(" TEST 2: Multi-stage Navigation\n");
    printf("========================================\n");

    /*
     * No obstacles for this test.
     */
    MapGraph_Build(0U);

    PathExecutor_Init(&exec);

    current = 9U;

    printf("Mission start : N%u\n", current);

    for (i = 0U;
         i < (sizeof(target) / sizeof(target[0]));
         i++)
    {
        if (!run_stage(&exec,
                       (uint8_t)(i + 1U),
                       current,
                       target[i]))
        {
            mission_ok = 0U;
            break;
        }

        /*
         * THIS is the important part.
         *
         * Do NOT write:
         *
         * current = target[i];
         *
         * Instead use the Executor's actual final state.
         */
        current =
            PathExecutor_GetCurrentNode(&exec);

        if (current != target[i])
        {
            mission_ok = 0U;
            break;
        }
    }

    check(mission_ok,
          "All navigation stages completed");

    check(current == 3U,
          "Mission final node N3");
}


/* =========================================================
 * TEST 3
 *
 * start == target
 *
 * This is important for future competition logic:
 *
 * If vehicle is already at the requested navigation node,
 * that stage must immediately finish rather than fail.
 * ========================================================= */

static void test_same_node_stage(void)
{
    PathExecutor_t exec;

    printf("\n");
    printf("========================================\n");
    printf(" TEST 3: Start == Target\n");
    printf("========================================\n");

    MapGraph_Build(0U);

    PathExecutor_Init(&exec);

    check(run_stage(&exec,
                    1U,
                    5U,
                    5U),
          "N5 -> N5 stage");

    check(PathExecutor_GetCurrentNode(&exec) == 5U,
          "Remain at N5");

    check(PathExecutor_IsDone(&exec),
          "Same-node stage DONE");
}


/* =========================================================
 * TEST 4
 *
 * HMI obstacle map -> Dijkstra -> Executor
 *
 * We use the previously verified z21 case.
 *
 * Previous PC tests showed:
 *
 * Empty:
 * 9 -> 6 -> 3
 *
 * z21 blocked:
 * 9 -> 8 -> 5 -> 2 -> 3
 *
 * This test checks the complete chain.
 * ========================================================= */

static void test_obstacle_reroute(void)
{
    uint8_t frame[HMI_MAP_FRAME_LEN];

    HMI_MapConfig_t map;
    PathResult_t path;
    PathExecutor_t exec;

    /*
     * z21 = bit20
     */
    const uint32_t z21_mask =
        (1UL << 20);

    printf("\n");
    printf("========================================\n");
    printf(" TEST 4: HMI Obstacle Reroute\n");
    printf("========================================\n");

    HMI_Protocol_Init();

    make_map_frame(frame,
                   9U,
                   3U,
                   z21_mask);

    feed_hmi_frame(frame,
                   HMI_MAP_FRAME_LEN);

    memset(&map, 0, sizeof(map));

    check(HMI_MapConfig_TakeUpdatedSnapshot(&map),
          "Take obstacle map snapshot");

    check(map.obstacle_mask == z21_mask,
          "z21 obstacle received");

    MapGraph_Build(map.obstacle_mask);

    path =
        PathPlanner_Dijkstra(map.start_node,
                             map.return_node);

    printf("Reroute : ");
    print_path(&path);
    printf("\n");

    check(path.valid,
          "Reroute path exists");

    /*
     * Based on the map tests already passed:
     *
     * z21 blocks N6-N9.
     *
     * Expected shortest path:
     *
     * 9 -> 8 -> 5 -> 2 -> 3
     */
    check(path.length == 5U &&
          path.node[0] == 9U &&
          path.node[1] == 8U &&
          path.node[2] == 5U &&
          path.node[3] == 2U &&
          path.node[4] == 3U,
          "z21 reroute is 9->8->5->2->3");

    PathExecutor_Init(&exec);

    check(simulate_executor(&exec, &path),
          "Executor consumes reroute");

    check(PathExecutor_GetCurrentNode(&exec) == 3U,
          "Reroute arrives N3");
}


/* =========================================================
 * TEST 5
 *
 * Mission abort when one stage has NO PATH.
 *
 * We use the previously verified mask that isolates N9:
 *
 * 0x00104000
 *
 * Important behavior:
 *
 * - planner says NO PATH
 * - executor must not falsely advance
 * - later mission stages must not run
 * ========================================================= */

static void test_no_path_abort(void)
{
    PathExecutor_t exec;

    PathResult_t path;

    uint8_t current = 9U;
    uint8_t later_stage_executed = 0U;

    const uint32_t isolate_n9_mask =
        0x00104000UL;

    printf("\n");
    printf("========================================\n");
    printf(" TEST 5: NO PATH Mission Abort\n");
    printf("========================================\n");

    MapGraph_Build(isolate_n9_mask);

    PathExecutor_Init(&exec);

    path =
        PathPlanner_Dijkstra(current, 3U);

    printf("Planner : ");
    print_path(&path);
    printf("\n");

    check(path.valid == 0U,
          "Isolated N9 produces NO PATH");

    /*
     * Critical:
     * don't start executor with an invalid route.
     */
    if (path.valid)
    {
        PathExecutor_Start(&exec, &path);

        later_stage_executed = 1U;
    }

    check(later_stage_executed == 0U,
          "Mission stops on failed stage");

    check(current == 9U,
          "Mission current node remains N9");
}


/* =========================================================
 * MAIN
 * ========================================================= */

int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  27_carrycar Navigation Chain PC Test\n");
    printf("========================================\n");

    test_hmi_to_executor();

    test_multi_stage_navigation();

    test_same_node_stage();

    test_obstacle_reroute();

    test_no_path_abort();

    printf("\n");
    printf("========================================\n");
    printf("PASS : %d\n", g_pass);
    printf("FAIL : %d\n", g_fail);
    printf("TOTAL: %d\n", g_pass + g_fail);
    printf("========================================\n");

    if (g_fail == 0)
    {
        printf("ALL NAVIGATION CHAIN TESTS PASSED\n");
        return 0;
    }

    printf("NAVIGATION CHAIN TEST FAILED\n");

    return 1;
}
