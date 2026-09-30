#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "hmi_protocol.h"
#include "map_graph.h"
#include "path_planner.h"

static int g_pass = 0;
static int g_fail = 0;


static void pass(const char *name)
{
    printf("[PASS] %s\n", name);
    g_pass++;
}


static void fail(const char *name)
{
    printf("[FAIL] %s\n", name);
    g_fail++;
}


static void print_path(const PathResult_t *path)
{
    uint8_t i;

    if (!path->valid)
    {
        printf("NO PATH");
        return;
    }

    for (i = 0; i < path->length; i++)
    {
        printf("%u", (unsigned)path->node[i]);

        if (i + 1 < path->length)
        {
            printf("->");
        }
    }
}


static void feed_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        HMI_Protocol_RxByte(data[i]);
    }
}


static void make_map_frame(uint8_t *frame,
                           uint8_t start,
                           uint8_t target,
                           uint32_t mask)
{
    uint8_t i;

    memset(frame, 0, HMI_MAP_FRAME_LEN);

    frame[0] = HMI_HEAD_1;
    frame[1] = HMI_HEAD_2;
    frame[2] = HMI_CMD_MAP_CONFIG;

    frame[3] = start;
    frame[4] = target;

    for (i = 0; i < HMI_OBSTACLE_NUM; i++)
    {
        frame[5 + i] =
            (mask & (1UL << i)) ? 1U : 0U;
    }

    frame[26] = HMI_TAIL_1;
    frame[27] = HMI_TAIL_2;
}


static int path_equals(const PathResult_t *path,
                       const uint8_t *expected,
                       uint8_t expected_len)
{
    uint8_t i;

    if (!path->valid)
    {
        return 0;
    }

    if (path->length != expected_len)
    {
        return 0;
    }

    for (i = 0; i < expected_len; i++)
    {
        if (path->node[i] != expected[i])
        {
            return 0;
        }
    }

    return 1;
}


static void run_integration_case(const char *name,
                                 uint8_t start,
                                 uint8_t target,
                                 uint32_t mask,
                                 const uint8_t *expected_path,
                                 uint8_t expected_len,
                                 int expect_valid)
{
    uint8_t frame[HMI_MAP_FRAME_LEN];
    HMI_MapConfig_t map;
    PathResult_t path;
    int ok = 1;

    HMI_Protocol_Init();

    make_map_frame(
        frame,
        start,
        target,
        mask
    );

    /*
     * 模拟 UART 一字节一字节收到淘晶驰数据
     */
    feed_bytes(frame, sizeof(frame));

    /*
     * 模拟主循环取得最新地图
     */
    if (!HMI_MapConfig_TakeUpdatedSnapshot(&map))
    {
        printf("    ERROR: no map snapshot\n");
        fail(name);
        return;
    }

    if (map.start_node != start)
    {
        printf("    ERROR: start mismatch\n");
        ok = 0;
    }

    if (map.return_node != target)
    {
        printf("    ERROR: target mismatch\n");
        ok = 0;
    }

    if (map.obstacle_mask != mask)
    {
        printf("    ERROR: mask mismatch\n");
        printf("    expected: 0x%08lX\n",
               (unsigned long)mask);
        printf("    actual  : 0x%08lX\n",
               (unsigned long)map.obstacle_mask);
        ok = 0;
    }

    /*
     * 建图
     */
    MapGraph_Build(map.obstacle_mask);

    /*
     * 寻路
     */
    path = PathPlanner_Dijkstra(
        map.start_node,
        map.return_node
    );

    printf("    start : %u\n",
           (unsigned)map.start_node);

    printf("    target: %u\n",
           (unsigned)map.return_node);

    printf("    mask  : 0x%08lX\n",
           (unsigned long)map.obstacle_mask);

    printf("    path  : ");
    print_path(&path);
    printf("\n");

    if (expect_valid)
    {
        if (!path.valid)
        {
            printf("    ERROR: expected valid path\n");
            ok = 0;
        }
        else if (expected_path != NULL)
        {
            if (!path_equals(
                    &path,
                    expected_path,
                    expected_len))
            {
                printf("    ERROR: path mismatch\n");
                ok = 0;
            }
        }
    }
    else
    {
        if (path.valid)
        {
            printf("    ERROR: expected NO PATH\n");
            ok = 0;
        }
    }

    if (ok)
    {
        pass(name);
    }
    else
    {
        fail(name);
    }
}


static void test_hmi_example(void)
{
    /*
     * 协议文件中的示例：
     *
     * 启停区1 -> 启停区2
     * start = N9
     * return = N3
     *
     * z10 = O10 = bit9
     * z18 = O18 = bit17
     *
     * 当前最短路径：
     * 9->6->3
     */
    static const uint8_t expected[] =
    {
        9, 6, 3
    };

    run_integration_case(
        "HMI example z10 + z18",
        9,
        3,
        (1UL << 9) |
        (1UL << 17),
        expected,
        sizeof(expected),
        1
    );
}


static void test_block_z21(void)
{
    /*
     * z21 = N6-N9
     *
     * 9不能直接去6，
     * 当前算法会绕：
     *
     * 9->8->5->2->3
     */
    static const uint8_t expected[] =
    {
        9, 8, 5, 2, 3
    };

    run_integration_case(
        "Block z21",
        9,
        3,
        (1UL << 20),
        expected,
        sizeof(expected),
        1
    );
}


static void test_no_path(void)
{
    /*
     * z15 = N8-N9
     * z21 = N6-N9
     *
     * 把N9的两条出口全部堵死
     */
    run_integration_case(
        "Isolate N9",
        9,
        3,
        (1UL << 14) |
        (1UL << 20),
        NULL,
        0,
        0
    );
}


static void test_reverse(void)
{
    static const uint8_t expected[] =
    {
        3, 6, 9
    };

    run_integration_case(
        "Reverse N3 -> N9",
        3,
        9,
        0,
        expected,
        sizeof(expected),
        1
    );
}


static void test_node_obstacle(void)
{
    /*
     * O5 = node N5
     *
     * 9->6->3 不经过N5
     */
    static const uint8_t expected[] =
    {
        9, 6, 3
    };

    run_integration_case(
        "Block node N5",
        9,
        3,
        (1UL << 4),
        expected,
        sizeof(expected),
        1
    );
}


int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  HMI -> Map -> Dijkstra Integration\n");
    printf("========================================\n\n");

    test_hmi_example();
    test_block_z21();
    test_no_path();
    test_reverse();
    test_node_obstacle();

    printf("\n");
    printf("========================================\n");
    printf("PASS : %d\n", g_pass);
    printf("FAIL : %d\n", g_fail);
    printf("TOTAL: %d\n", g_pass + g_fail);
    printf("========================================\n");

    if (g_fail == 0)
    {
        printf("ALL INTEGRATION TESTS PASSED\n");
        return 0;
    }

    printf("INTEGRATION TEST FAILED\n");
    return 1;
}
