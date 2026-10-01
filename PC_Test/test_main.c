#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "map_graph.h"
#include "path_planner.h"

#define TEST_NODE_COUNT 9

static int g_pass = 0;
static int g_fail = 0;

/* O10 ~ O21 对应的边 */
typedef struct
{
    uint8_t a;
    uint8_t b;
} TestEdge_t;

static const TestEdge_t g_test_edges[12] =
{
    {1, 2}, /* O10 */
    {2, 3}, /* O11 */
    {4, 5}, /* O12 */
    {5, 6}, /* O13 */
    {7, 8}, /* O14 */
    {8, 9}, /* O15 */

    {1, 4}, /* O16 */
    {4, 7}, /* O17 */
    {2, 5}, /* O18 */
    {5, 8}, /* O19 */
    {3, 6}, /* O20 */
    {6, 9}  /* O21 */
};


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


static int is_node_blocked(uint32_t mask, uint8_t node)
{
    if (node < 1 || node > 9)
    {
        return 1;
    }

    return (mask & (1UL << (node - 1))) != 0;
}


static int is_edge_blocked(uint32_t mask,
                           uint8_t a,
                           uint8_t b)
{
    uint8_t i;

    for (i = 0; i < 12; i++)
    {
        uint8_t ea = g_test_edges[i].a;
        uint8_t eb = g_test_edges[i].b;

        if (((a == ea) && (b == eb)) ||
            ((a == eb) && (b == ea)))
        {
            return (mask & (1UL << (9 + i))) != 0;
        }
    }

    return 0;
}


static int validate_path(const PathResult_t *path,
                         uint8_t start,
                         uint8_t target,
                         uint32_t mask)
{
    uint8_t i;

    if (!path->valid)
    {
        return 1;
    }

    if (path->length == 0)
    {
        printf("    ERROR: valid path but length == 0\n");
        return 0;
    }

    if (path->node[0] != start)
    {
        printf("    ERROR: path does not start at start node\n");
        return 0;
    }

    if (path->node[path->length - 1] != target)
    {
        printf("    ERROR: path does not end at target node\n");
        return 0;
    }

    /* 检查是否经过被封锁节点 */
    for (i = 0; i < path->length; i++)
    {
        if (is_node_blocked(mask, path->node[i]))
        {
            printf("    ERROR: path crosses blocked node N%u\n",
                   (unsigned)path->node[i]);
            return 0;
        }
    }

    /* 检查每相邻两个节点 */
    for (i = 0; i + 1 < path->length; i++)
    {
        uint8_t a = path->node[i];
        uint8_t b = path->node[i + 1];

        if (!MapGraph_IsConnected(a, b))
        {
            printf("    ERROR: invalid edge %u->%u\n",
                   (unsigned)a,
                   (unsigned)b);
            return 0;
        }

        if (is_edge_blocked(mask, a, b))
        {
            printf("    ERROR: path crosses blocked edge %u-%u\n",
                   (unsigned)a,
                   (unsigned)b);
            return 0;
        }
    }

    /*
     * 边权不再统一（按实测坐标距离），改为沿路径累加
     * g_graph 的真实边权，验证 total_cost 求和正确。
     */
    {
        uint16_t expected_cost = 0;

        for (i = 0; i + 1 < path->length; i++)
        {
            expected_cost += g_graph[path->node[i]][path->node[i + 1]];
        }

        if (path->total_cost != expected_cost)
        {
            printf("    ERROR: cost mismatch, cost=%u expected=%u\n",
                   (unsigned)path->total_cost,
                   (unsigned)expected_cost);
            return 0;
        }
    }

    return 1;
}


static void report_result(const char *name,
                          int pass,
                          const PathResult_t *path)
{
    if (pass)
    {
        printf("[PASS] %-36s  ", name);

        if (path != NULL)
        {
            print_path(path);
        }

        printf("\n");
        g_pass++;
    }
    else
    {
        printf("[FAIL] %-36s  ", name);

        if (path != NULL)
        {
            print_path(path);
        }

        printf("\n");
        g_fail++;
    }
}


static void run_basic_test(const char *name,
                           uint8_t start,
                           uint8_t target,
                           uint32_t mask,
                           int expect_valid)
{
    PathResult_t path;
    int ok;

    MapGraph_Build(mask);
    path = PathPlanner_Dijkstra(start, target);

    ok = 1;

    if ((path.valid != 0) != (expect_valid != 0))
    {
        printf("    ERROR: valid=%u expected=%u\n",
               (unsigned)path.valid,
               (unsigned)expect_valid);
        ok = 0;
    }

    if (path.valid)
    {
        if (!validate_path(&path, start, target, mask))
        {
            ok = 0;
        }
    }

    report_result(name, ok, &path);
}


static void test_basic_cases(void)
{
    printf("\n=== Basic tests ===\n");

    run_basic_test(
        "Empty map N9 -> N3",
        9,
        3,
        0,
        1
    );

    run_basic_test(
        "Empty map N3 -> N9",
        3,
        9,
        0,
        1
    );

    run_basic_test(
        "Block O21 / z21 / N6-N9",
        9,
        3,
        (1UL << 20),
        1
    );

    run_basic_test(
        "Isolate N9 with O15 + O21",
        9,
        3,
        (1UL << 14) |
        (1UL << 20),
        0
    );

    run_basic_test(
        "HMI example O10 + O18",
        9,
        3,
        (1UL << 9) |
        (1UL << 17),
        1
    );

    run_basic_test(
        "Block node N5",
        9,
        3,
        (1UL << 4),
        1
    );
}


static void test_all_node_obstacles(void)
{
    uint8_t node;

    printf("\n=== O1 ~ O9 node obstacle tests ===\n");

    for (node = 1; node <= 9; node++)
    {
        char name[64];
        uint32_t mask = (1UL << (node - 1));
        PathResult_t path;
        int ok = 1;

        snprintf(name,
                 sizeof(name),
                 "O%u blocks node N%u",
                 (unsigned)node,
                 (unsigned)node);

        MapGraph_Build(mask);
        path = PathPlanner_Dijkstra(9, 3);

        /*
         * 如果起点9或终点3被封锁，
         * 当前算法按业务逻辑最好应该无路径。
         */
        if ((node == 9) || (node == 3))
        {
            if (path.valid)
            {
                printf("    ERROR: blocked start/target still returns path\n");
                ok = 0;
            }
        }
        else
        {
            if (path.valid &&
                !validate_path(&path, 9, 3, mask))
            {
                ok = 0;
            }
        }

        report_result(name, ok, &path);
    }
}


static void test_all_edge_obstacles(void)
{
    uint8_t i;

    printf("\n=== O10 ~ O21 edge obstacle tests ===\n");

    for (i = 0; i < 12; i++)
    {
        char name[80];
        uint8_t obstacle_no = (uint8_t)(10 + i);
        uint32_t mask = (1UL << (9 + i));
        PathResult_t path;
        int ok = 1;

        snprintf(name,
                 sizeof(name),
                 "O%u blocks edge N%u-N%u",
                 (unsigned)obstacle_no,
                 (unsigned)g_test_edges[i].a,
                 (unsigned)g_test_edges[i].b);

        MapGraph_Build(mask);
        path = PathPlanner_Dijkstra(9, 3);

        if (path.valid &&
            !validate_path(&path, 9, 3, mask))
        {
            ok = 0;
        }

        report_result(name, ok, &path);
    }
}


static void test_all_start_target_pairs(void)
{
    uint8_t start;
    uint8_t target;

    printf("\n=== All start/target pairs on empty map ===\n");

    for (start = 1; start <= 9; start++)
    {
        for (target = 1; target <= 9; target++)
        {
            char name[64];
            PathResult_t path;
            int ok = 1;

            snprintf(name,
                     sizeof(name),
                     "N%u -> N%u",
                     (unsigned)start,
                     (unsigned)target);

            MapGraph_Build(0);
            path = PathPlanner_Dijkstra(start, target);

            /*
             * 空地图3x3连通图里，
             * 所有合法节点之间都应可达。
             */
            if (!path.valid)
            {
                printf("    ERROR: empty map should be reachable\n");
                ok = 0;
            }
            else if (!validate_path(&path,
                                    start,
                                    target,
                                    0))
            {
                ok = 0;
            }

            report_result(name, ok, &path);
        }
    }
}


int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  27_carrycar Automatic Path Test\n");
    printf("========================================\n");

    test_basic_cases();
    test_all_node_obstacles();
    test_all_edge_obstacles();
    test_all_start_target_pairs();

    printf("\n");
    printf("========================================\n");
    printf("PASS : %d\n", g_pass);
    printf("FAIL : %d\n", g_fail);
    printf("TOTAL: %d\n", g_pass + g_fail);
    printf("========================================\n");

    if (g_fail == 0)
    {
        printf("ALL TESTS PASSED\n");
        return 0;
    }

    printf("TEST FAILED\n");
    return 1;
}
