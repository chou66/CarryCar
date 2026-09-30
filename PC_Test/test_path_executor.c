#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "path_executor.h"


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


static void check(int condition, const char *name)
{
    if (condition)
    {
        pass(name);
    }
    else
    {
        fail(name);
    }
}


static PathResult_t make_path(
    const uint8_t *nodes,
    uint8_t length)
{
    PathResult_t path;
    uint8_t i;

    memset(&path, 0, sizeof(path));

    path.valid = 1;
    path.length = length;

    for (i = 0;
         (i < length) && (i < PATH_MAX_LEN);
         i++)
    {
        path.node[i] = nodes[i];
    }

    return path;
}


/* ====================================================
 * 基本路径：
 *
 * 9 -> 8 -> 5 -> 2 -> 3
 * ==================================================== */

static void test_normal_path(void)
{
    PathExecutor_t exec;

    uint8_t nodes[] =
    {
        9, 8, 5, 2, 3
    };

    PathResult_t path =
        make_path(nodes, 5);


    printf("\n=== Normal path execution ===\n");


    PathExecutor_Init(&exec);

    check(
        PathExecutor_GetState(&exec) ==
        PATH_EXEC_IDLE,
        "Init -> IDLE"
    );


    check(
        PathExecutor_Start(&exec, &path) == 1,
        "Start 9->8->5->2->3"
    );


    check(
        PathExecutor_GetState(&exec) ==
        PATH_EXEC_RUNNING,
        "State -> RUNNING"
    );


    check(
        PathExecutor_GetCurrentNode(&exec) == 9 &&
        PathExecutor_GetNextNode(&exec) == 8,
        "Edge 9 -> 8"
    );


    PathExecutor_EdgeReached(&exec);

    check(
        PathExecutor_GetCurrentNode(&exec) == 8 &&
        PathExecutor_GetNextNode(&exec) == 5,
        "Reach N8, next N5"
    );


    PathExecutor_EdgeReached(&exec);

    check(
        PathExecutor_GetCurrentNode(&exec) == 5 &&
        PathExecutor_GetNextNode(&exec) == 2,
        "Reach N5, next N2"
    );


    PathExecutor_EdgeReached(&exec);

    check(
        PathExecutor_GetCurrentNode(&exec) == 2 &&
        PathExecutor_GetNextNode(&exec) == 3,
        "Reach N2, next N3"
    );


    PathExecutor_EdgeReached(&exec);

    check(
        PathExecutor_GetCurrentNode(&exec) == 3,
        "Reach final N3"
    );


    check(
        PathExecutor_GetNextNode(&exec) == 0,
        "No next node after finish"
    );


    check(
        PathExecutor_GetState(&exec) ==
        PATH_EXEC_DONE,
        "State -> DONE"
    );


    check(
        PathExecutor_IsDone(&exec) == 1,
        "IsDone"
    );
}


/* ====================================================
 * Pause / Resume
 * ==================================================== */

static void test_pause_resume(void)
{
    PathExecutor_t exec;

    uint8_t nodes[] =
    {
        9, 6, 3
    };

    PathResult_t path =
        make_path(nodes, 3);


    printf("\n=== Pause / Resume ===\n");


    PathExecutor_Init(&exec);

    PathExecutor_Start(&exec, &path);

    PathExecutor_Pause(&exec);


    check(
        PathExecutor_GetState(&exec) ==
        PATH_EXEC_PAUSED,
        "Pause"
    );


    /*
     * 暂停时，即使错误调用EdgeReached，
     * 也不允许路径推进。
     */
    check(
        PathExecutor_EdgeReached(&exec) == 0,
        "Reject EdgeReached while paused"
    );


    check(
        PathExecutor_GetCurrentNode(&exec) == 9 &&
        PathExecutor_GetNextNode(&exec) == 6,
        "Path unchanged while paused"
    );


    PathExecutor_Resume(&exec);


    check(
        PathExecutor_GetState(&exec) ==
        PATH_EXEC_RUNNING,
        "Resume"
    );


    PathExecutor_EdgeReached(&exec);


    check(
        PathExecutor_GetCurrentNode(&exec) == 6 &&
        PathExecutor_GetNextNode(&exec) == 3,
        "Continue after resume"
    );
}


/* ====================================================
 * 起点 == 终点
 * ==================================================== */

static void test_single_node(void)
{
    PathExecutor_t exec;

    uint8_t nodes[] =
    {
        5
    };

    PathResult_t path =
        make_path(nodes, 1);


    printf("\n=== Single node path ===\n");


    PathExecutor_Init(&exec);


    check(
        PathExecutor_Start(&exec, &path) == 1,
        "Start single node N5"
    );


    check(
        PathExecutor_GetCurrentNode(&exec) == 5,
        "Current node N5"
    );


    check(
        PathExecutor_GetNextNode(&exec) == 0,
        "Single node has no next"
    );


    check(
        PathExecutor_IsDone(&exec) == 1,
        "Single node immediately DONE"
    );
}


/* ====================================================
 * 非法路径
 * ==================================================== */

static void test_invalid_paths(void)
{
    PathExecutor_t exec;
    PathResult_t path;

    printf("\n=== Invalid path tests ===\n");


    PathExecutor_Init(&exec);

    check(
        PathExecutor_Start(&exec, NULL) == 0,
        "Reject NULL path"
    );


    memset(&path, 0, sizeof(path));

    path.valid = 0;
    path.length = 0;


    check(
        PathExecutor_Start(&exec, &path) == 0,
        "Reject invalid Dijkstra result"
    );


    memset(&path, 0, sizeof(path));

    path.valid = 1;
    path.length = 0;


    check(
        PathExecutor_Start(&exec, &path) == 0,
        "Reject zero length"
    );


    memset(&path, 0, sizeof(path));

    path.valid = 1;
    path.length = 2;

    path.node[0] = 9;
    path.node[1] = 10;


    check(
        PathExecutor_Start(&exec, &path) == 0,
        "Reject node > N9"
    );


    memset(&path, 0, sizeof(path));

    path.valid = 1;
    path.length = 2;

    path.node[0] = 0;
    path.node[1] = 3;


    check(
        PathExecutor_Start(&exec, &path) == 0,
        "Reject node N0"
    );
}


/* ====================================================
 * DONE后不能继续推进
 * ==================================================== */

static void test_done_protection(void)
{
    PathExecutor_t exec;

    uint8_t nodes[] =
    {
        1, 2
    };

    PathResult_t path =
        make_path(nodes, 2);


    printf("\n=== DONE protection ===\n");


    PathExecutor_Init(&exec);

    PathExecutor_Start(&exec, &path);

    PathExecutor_EdgeReached(&exec);


    check(
        PathExecutor_GetState(&exec) ==
        PATH_EXEC_DONE,
        "1->2 finished"
    );


    check(
        PathExecutor_EdgeReached(&exec) == 0,
        "Reject advance after DONE"
    );


    check(
        PathExecutor_GetCurrentNode(&exec) == 2,
        "Current remains N2"
    );
}


int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  27_carrycar Path Executor PC Test\n");
    printf("========================================\n");


    test_normal_path();
    test_pause_resume();
    test_single_node();
    test_invalid_paths();
    test_done_protection();


    printf("\n");
    printf("========================================\n");
    printf("PASS : %d\n", g_pass);
    printf("FAIL : %d\n", g_fail);
    printf("TOTAL: %d\n", g_pass + g_fail);
    printf("========================================\n");


    if (g_fail == 0)
    {
        printf("ALL PATH EXECUTOR TESTS PASSED\n");
        return 0;
    }


    printf("PATH EXECUTOR TEST FAILED\n");

    return 1;
}
