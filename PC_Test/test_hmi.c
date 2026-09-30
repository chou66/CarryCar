#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "hmi_protocol.h"


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


static void feed_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        HMI_Protocol_RxByte(data[i]);
    }
}


/*
 * 构造一帧合法地图配置：
 *
 * AA 55 01
 * SS RR
 * O1 ... O21
 * 55 AA
 */
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
        if (mask & (1UL << i))
        {
            frame[5 + i] = 1;
        }
        else
        {
            frame[5 + i] = 0;
        }
    }

    frame[26] = HMI_TAIL_1;
    frame[27] = HMI_TAIL_2;
}


static void test_init(void)
{
    HMI_Protocol_Init();

    if ((HMI_MapConfig_IsValid() == 0) &&
        (HMI_MapConfig_IsUpdated() == 0) &&
        (HMI_Control_GetPauseRequest() == 0) &&
        (HMI_Control_GetResumeRequest() == 0) &&
        (HMI_Mode_Get() == HMI_MODE_NONE) &&
        (HMI_Mode_IsUpdated() == 0))
    {
        pass("Protocol init");
    }
    else
    {
        fail("Protocol init");
    }
}


static void test_map_normal(void)
{
    uint8_t frame[HMI_MAP_FRAME_LEN];
    HMI_MapConfig_t map;

    HMI_Protocol_Init();

    /*
     * 示例：
     *
     * start  = N9
     * target = N3
     *
     * O10 = bit9
     * O18 = bit17
     */
    make_map_frame(
        frame,
        9,
        3,
        (1UL << 9) | (1UL << 17)
    );

    feed_bytes(frame, sizeof(frame));

    if (!HMI_MapConfig_IsValid())
    {
        fail("Valid map frame");
        return;
    }

    if (!HMI_MapConfig_IsUpdated())
    {
        fail("Map updated flag");
        return;
    }

    if (!HMI_MapConfig_TakeUpdatedSnapshot(&map))
    {
        fail("Take map snapshot");
        return;
    }

    if (map.start_node != 9)
    {
        fail("Map start node");
        return;
    }

    if (map.return_node != 3)
    {
        fail("Map return node");
        return;
    }

    if (map.obstacle_mask !=
        ((1UL << 9) | (1UL << 17)))
    {
        printf("Expected mask: 0x%08lX\n",
               (unsigned long)
               ((1UL << 9) | (1UL << 17)));

        printf("Actual mask  : 0x%08lX\n",
               (unsigned long)map.obstacle_mask);

        fail("Obstacle mask");
        return;
    }

    if (map.obstacle[9] != 1 ||
        map.obstacle[17] != 1)
    {
        fail("Obstacle array");
        return;
    }

    /*
     * TakeSnapshot() 后 updated 应清零。
     */
    if (HMI_MapConfig_IsUpdated())
    {
        fail("Snapshot clears updated");
        return;
    }

    pass("Valid map frame + snapshot");
}


static void test_pause(void)
{
    const uint8_t frame[] =
    {
        0xAA, 0x55,
        0x02,
        0x01,
        0x55, 0xAA
    };

    HMI_Protocol_Init();

    feed_bytes(frame, sizeof(frame));

    if ((HMI_Control_GetPauseRequest() == 1) &&
        (HMI_Control_GetResumeRequest() == 0))
    {
        pass("Pause command");
    }
    else
    {
        fail("Pause command");
    }

    HMI_Control_ClearPauseRequest();

    if (HMI_Control_GetPauseRequest() == 0)
    {
        pass("Clear pause request");
    }
    else
    {
        fail("Clear pause request");
    }
}


static void test_resume(void)
{
    const uint8_t frame[] =
    {
        0xAA, 0x55,
        0x02,
        0x02,
        0x55, 0xAA
    };

    HMI_Protocol_Init();

    feed_bytes(frame, sizeof(frame));

    if ((HMI_Control_GetResumeRequest() == 1) &&
        (HMI_Control_GetPauseRequest() == 0))
    {
        pass("Resume command");
    }
    else
    {
        fail("Resume command");
    }
}


static void test_modes(void)
{
    const uint8_t race[] =
    {
        0xAA, 0x55,
        0x03,
        0x01,
        0x55, 0xAA
    };

    const uint8_t debug[] =
    {
        0xAA, 0x55,
        0x03,
        0x02,
        0x55, 0xAA
    };

    HMI_Protocol_Init();

    feed_bytes(race, sizeof(race));

    if ((HMI_Mode_Get() == HMI_MODE_RACE) &&
        (HMI_Mode_IsUpdated() == 1))
    {
        pass("Race display mode");
    }
    else
    {
        fail("Race display mode");
    }

    HMI_Mode_ClearUpdated();

    feed_bytes(debug, sizeof(debug));

    if ((HMI_Mode_Get() == HMI_MODE_DEBUG) &&
        (HMI_Mode_IsUpdated() == 1))
    {
        pass("Debug display mode");
    }
    else
    {
        fail("Debug display mode");
    }
}


/*
 * 错误帧不能污染已经存在的合法地图。
 */
static void test_bad_tail(void)
{
    uint8_t good[HMI_MAP_FRAME_LEN];
    uint8_t bad[HMI_MAP_FRAME_LEN];

    HMI_Protocol_Init();

    make_map_frame(good, 9, 3, 0);

    feed_bytes(good, sizeof(good));

    HMI_MapConfig_ClearUpdated();

    make_map_frame(
        bad,
        3,
        9,
        (1UL << 20)
    );

    /*
     * 故意破坏帧尾。
     */
    bad[27] = 0x00;

    feed_bytes(bad, sizeof(bad));

    if (HMI_MapConfig_IsUpdated())
    {
        fail("Reject bad tail");
        return;
    }

    /*
     * 原合法配置应该仍然存在。
     */
    if ((g_hmi_map.start_node == 9) &&
        (g_hmi_map.return_node == 3) &&
        (g_hmi_map.obstacle_mask == 0))
    {
        pass("Reject bad tail");
    }
    else
    {
        fail("Bad frame changed old map");
    }
}


static void test_invalid_start(void)
{
    uint8_t frame[HMI_MAP_FRAME_LEN];

    HMI_Protocol_Init();

    /*
     * 当前协议只允许3或9。
     */
    make_map_frame(frame, 5, 3, 0);

    feed_bytes(frame, sizeof(frame));

    if (!HMI_MapConfig_IsValid() &&
        !HMI_MapConfig_IsUpdated())
    {
        pass("Reject invalid start node");
    }
    else
    {
        fail("Reject invalid start node");
    }
}


static void test_invalid_obstacle(void)
{
    uint8_t frame[HMI_MAP_FRAME_LEN];

    HMI_Protocol_Init();

    make_map_frame(frame, 9, 3, 0);

    /*
     * O5只能是0或1。
     * 故意发送2。
     */
    frame[5 + 4] = 2;

    feed_bytes(frame, sizeof(frame));

    if (!HMI_MapConfig_IsValid())
    {
        pass("Reject obstacle value > 1");
    }
    else
    {
        fail("Reject obstacle value > 1");
    }
}


/*
 * 测试状态机重新同步：
 *
 * 噪声 + AA AA 55 ...
 *
 * 第二个AA应该能够作为新帧起点。
 */
static void test_resync(void)
{
    uint8_t frame[HMI_MAP_FRAME_LEN];
    uint8_t prefix[] =
    {
        0x11,
        0x22,
        0x33,
        0xAA
    };

    HMI_Protocol_Init();

    make_map_frame(
        frame,
        9,
        3,
        (1UL << 20)
    );

    feed_bytes(prefix, sizeof(prefix));

    /*
     * 此时状态机已经收到一个AA。
     *
     * 再喂完整frame：
     * 开头又是AA 55，
     *
     * 实际字节流：
     *
     * ... AA AA 55 01 ...
     */
    feed_bytes(frame, sizeof(frame));

    if (HMI_MapConfig_IsValid() &&
        g_hmi_map.obstacle_mask == (1UL << 20))
    {
        pass("Receiver resync AA AA 55");
    }
    else
    {
        fail("Receiver resync AA AA 55");
    }
}


int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  27_carrycar HMI Protocol PC Test\n");
    printf("========================================\n\n");

    test_init();
    test_map_normal();

    test_pause();
    test_resume();

    test_modes();

    test_bad_tail();
    test_invalid_start();
    test_invalid_obstacle();

    test_resync();

    printf("\n");
    printf("========================================\n");
    printf("PASS : %d\n", g_pass);
    printf("FAIL : %d\n", g_fail);
    printf("TOTAL: %d\n", g_pass + g_fail);
    printf("========================================\n");

    if (g_fail == 0)
    {
        printf("ALL HMI TESTS PASSED\n");
        return 0;
    }

    printf("HMI TEST FAILED\n");
    return 1;
}
