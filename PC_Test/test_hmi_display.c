#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "hmi_display.h"
#include "hmi_protocol.h"



#define TX_BUF_SIZE 2048

static uint8_t tx_buf[TX_BUF_SIZE];
static uint32_t tx_len = 0;

static int g_pass = 0;
static int g_fail = 0;


/* hmi_display.c 中声明了 extern huart1 */
UART_HandleTypeDef huart1;


/*
 * 模拟当前HMI模式。
 *
 * hmi_display.c 的 Periodic() 会调用
 * HMI_Mode_Get()。
 */
static HMI_DisplayMode_t mock_mode = HMI_MODE_NONE;


HMI_DisplayMode_t HMI_Mode_Get(void)
{
    return mock_mode;
}


/* ====================================================
 * 模拟 HAL_UART_Transmit
 *
 * 不真正发送串口，
 * 只把所有发送数据记录到 tx_buf。
 * ==================================================== */

HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart,
    const uint8_t *pData,
    uint16_t Size,
    uint32_t Timeout)
{
    uint16_t i;

    (void)huart;
    (void)Timeout;

    for (i = 0; i < Size; i++)
    {
        if (tx_len < TX_BUF_SIZE)
        {
            tx_buf[tx_len++] = pData[i];
        }
    }

    return HAL_OK;
}


static void reset_tx(void)
{
    memset(tx_buf, 0, sizeof(tx_buf));
    tx_len = 0;
}


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


/* ====================================================
 * 检查单条淘晶驰命令
 *
 * 期望格式：
 *
 * ASCII command
 * FF FF FF
 * ==================================================== */

static int expect_command(const char *cmd)
{
    uint32_t cmd_len;
    uint32_t i;

    cmd_len = (uint32_t)strlen(cmd);

    if (tx_len != cmd_len + 3)
    {
        printf("    ERROR: length mismatch\n");
        printf("    expected = %lu\n",
               (unsigned long)(cmd_len + 3));
        printf("    actual   = %lu\n",
               (unsigned long)tx_len);

        return 0;
    }

    for (i = 0; i < cmd_len; i++)
    {
        if (tx_buf[i] != (uint8_t)cmd[i])
        {
            printf("    ERROR: command mismatch at byte %lu\n",
                   (unsigned long)i);

            return 0;
        }
    }

    if ((tx_buf[cmd_len]     != 0xFF) ||
        (tx_buf[cmd_len + 1] != 0xFF) ||
        (tx_buf[cmd_len + 2] != 0xFF))
    {
        printf("    ERROR: missing FF FF FF\n");
        return 0;
    }

    return 1;
}


static void run_command_test(
    const char *name,
    void (*func)(void),
    const char *expected)
{
    reset_tx();

    func();

    if (expect_command(expected))
    {
        pass(name);
    }
    else
    {
        fail(name);
    }
}


/* ====================================================
 * 页面测试
 * ==================================================== */

static void test_pages(void)
{
    printf("\n=== Page switch tests ===\n");

    run_command_test(
        "Page config",
        HMI_Page_Config,
        "page page0"
    );

    run_command_test(
        "Page mode select",
        HMI_Page_ModeSelect,
        "page page3"
    );

    run_command_test(
        "Page race",
        HMI_Page_Race,
        "page page4"
    );

    run_command_test(
        "Page debug",
        HMI_Page_Debug,
        "page page5"
    );
}


/* ====================================================
 * page4
 * ==================================================== */

static void test_page4(void)
{
    printf("\n=== page4 race display tests ===\n");

    reset_tx();
    HMI_Race_SetTaskCode("123+231+312");

    if (expect_command(
        "page4.tTaskCode.txt=\"123+231+312\""))
    {
        pass("page4 tTaskCode");
    }
    else
    {
        fail("page4 tTaskCode");
    }


    reset_tx();
    HMI_Race_SetStage("TEST_STAGE");

    if (expect_command(
        "page4.tStage.txt=\"TEST_STAGE\""))
    {
        pass("page4 tStage");
    }
    else
    {
        fail("page4 tStage");
    }


    reset_tx();
    HMI_Race_SetAction("MOVE");

    if (expect_command(
        "page4.tAction.txt=\"MOVE\""))
    {
        pass("page4 tAction");
    }
    else
    {
        fail("page4 tAction");
    }


    reset_tx();
    HMI_Race_SetColor("RED");

    if (expect_command(
        "page4.tColor.txt=\"RED\""))
    {
        pass("page4 tColor");
    }
    else
    {
        fail("page4 tColor");
    }


    reset_tx();
    HMI_Race_SetResult("RUNNING");

    if (expect_command(
        "page4.tResult.txt=\"RUNNING\""))
    {
        pass("page4 tResult");
    }
    else
    {
        fail("page4 tResult");
    }
}


/* ====================================================
 * page5
 * ==================================================== */

static void test_page5(void)
{
    uint8_t path[] =
    {
        9, 8, 5, 2, 3
    };


    printf("\n=== page5 debug display tests ===\n");


    reset_tx();
    HMI_Debug_SetTaskCode("123");

    if (expect_command(
        "page5.tDbgTask.txt=\"123\""))
    {
        pass("page5 tDbgTask");
    }
    else
    {
        fail("page5 tDbgTask");
    }


    reset_tx();
    HMI_Debug_SetTargetYaw(90.25f);

    if (expect_command(
        "page5.tTargetYaw.txt=\"90.2\""))
    {
        pass("page5 tTargetYaw");
    }
    else
    {
        fail("page5 tTargetYaw");
    }


    reset_tx();
    HMI_Debug_SetCurrentYaw(-45.67f);

    if (expect_command(
        "page5.tCurrentYaw.txt=\"-45.7\""))
    {
        pass("page5 tCurrentYaw");
    }
    else
    {
        fail("page5 tCurrentYaw");
    }


    reset_tx();
    HMI_Debug_SetCurrentNode(8);

    if (expect_command(
        "page5.tCurrentNode.txt=\"8\""))
    {
        pass("page5 tCurrentNode");
    }
    else
    {
        fail("page5 tCurrentNode");
    }


    reset_tx();
    HMI_Debug_SetTargetNode(3);

    if (expect_command(
        "page5.tTargetNode.txt=\"3\""))
    {
        pass("page5 tTargetNode");
    }
    else
    {
        fail("page5 tTargetNode");
    }


    reset_tx();

    HMI_Debug_SetPath(
        path,
        sizeof(path)
    );

    if (expect_command(
        "page5.tPath.txt=\"9>8>5>2>3\""))
    {
        pass("page5 tPath");
    }
    else
    {
        fail("page5 tPath");
    }


    reset_tx();

    HMI_Debug_SetPath(
        NULL,
        0
    );

    if (expect_command(
        "page5.tPath.txt=\"NO PATH\""))
    {
        pass("page5 NO PATH");
    }
    else
    {
        fail("page5 NO PATH");
    }


    reset_tx();

    HMI_Debug_SetSpeed(321.4f);

    if (expect_command(
        "page5.tSpeed.txt=\"321\""))
    {
        pass("page5 tSpeed");
    }
    else
    {
        fail("page5 tSpeed");
    }
}


/* ====================================================
 * NULL保护
 * ==================================================== */

static void test_null_protection(void)
{
    printf("\n=== NULL protection ===\n");

    reset_tx();

    HMI_SendCommand(NULL);

    if (tx_len == 0)
    {
        pass("HMI_SendCommand NULL");
    }
    else
    {
        fail("HMI_SendCommand NULL");
    }


    reset_tx();

    HMI_Race_SetTaskCode(NULL);

    if (tx_len == 0)
    {
        pass("Race task NULL");
    }
    else
    {
        fail("Race task NULL");
    }


    reset_tx();

    HMI_Debug_SetTaskCode(NULL);

    if (tx_len == 0)
    {
        pass("Debug task NULL");
    }
    else
    {
        fail("Debug task NULL");
    }
}


/* ====================================================
 * Periodic刷新
 * ==================================================== */

static void test_periodic(void)
{
    printf("\n=== Periodic update tests ===\n");


    HMI_Display_Init();


    /*
     * 非DEBUG模式：
     * 即使时间够，也不应该发送。
     */
    mock_mode = HMI_MODE_RACE;

    reset_tx();

    HMI_Display_Periodic(
        1000,
        12.3f,
        90.0f,
        300.0f
    );

    if (tx_len == 0)
    {
        pass("Periodic disabled in race mode");
    }
    else
    {
        fail("Periodic disabled in race mode");
    }


    /*
     * DEBUG模式，
     * 但只有100ms，不到150ms。
     */
    mock_mode = HMI_MODE_DEBUG;

    HMI_Display_Init();

    reset_tx();

    HMI_Display_Periodic(
        100,
        12.3f,
        90.0f,
        300.0f
    );

    if (tx_len == 0)
    {
        pass("Periodic rate limit <150ms");
    }
    else
    {
        fail("Periodic rate limit <150ms");
    }


    /*
     * 到150ms后必须发送：
     *
     * current yaw
     * target yaw
     * speed
     */
    reset_tx();

    HMI_Display_Periodic(
        150,
        12.3f,
        90.0f,
        300.0f
    );

    if (tx_len > 0)
    {
        pass("Periodic sends at 150ms");
    }
    else
    {
        fail("Periodic sends at 150ms");
    }


    /*
     * 再过50ms，不应该再次发送。
     */
    reset_tx();

    HMI_Display_Periodic(
        200,
        13.0f,
        91.0f,
        310.0f
    );

    if (tx_len == 0)
    {
        pass("Periodic rate limit after update");
    }
    else
    {
        fail("Periodic rate limit after update");
    }


    /*
     * 再到300ms，可以再次刷新。
     */
    reset_tx();

    HMI_Display_Periodic(
        300,
        13.0f,
        91.0f,
        310.0f
    );

    if (tx_len > 0)
    {
        pass("Periodic second update");
    }
    else
    {
        fail("Periodic second update");
    }
}


int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf("  27_carrycar HMI Display PC Test\n");
    printf("========================================\n");

    HMI_Display_Init();

    test_pages();
    test_page4();
    test_page5();
    test_null_protection();
    test_periodic();

    printf("\n");
    printf("========================================\n");
    printf("PASS : %d\n", g_pass);
    printf("FAIL : %d\n", g_fail);
    printf("TOTAL: %d\n", g_pass + g_fail);
    printf("========================================\n");

    if (g_fail == 0)
    {
        printf("ALL HMI DISPLAY TESTS PASSED\n");
        return 0;
    }

    printf("HMI DISPLAY TEST FAILED\n");
    return 1;
}
