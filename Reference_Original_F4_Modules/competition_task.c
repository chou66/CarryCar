/*
 * competition_task.c
 * ==========================================================================
 * 2026 智能搬运车任务一、任务二总状态机。
 *
 * 本文件只负责“比赛流程编排”，底层运动由 chassis.c 完成：
 *
 *   四轮编码器  -> 计算车体前进/横移距离；
 *   HWT101      -> 提供实时航向，将车体位移换算为地图 X/Y；
 *   八路寻迹模块 -> 沿黑线闭环行驶；
 *   二维码       -> 读取奖杯/颜色任务，并在固定点修正 X/Y；
 *   K230         -> 放料前识别最小同心圆，做厘米级末端微调；
 *   转盘/舵机    -> 收取、存储和释放奖杯或物料。
 *
 * 地图坐标：
 *   +X 向场地右侧，+Y 向场地上方；
 *   yaw=0° 时车头朝 +Y；
 *   逆时针为正角度，顺时针为负角度。
 *
 * 车体相对移动：
 *   x_mm > 0 为右移，x_mm < 0 为左移；
 *   y_mm > 0 为前进，y_mm < 0 为后退。
 *
 * 任务二开始顺序（按当前最新描述）：
 *   Home -> 圆盘翻到车前并下降L0 -> 保持Yaw=0°斜线直达任务二二维码点
 *   -> 到点原地转Yaw=-90° -> 扫码并记录A/B/C名次
 *   -> 继续直行找黑线 -> 逆时针寻迹收取三个奖杯。
 *
 * 任务一真实切入动作：
 *   放完季军 -> 逆时针90° -> 前进285 mm -> 右移240 mm
 *   -> 黑线位于八路模块中间 -> 逆时针寻迹收五个物料。
 *
 * 任务一放置顺序（最终实车）：
 *   A -> B -> C -> D -> E。
 *   全程MapYaw=0°，每点K230后按真实颜色槽放料。
 *
 * 所有未实测的高度、局部距离和K230比例集中在competition_config.h中。
 * ==========================================================================
 */

#include "competition_task.h"
#include "competition_config.h"
#include "app_control.h"
#include "bluetooth.h"

#include "chassis.h"
#include "gy33.h"
#include "hwt101.h"
#include "k230.h"
#include "line_sensor.h"
#include "mj6000.h"
#include "servo.h"
#include "turnable2.h"

#include <math.h>
#include <string.h>

typedef enum
{
    MAT_UNKNOWN = 0,
    MAT_BLACK,
    MAT_WHITE,
    MAT_RED,
    MAT_GREEN,
    MAT_BLUE
} MaterialColor_t;

typedef enum
{
    PODIUM_UNKNOWN = 0,
    PODIUM_FIRST,
    PODIUM_SECOND,
    PODIUM_THIRD
} PodiumRank_t;

typedef struct
{
    PodiumRank_t trophy_a;
    PodiumRank_t trophy_b;
    PodiumRank_t trophy_c;
    uint8_t valid;
} TrophyQrResult_t;

static CompetitionState_t s_state = COMP_STATE_IDLE;
static uint32_t s_state_enter_ms = 0U;
static TrophyQrResult_t s_trophy_qr;
static MaterialColor_t s_target_by_position[5]; /* A B C D E */
static uint8_t s_material_count = 0U;
static uint8_t s_init_ok = 0U;
static char s_route_error_text[4] = {'E', '0', '0', '\0'};
static const char *s_last_error = "-";

/* 二维码断线临时随机任务，不用stdlib rand()以节省代码空间。 */
static uint32_t s_qr_random_state = 0x13579BDFUL;

static uint32_t Route_RandomNext(void)
{
    s_qr_random_state =
        s_qr_random_state * 1664525UL +
        1013904223UL +
        HAL_GetTick();
    return s_qr_random_state;
}


static CompetitionModule_t s_selected_module = COMP_MODULE_FULL_ROUTE;
static CompetitionModule_t s_active_module = COMP_MODULE_FULL_ROUTE;
static CompetitionState_t s_module_stop_before_state = COMP_STATE_IDLE;
static uint8_t s_module_boundary_enabled = 0U;

/* 固定收取 C/B/A，分别进入 slot 0/1/2。 */
static const char s_trophy_id_by_slot[3] = {'C', 'B', 'A'};

/**
 * @brief 把角度归一化到 [-180°, 180°]。
 *
 * 用于处理 HWT101 在 +180°/-180°边界处的跳变，保证累计转角连续。
 */
static float Route_NormalizeAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 将浮点值限制在指定上下限之间。
 *
 * 用于限制寻迹转向量和 K230 单次修正距离，避免动作过猛。
 */
static float Route_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

CompetitionState_t CompetitionTask_GetState(void)
{
    return s_state;
}


void CompetitionTask_SelectModule(CompetitionModule_t module)
{
    if ((s_state == COMP_STATE_FINISHED) || (s_state == COMP_STATE_ERROR))
    {
        s_state = COMP_STATE_IDLE;
    }

    if ((module < COMP_MODULE_COUNT) && (s_state == COMP_STATE_IDLE))
    {
        s_selected_module = module;
    }
}

void CompetitionTask_SelectNextModule(void)
{
    if ((s_state == COMP_STATE_FINISHED) || (s_state == COMP_STATE_ERROR))
    {
        s_state = COMP_STATE_IDLE;
    }
    if (s_state != COMP_STATE_IDLE) return;
    s_selected_module = (CompetitionModule_t)(
        ((uint32_t)s_selected_module + 1U) % (uint32_t)COMP_MODULE_COUNT);
}

void CompetitionTask_SelectPreviousModule(void)
{
    if ((s_state == COMP_STATE_FINISHED) || (s_state == COMP_STATE_ERROR))
    {
        s_state = COMP_STATE_IDLE;
    }
    if (s_state != COMP_STATE_IDLE) return;

    if (s_selected_module == COMP_MODULE_FULL_ROUTE)
    {
        s_selected_module = (CompetitionModule_t)(COMP_MODULE_COUNT - 1U);
    }
    else
    {
        s_selected_module = (CompetitionModule_t)((uint32_t)s_selected_module - 1U);
    }
}

CompetitionModule_t CompetitionTask_GetSelectedModule(void)
{
    return s_selected_module;
}

CompetitionModule_t CompetitionTask_GetActiveModule(void)
{
    return s_active_module;
}

uint8_t CompetitionTask_IsRunning(void)
{
    return ((s_state != COMP_STATE_IDLE) &&
            (s_state != COMP_STATE_FINISHED) &&
            (s_state != COMP_STATE_ERROR)) ? 1U : 0U;
}

uint8_t CompetitionTask_IsInitialized(void)
{
    return s_init_ok;
}

const char *CompetitionTask_GetLastError(void)
{
    return (s_last_error != NULL) ? s_last_error : "?";
}

/**
 * @brief 停车后切换到下一个比赛状态。
 *
 * 先停车可以防止上一状态的速度命令残留到下一状态。
 */
static void Route_StopToIdle(void)
{
    Chassis_Stop();
#if COMP_ENABLE_LIFT
    Lift_Stop();
#endif
    Turnable_ResetStateMachines();
    s_state = COMP_STATE_IDLE;
    s_state_enter_ms = HAL_GetTick();
    s_module_boundary_enabled = 0U;
    AppControl_SetRunning(0U);
}

static void Route_ChangeState(CompetitionState_t next)
{
    Chassis_Stop();

    /* 单模块调试到达边界后自动停车，不继续串到下一模块。 */
    if ((s_module_boundary_enabled != 0U) &&
        (next == s_module_stop_before_state))
    {
        Route_StopToIdle();
        return;
    }

    s_state = next;
    s_state_enter_ms = HAL_GetTick();
}

/**
 * @brief 统一错误处理。
 *
 * 任一步失败后停止底盘、复位转盘子状态机并进入 ERROR 状态。
 */
static void Route_EnterError(uint8_t code)
{
    uint8_t value = code;
    uint8_t tens = 0U;

    while (value >= 10U)
    {
        value -= 10U;
        tens++;
    }

    s_route_error_text[0] = 'E';
    s_route_error_text[1] = (char)('0' + tens);
    s_route_error_text[2] = (char)('0' + value);
    s_route_error_text[3] = '\0';
    s_last_error = s_route_error_text;

    if (AppControl_StopRequested() != 0U)
    {
        Route_StopToIdle();
        return;
    }

    Chassis_Stop();
#if COMP_ENABLE_LIFT
    Lift_Stop();
#endif
    Turnable_ResetStateMachines();

    s_state = COMP_STATE_ERROR;
    s_state_enter_ms = HAL_GetTick();
    AppControl_SetRunning(0U);
}


/**
 * @brief 使用编码器里程计移动到地图绝对坐标。
 * @param x_mm 地图目标 X，单位 mm。
 * @param y_mm 地图目标 Y，单位 mm。
 * @param speed 移动速度。
 * @return 1 到点且误差合格；0 多次尝试仍失败。
 */






/**
 * @brief 执行车体坐标系下的相对移动。
 *
 * x正值右移、x负值左移；y正值前进、y负值后退。
 * 用于压线、放料前进、放料后退等局部动作。
 */




/**
 * @brief 任务一放料换点：保持车头固定朝Home端，先横移再纵移。
 *
 * 已经放好的圆柱在圆盘开口后方。若直接斜线去下一点，展开圆盘的
 * 开口/前缘可能扫过刚放好的圆柱。先保持当前Y只改X，可以先把整车
 * 从已放物料侧面拉开；再沿Y到下一同心圆的K230粗定位点。
 */
static uint8_t Route_RotateTo(float yaw_deg);

static uint8_t Route_MovePosition(float x_mm,
                                      float y_mm,
                                      uint16_t rpm,
                                      uint8_t acc)
{
    return (Chassis_MoveMMPositionBlocking(x_mm, y_mm, rpm, acc,
                                           POS_ROUTE_TIMEOUT_MS) != false) ? 1U : 0U;
}



/**
 * @brief 利用 HWT101 航向闭环旋转到地图绝对角度。
 */
static uint8_t Route_RotateTo(float yaw_deg)
{
    ChassisPose_t pose;
    float delta_deg;
    uint8_t use_fast = 0U;
    uint8_t ok;

    /*
     * V2：这里只用缓存姿态判断是否启用fast，不再为了“选速度档”
     * 额外阻塞查询一次1~4号编码器。真正开始旋转时底层仍会先结算一次里程。
     */
    pose = Chassis_GetPose();
    if (pose.valid != false)
    {
        delta_deg = fabsf(Route_NormalizeAngle(yaw_deg - pose.yaw_deg));
        if (delta_deg >= 75.0f)
        {
            use_fast = 1U;
        }
    }

    if (use_fast != 0U)
    {
        Chassis_SetRotateFastMode(true);
    }

    ok = (Chassis_RotateToYaw(yaw_deg,
                              COMP_ROTATE_TIMEOUT_MS) != false) ? 1U : 0U;

    if (use_fast != 0U)
    {
        Chassis_SetRotateFastMode(false);
    }

    if (ok == 0U)
    {
        Chassis_Stop();
        return 0U;
    }

    return 1U;
}

static uint8_t Route_RotateToFast(float yaw_deg)
{
    uint8_t ok;

    Chassis_SetRotateFastMode(true);
    ok = (Chassis_RotateToYaw(yaw_deg,
                              COMP_ROTATE_TIMEOUT_MS) != false) ? 1U : 0U;
    Chassis_SetRotateFastMode(false);

    if (ok == 0U)
    {
        Chassis_Stop();
        return 0U;
    }

    return 1U;
}

static uint8_t Route_RotateToPrecise(float yaw_deg)
{
    if (Chassis_RotateToYawPrecise(yaw_deg,
                                   COMP_ROTATE_TIMEOUT_MS) == false)
    {
        Chassis_Stop();
        return 0U;
    }
    return 1U;
}


/**
 * @brief 先到地图 X/Y，再旋转到指定 yaw。
 *
 * 用于二维码点和放料预备点等同时要求位置与朝向的目标。
 */



/* ============================= Turnable2 ============================= */

static uint8_t TurnablePrepareTask1GetBlocking(uint8_t slot)
{
#if COMP_ENABLE_TURNTABLE
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < TURNABLE_PREPARE_TIMEOUT_MS)
    {
        AppControl_Service();
        if (AppControl_StopRequested() != 0U)
        {
            Chassis_Stop();
            return 0U;
        }
        Turnable_Get_Material_Task1(slot, 1U);

        if (flag_task1_get_ready != 0U) return 1U;

        if (flag_task1_get_done != 0U)
        {
            Turnable_Get_Material_Task1(slot, 0U);
            return 0U;
        }

        HAL_Delay(TURNABLE_POLL_PERIOD_MS);
    }

    Turnable_Get_Material_Task1(slot, 0U);
    return 0U;
#else
    (void)slot;
    return 1U;
#endif
}




static uint8_t TurnablePrepareTask2GetBlocking(uint8_t slot)
{
#if COMP_ENABLE_TURNTABLE
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < TURNABLE_PREPARE_TIMEOUT_MS)
    {
        AppControl_Service();
        if (AppControl_StopRequested() != 0U)
        {
            Chassis_Stop();
            return 0U;
        }
        Turnable_Get_Material_Task2(slot, 1U);

        if (flag_task2_get_ready != 0U) return 1U;

        if (flag_task2_get_done != 0U)
        {
            Turnable_Get_Material_Task2(slot, 0U);
            return 0U;
        }

        HAL_Delay(TURNABLE_POLL_PERIOD_MS);
    }

    Turnable_Get_Material_Task2(slot, 0U);
    return 0U;
#else
    (void)slot;
    return 1U;
#endif
}




static uint8_t TurnableAlignTask1PutBlocking(GY33_CustomColor_t color,
                                              int8_t *aligned_slot)
{
#if COMP_ENABLE_TURNTABLE
    int8_t slot = Turnable_Find_Slot_By_Color(color);

    /*
     * Task1放料不再依赖另一套put子状态机的done/ready时序。
     * 无论当前是虚拟颜色还是以后恢复真实GY33颜色，只要槽位记录有效：
     *   按颜色找到物理槽 -> 直接转到该槽开口 -> 等原机械稳定时间。
     *
     * 这样K230已经对准圆以后，不会因为put状态机残留/握手超时而
     * 停在原地报“A put”。
     */
    if ((slot < 0) || (slot >= (int8_t)TURNABLE_TASK1_SLOT_COUNT))
    {
        return 0U;
    }

    Servo_Spin_360_SetAngle(task1_configs[(uint8_t)slot].angle_opening);

    if (AppControl_Delay(task1_configs[(uint8_t)slot].time_wait_align) == 0U)
    {
        return 0U;
    }

    if (aligned_slot != NULL)
    {
        *aligned_slot = slot;
    }

    return 1U;
#else
    (void)color;
    if (aligned_slot != NULL) *aligned_slot = 0;
    return 1U;
#endif
}

static uint8_t TurnableAlignTask2PutBlocking(uint8_t slot)
{
#if COMP_ENABLE_TURNTABLE
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < TURNABLE_TASK2_PUT_TIMEOUT_MS)
    {
        AppControl_Service();
        if (AppControl_StopRequested() != 0U)
        {
            Chassis_Stop();
            return 0U;
        }
        Turnable_Put_Material_Task2(slot, 1U);

        if (flag_task2_put_done != 0U) return 1U;

        HAL_Delay(TURNABLE_POLL_PERIOD_MS);
    }

    Turnable_Put_Material_Task2(slot, 0U);
    return 0U;
#else
    (void)slot;
    return 1U;
#endif
}

/* ============================= QR parsing ============================= */

static GY33_CustomColor_t MaterialColorToGy33(MaterialColor_t color)
{
    switch (color)
    {
        case MAT_RED:   return GY33_COLOR_RED;
        case MAT_GREEN: return GY33_COLOR_GREEN;
        case MAT_BLUE:  return GY33_COLOR_BLUE;
        case MAT_BLACK: return GY33_COLOR_BLACK;
        case MAT_WHITE: return GY33_COLOR_WHITE;
        default:        return GY33_COLOR_UNKNOWN;
    }
}


/**
 * @brief 检查A~E是否恰好包含红、绿、蓝、黑、白五种不同颜色。
 */
static uint8_t ValidateTask1TargetColors(void)
{
    uint8_t color_mask = 0U;

    for (uint8_t i = 0U; i < 5U; i++)
    {
        uint8_t color = (uint8_t)s_target_by_position[i];

        if ((color == (uint8_t)MAT_UNKNOWN) || (color > (uint8_t)MAT_BLUE))
        {
            return 0U;
        }

        if ((color_mask & (uint8_t)(1U << color)) != 0U)
        {
            return 0U;
        }

        color_mask |= (uint8_t)(1U << color);
    }

    return 1U;
}

/**
 * @brief 检查五个转盘槽位是否都识别成功且颜色没有重复。
 */
static void Task1_AssignDummyUniqueSlotColors(void)
{
#if T1_BYPASS_COLOR_RANDOM_PLACE
    /*
     * 这里只把槽位编号伪装成5种唯一颜色，完全不代表真实物料颜色。
     * 目的仅是复用现有“按颜色找槽->PD15转槽”的成熟放料代码。
     *
     * 实际A~E任务仍由ApplyRandomQrTask(0)随机生成，因此物料放置顺序
     * 每次可以变化；等黑色圆盘/顶盖完成后再恢复真实GY33识色。
     */
    global_block_material_color_array[0] = GY33_COLOR_RED;
    global_block_material_color_array[1] = GY33_COLOR_GREEN;
    global_block_material_color_array[2] = GY33_COLOR_BLUE;
    global_block_material_color_array[3] = GY33_COLOR_BLACK;
    global_block_material_color_array[4] = GY33_COLOR_WHITE;
#endif
}

static void Task1_ReportCollectedColors(void)
{
    static const char code[6] = {'U','R','G','B','K','W'};
    char msg[16];
    uint8_t i;
    uint8_t p = 4U;

    msg[0] = '#'; msg[1] = 'M'; msg[2] = 'C'; msg[3] = ' ';

    for (i = 0U; i < 5U; i++)
    {
        uint8_t c = (uint8_t)Turnable_GetSlotColor(i);
        msg[p++] = (c <= (uint8_t)GY33_COLOR_WHITE) ? code[c] : 'U';
        if (i < 4U) msg[p++] = ',';
    }

    msg[p++] = '\r';
    msg[p++] = '\n';
    msg[p] = '\0';
    (void)Bluetooth_SendString(msg);
}

static uint8_t ValidateCollectedMaterialColors(void)
{
    uint8_t color_mask = 0U;

    for (uint8_t slot = 0U; slot < 5U; slot++)
    {
        uint8_t color = (uint8_t)Turnable_GetSlotColor(slot);

        if ((color == (uint8_t)GY33_COLOR_UNKNOWN) ||
            (color > (uint8_t)GY33_COLOR_WHITE))
        {
            return 0U;
        }

        if ((color_mask & (uint8_t)(1U << color)) != 0U)
        {
            return 0U;
        }

        color_mask |= (uint8_t)(1U << color);
    }

    return 1U;
}

/**
 * @brief 点位测试阶段的颜色容错。
 *
 * 保留已经识别成功且不重复的真实颜色；
 * UNKNOWN/非法/重复槽临时填入尚未使用的颜色。
 *
 * 目的只有一个：即使GY33暂时没识别好，也让A~E五个点位继续全部跑完。
 * 这些补出来的颜色不是识别结果，正式比赛前应关闭
 * T1_COLOR_FAIL_CONTINUE_ROUTE_TEST并恢复严格校验。
 */
static void Task1_RepairCollectedColorsForRouteTest(void)
{
#if T1_COLOR_FAIL_CONTINUE_ROUTE_TEST
    uint8_t used[6] = {0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t slot;
    uint8_t color;
    uint8_t fill_color = (uint8_t)GY33_COLOR_RED;

    /* 第一遍：只保留合法且第一次出现的真实颜色。 */
    for (slot = 0U; slot < 5U; slot++)
    {
        color = (uint8_t)global_block_material_color_array[slot];

        if ((color < (uint8_t)GY33_COLOR_RED) ||
            (color > (uint8_t)GY33_COLOR_WHITE) ||
            (used[color] != 0U))
        {
            global_block_material_color_array[slot] =
                GY33_COLOR_UNKNOWN;
        }
        else
        {
            used[color] = 1U;
        }
    }

    /* 第二遍：失败槽按“剩余未用颜色”依次补齐，仅用于继续测路线。 */
    for (slot = 0U; slot < 5U; slot++)
    {
        if (global_block_material_color_array[slot] !=
            GY33_COLOR_UNKNOWN)
        {
            continue;
        }

        while ((fill_color <= (uint8_t)GY33_COLOR_WHITE) &&
               (used[fill_color] != 0U))
        {
            fill_color++;
        }

        if (fill_color <= (uint8_t)GY33_COLOR_WHITE)
        {
            global_block_material_color_array[slot] =
                (GY33_CustomColor_t)fill_color;
            used[fill_color] = 1U;
            fill_color++;
        }
    }

    /* 五个槽都已有物料，保证后面的按颜色找槽逻辑能继续跑点位。 */
    global_block_count = 5U;
#endif
}

static uint8_t ParseAsciiId(const char *text,
                            uint8_t maximum,
                            uint8_t *id_out)
{
    uint16_t value = 0U;
    uint8_t digits = 0U;

    if ((text == NULL) || (id_out == NULL))
    {
        return 0U;
    }

    while ((*text == ' ') || (*text == '\t') ||
           (*text == '\r') || (*text == '\n'))
    {
        text++;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        value = (uint16_t)(value * 10U + (uint16_t)(*text - '0'));
        digits++;
        text++;
        if (value > maximum) return 0U;
    }

    while ((*text == ' ') || (*text == '\t') ||
           (*text == '\r') || (*text == '\n'))
    {
        text++;
    }

    if ((digits == 0U) || (*text != '\0') ||
        (value == 0U) || (value > maximum))
    {
        return 0U;
    }

    *id_out = (uint8_t)value;
    return 1U;
}


/**
 * @brief 解析任务二二维码，建立 A/B/C 奖杯与冠亚季军的对应关系。
 */
static const TrophyQrResult_t s_task2_qr_map[6] =
{
    {PODIUM_FIRST,  PODIUM_SECOND, PODIUM_THIRD,  1U},
    {PODIUM_FIRST,  PODIUM_THIRD,  PODIUM_SECOND, 1U},
    {PODIUM_SECOND, PODIUM_FIRST,  PODIUM_THIRD,  1U},
    {PODIUM_THIRD,  PODIUM_FIRST,  PODIUM_SECOND, 1U},
    {PODIUM_SECOND, PODIUM_THIRD,  PODIUM_FIRST,  1U},
    {PODIUM_THIRD,  PODIUM_SECOND, PODIUM_FIRST,  1U}
};

static uint8_t ApplyTask2QrId(uint8_t id)
{
    if ((id == 0U) || (id > 6U))
    {
        memset(&s_trophy_qr, 0, sizeof(s_trophy_qr));
        return 0U;
    }

    s_trophy_qr = s_task2_qr_map[id - 1U];
    return 1U;
}

static uint8_t ParseTask2Qr(const char *text)
{
    uint8_t id;

    if (ParseAsciiId(text, 6U, &id) == 0U)
    {
        memset(&s_trophy_qr, 0, sizeof(s_trophy_qr));
        return 0U;
    }

    return ApplyTask2QrId(id);
}

static const MaterialColor_t s_task1_qr_map[16][5] =
{
    /* A          B          C          D          E */
    {MAT_BLACK,  MAT_WHITE, MAT_RED,   MAT_GREEN, MAT_BLUE }, /*  1 */
    {MAT_WHITE,  MAT_BLACK, MAT_RED,   MAT_GREEN, MAT_BLUE }, /*  2 */
    {MAT_WHITE,  MAT_BLACK, MAT_GREEN, MAT_RED,   MAT_BLUE }, /*  3 */
    {MAT_BLUE,   MAT_WHITE, MAT_BLACK, MAT_RED,   MAT_GREEN}, /*  4 */
    {MAT_WHITE,  MAT_RED,   MAT_BLUE,  MAT_BLACK, MAT_GREEN}, /*  5 */
    {MAT_BLACK,  MAT_RED,   MAT_BLUE,  MAT_WHITE, MAT_GREEN}, /*  6 */
    {MAT_BLUE,   MAT_GREEN, MAT_BLACK, MAT_WHITE, MAT_RED  }, /*  7 */
    {MAT_GREEN,  MAT_WHITE, MAT_BLUE,  MAT_BLACK, MAT_RED  }, /*  8 */
    {MAT_WHITE,  MAT_GREEN, MAT_BLACK, MAT_BLUE,  MAT_RED  }, /*  9 */
    {MAT_BLACK,  MAT_RED,   MAT_BLUE,  MAT_GREEN, MAT_WHITE}, /* 10 */
    {MAT_RED,    MAT_BLUE,  MAT_GREEN, MAT_BLACK, MAT_WHITE}, /* 11 */
    {MAT_GREEN,  MAT_RED,   MAT_BLACK, MAT_BLUE,  MAT_WHITE}, /* 12 */
    {MAT_WHITE,  MAT_RED,   MAT_BLUE,  MAT_GREEN, MAT_BLACK}, /* 13 */
    {MAT_RED,    MAT_GREEN, MAT_WHITE, MAT_BLUE,  MAT_BLACK}, /* 14 */
    {MAT_BLUE,   MAT_WHITE, MAT_GREEN, MAT_RED,   MAT_BLACK}, /* 15 */
    {MAT_GREEN,  MAT_BLUE,  MAT_RED,   MAT_WHITE, MAT_BLACK}  /* 16 */
};

/**
 * @brief 解析任务一二维码：ASCII十进制整数1~16。
 *
 * 二维码编号直接查表得到 A/B/C/D/E 五个圆对应的颜色。
 * 除数字前后的空白外，不接受其他字符或旧版颜色字符串格式。
 */
static uint8_t ApplyTask1QrId(uint8_t id)
{
    if ((id == 0U) || (id > 16U))
    {
        memset(s_target_by_position, 0, sizeof(s_target_by_position));
        return 0U;
    }

    memcpy(s_target_by_position,
           s_task1_qr_map[id - 1U],
           sizeof(s_target_by_position));

    return ValidateTask1TargetColors();
}

static uint8_t ApplyRandomQrTask(uint8_t task2)
{
    uint32_t r = Route_RandomNext();

    if (task2 != 0U)
    {
        return ApplyTask2QrId((uint8_t)(r % 6UL) + 1U);
    }

    return ApplyTask1QrId((uint8_t)(r % 16UL) + 1U);
}

static uint8_t ParseTask1Qr(const char *text)
{
    uint8_t id;

    if (ParseAsciiId(text, 16U, &id) == 0U)
    {
        memset(s_target_by_position, 0, sizeof(s_target_by_position));
        return 0U;
    }

    return ApplyTask1QrId(id);
}

/**
 * @brief 按超时和重试次数读取二维码。
 * @param task2 1=任务二奖杯二维码，0=任务一物料二维码。
 */
static uint8_t ReadQrWithRetry(uint8_t task2)
{
#if COMP_QR_BYPASS_RANDOM
    /*
     * QR断线临时模式：
     * 不访问MJ6000、不清USART2、不等待二维码超时。
     * 直接随机抽取现有合法任务表中的一项。
     */
    return ApplyRandomQrTask(task2);
#else
#if COMP_ENABLE_TASK2_QR || COMP_ENABLE_TASK1_QR
    for (uint8_t attempt = 0U; attempt < QR_READ_RETRY_COUNT; attempt++)
    {
        char code[96];
        uint8_t result;

        AppControl_Service();
        if (AppControl_StopRequested() != 0U) return 0U;

        memset(code, 0, sizeof(code));

        /*
         * 第一次读取不清RX：
         * 去二维码点之前已经清过一次旧缓存，接近过程中收到的第一帧
         * 必须保留下来，避免到点后把刚扫到的有效二维码自己清掉。
         * 只有前一次读取/解析失败后，重试才清残帧。
         */
        if (attempt != 0U)
        {
            MJ6000_ClearRx();
        }

        result = MJ6000_ReadCode(code, sizeof(code), QR_READ_TIMEOUT_MS);

        if (result == MJ6000_OK)
        {
            uint8_t parsed =
                task2 ? ParseTask2Qr(code) : ParseTask1Qr(code);
            if (parsed != 0U) return 1U;
        }

        if (AppControl_Delay(300U) == 0U) return 0U;
    }

    return 0U;
#else
    (void)task2;
    return 0U;
#endif
#endif
}

static char GetTrophyForPodium(PodiumRank_t podium)
{
    if (s_trophy_qr.valid == 0U) return '\0';
    if (s_trophy_qr.trophy_a == podium) return 'A';
    if (s_trophy_qr.trophy_b == podium) return 'B';
    if (s_trophy_qr.trophy_c == podium) return 'C';
    return '\0';
}

static int8_t FindTrophySlot(char trophy_id)
{
    if ((trophy_id >= 'a') && (trophy_id <= 'z'))
    {
        trophy_id = (char)(trophy_id - ('a' - 'A'));
    }

    for (uint8_t slot = 0U; slot < 3U; slot++)
    {
        if (s_trophy_id_by_slot[slot] == trophy_id) return (int8_t)slot;
    }
    return -1;
}

/* ============================= Pick / place ============================= */

/*
 * 正式比赛放置统一原则：
 * K230连续“新5帧中位数 -> 限幅移动 -> 停稳 -> 新5帧复核”，直到进入窗口；
 * 复核通过后PD15选料，执行固定机械补偿；
 * 放好后保持当前车头直退100 mm，再按黑十字车体中心安全通道去下一点。
 */
static void Route_SortFloat(float *values, uint8_t count)
{
    uint8_t i, j;
    if (values == NULL) return;

    for (i = 0U; i < count; i++)
    {
        for (j = (uint8_t)(i + 1U); j < count; j++)
        {
            if (values[j] < values[i])
            {
                float temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
}

static float s_k230_last_x = 0.0f;
static float s_k230_last_y = 0.0f;

/* 连续取新的K230数据帧，帧数由K230_MEDIAN_FRAME_COUNT决定。 */
static uint8_t K230_ReadMedian5(float *x, float *y)
{
#if COMP_ENABLE_K230
    K230_Result_t result;
    float xs[K230_MEDIAN_FRAME_COUNT];
    float ys[K230_MEDIAN_FRAME_COUNT];
    uint8_t count = 0U;
    uint32_t start_ms;

    if ((x == NULL) || (y == NULL)) return 0U;

    /*
     * K230加速版（不修改任何competition_config.h参数）：
     *
     * 原代码进入本函数后会先把当前已经到达的一帧NEW结果直接丢掉，
     * 然后才开始等待K230_MEDIAN_FRAME_COUNT个新帧。
     * 对准过程会多次调用本函数，因此每轮都白等一个摄像头帧周期。
     *
     * 现在先处理一次K230，并把“此刻确实是NEW且valid”的结果直接作为
     * 本轮中位数的第1帧；随后仍继续收满原来的
     * K230_MEDIAN_FRAME_COUNT（当前仍是3帧）。
     *
     * 没有修改：
     * - K230_MEDIAN_FRAME_COUNT
     * - K230_VERIFY_TOLERANCE_X/Y
     * - K230_PLACE_SETTLE_MS
     * - K230_FRAME_TIMEOUT_MS
     * - K230_PLACE_SPEED
     * - 任何路线/点位/PD15/Yaw参数
     */
    K230_Process();
    if ((K230_GetNewResult(&result) != 0U) &&
        (result.valid != 0U))
    {
        xs[count] = result.center_x;
        ys[count] = result.center_y;
        count++;
    }

    start_ms = HAL_GetTick();

    while (((HAL_GetTick() - start_ms) < K230_FRAME_TIMEOUT_MS) &&
           (count < K230_MEDIAN_FRAME_COUNT))
    {
        AppControl_Service();
        if (AppControl_StopRequested() != 0U) return 0U;

        K230_Process();
        if ((K230_GetNewResult(&result) != 0U) &&
            (result.valid != 0U))
        {
            xs[count] = result.center_x;
            ys[count] = result.center_y;
            count++;
        }

        HAL_Delay(2U);
    }

    /*
     * 只要本轮确实收到过有效圆心，就说明圆仍然存在。
     *
     * 优先仍然是收满K230_MEDIAN_FRAME_COUNT帧后取中位数；
     * 但移动后如果因为帧率/串口时序只收到1~2个valid新帧，
     * 不再把它误判成“完全没看到圆”，从而触发大范围前后搜圆。
     *
     * count=1：直接使用这一帧；
     * count=2：排序后取较大下标这一帧；
     * count>=3：保持原来的中位数处理。
     */
    if (count == 0U) return 0U;

    Route_SortFloat(xs, count);
    Route_SortFloat(ys, count);
    *x = xs[count / 2U];
    *y = ys[count / 2U];
    s_k230_last_x = *x;
    s_k230_last_y = *y;
    return 1U;
#else
    (void)x;
    (void)y;
    return 1U;
#endif
}

static float K230_MinExecutableMove(float mm)
{
    if (fabsf(mm) < 0.5f) return 0.0f;
    return (mm > 0.0f) ? (mm + 8.5f) : (mm - 8.5f);
}

/*
 * K230对圆诊断，仅打印整数，避免printf/snprintf增大链接体积。
 *
 * #KA 108,073  = 本轮5帧中位数圆心约(108,73)
 * #KM +015,-024 = 本轮实际发送给底盘的车体修正约右15mm、后24mm
 *
 * 只加日志，不改变K230原有控制、增益、速度、容差或移动算法。
 */
/* 5帧中位数 -> 限幅XY修正 -> 停稳取新帧循环；失败则禁止盲放。 */
static uint8_t K230SearchLongitudinal(float *x, float *y)
{
    if ((x == NULL) || (y == NULL)) return 0U;

    /*
     * PRE点完全看不到圆时，快速搜索：
     *
     *   当前位置已经由调用者确认“没找到圆”；
     *   1. 先后退50mm，到后方5cm位置重新识别；
     *   2. 如果找到圆，立刻return 1，马上继续后面的K230对中/放料流程；
     *   3. 如果仍没找到，从后方5cm直接前进100mm，
     *      到原PRE点前方5cm位置重新识别；
     *   4. 找到后同样立刻return 1，不再做额外搜索。
     *
     * 这样实际搜索范围是“原PRE点后方5cm ~ 前方5cm”，
     * 并取消原来每20mm停一次、反复拍圆的慢速搜索。
     */

    /* 先到原PRE点后方5cm。 */
    if (Chassis_MoveMMPositionBlocking(0.0f, -50.0f,
                                       78U, 35U, 2500U) == false)
    {
        return 0U;
    }

    /* 后方5cm找到圆：立即结束搜索，执行下一步。 */
    if (K230_ReadMedian5(x, y) != 0U)
    {
        return 1U;
    }

    /*
     * 后方没找到：直接越过原PRE点，到前方5cm。
     * 当前在-50mm，所以这里前进100mm后位于+50mm。
     */
    if (Chassis_MoveMMPositionBlocking(0.0f, 100.0f,
                                       78U, 35U, 3000U) == false)
    {
        return 0U;
    }

    /* 前方5cm找到圆：立即结束搜索，执行下一步。 */
    if (K230_ReadMedian5(x, y) != 0U)
    {
        return 1U;
    }

    /*
     * 后方5cm和前方5cm都没有找到圆：
     * 回到本轮搜索开始时的中心位置，然后交给外层继续下一轮搜圆。
     *
     * 返回值约定：
     *   1 = 已重新找到圆；
     *   2 = 本轮前后搜索完成但仍未找到圆，位置已回到搜索中心；
     *   0 = 底盘移动失败/搜索流程无法继续。
     *
     * 注意：这里沿用现有的50mm、78U、35U、2500U参数，
     * 不修改competition_config.h中的任何参数。
     */
    if (Chassis_MoveMMPositionBlocking(0.0f, -50.0f,
                                       78U, 35U, 2500U) == false)
    {
        return 0U;
    }

    Chassis_Stop();
    return 2U;
}

static uint8_t K230AlignCameraOnce(uint8_t use_task1_position_mode)
{
#if COMP_ENABLE_K230

		const uint8_t max_iter = 8U;          // 原12，最多5轮
		const float max_step_far_mm = 55.0f;  // 原25，一次最多走50mm
		const float max_step_near_mm = 25.0f; // 原12
		const float near_pixel = 10.0f;       // 原12
		const float near_gain = 0.85f;        // 原0.55
		const float fine_min_move_mm = 3.0f;  // 原2.5


    uint8_t iter;
    uint8_t target_acquired = 0U;

    for (iter = 0U; iter < max_iter; ++iter)
    {
        float x, y, dx, dy;
        float right_mm, forward_mm;
        float gain = 1.0f;
        float max_step_mm = max_step_far_mm;
        float yaw_error_deg;
        uint8_t near_target = 0U;

        if (K230_ReadMedian5(&x, &y) == 0U)
        {
            uint8_t search_result;
            uint8_t reacquired = 0U;
            uint8_t retry;

            Chassis_Stop();

            /*
             * 已经识别并锁定过圆以后，移动后的短暂0帧优先认为是
             * 摄像头/串口新帧还没跟上，而不是圆真的丢了。
             *
             * 先保持当前位置连续重读，不做任何前后移动；
             * 只有连续重读仍然完全收不到有效圆心，才退回原有
             * “后退50mm -> 前进100mm”的搜索流程。
             *
             * 第一次从未见过圆时仍直接执行原有大范围搜索，
             * 所以PRE点找圆能力和所有运动参数保持不变。
             */
            if (target_acquired != 0U)
            {
                for (retry = 0U; retry < 3U; ++retry)
                {
                    if (K230_ReadMedian5(&x, &y) != 0U)
                    {
                        reacquired = 1U;
                        break;
                    }

                    if (AppControl_StopRequested() != 0U)
                    {
                        Chassis_Stop();
                        return 0U;
                    }
                }
            }

            if (reacquired == 0U)
            {
                search_result = K230SearchLongitudinal(&x, &y);

                if (search_result == 0U)
                {
                    return 0U;
                }

                if (search_result == 2U)
                {
                    continue;
                }
            }

            s_k230_last_x = x;
            s_k230_last_y = y;
        }

        /*
         * 能走到这里说明本轮已经拿到有效圆心。
         * 从这一刻起，后续短暂丢帧先原地恢复，不再立刻大范围搜圆。
         */
        target_acquired = 1U;

        dx = x - K230_TARGET_CENTER_X;
        dy = K230_TARGET_CENTER_Y - y;

        yaw_error_deg = Route_NormalizeAngle(
            MAP_HOME_YAW_DEG - Chassis_GetCurrentMapYawDeg());

        /*
         * 只有“圆心在最终窗口 + Yaw也在最终窗口”才能成功。
         */
        if ((fabsf(dx) <= K230_VERIFY_TOLERANCE_X) &&
            (fabsf(dy) <= K230_VERIFY_TOLERANCE_Y))
        {
						if (fabsf(yaw_error_deg) > K230_PLACE_YAW_TOLERANCE_DEG)
						{
								if (Route_RotateToPrecise(MAP_HOME_YAW_DEG) == 0U)
								{
										/*
										 * 精确Yaw偶发失败：
										 * 不退出整个放料任务。
										 * 停车后重新拍K230继续尝试。
										 */
										Chassis_Stop();
										continue;
								}

								/*
								 * 回0°后位置可能轻微变化，
								 * 直接重新拍圆，不再额外原地等待。
								 */
								continue;
						}

						Chassis_Stop();
						return 1U;

        }

        

        if ((fabsf(dx) <= near_pixel) && (fabsf(dy) <= near_pixel))
        {
            near_target = 1U;
            gain = near_gain;
            max_step_mm = max_step_near_mm;
        }

        right_mm = (K230_MAP_R_DX * dx + K230_MAP_R_DY * dy) * gain;
        forward_mm = (K230_MAP_F_DX * dx + K230_MAP_F_DY * dy) * gain;

        /*
         * 某一轴已经进最终窗口，就不再修这一轴；
         * 另一轴还没进窗口就必须继续修，不能因为毫米命令太小被清零。
         */
        if (fabsf(dx) <= K230_VERIFY_TOLERANCE_X)
        {
            right_mm = 0.0f;
        }
        if (fabsf(dy) <= K230_VERIFY_TOLERANCE_Y)
        {
            forward_mm = 0.0f;
        }

        right_mm = Route_ClampFloat(right_mm, -max_step_mm, max_step_mm);
        forward_mm = Route_ClampFloat(forward_mm, -max_step_mm, max_step_mm);

        if ((fabsf(dx) > K230_VERIFY_TOLERANCE_X) &&
            (fabsf(right_mm) < fine_min_move_mm))
        {
            right_mm = (right_mm >= 0.0f) ?
                       fine_min_move_mm : -fine_min_move_mm;
        }

        if ((fabsf(dy) > K230_VERIFY_TOLERANCE_Y) &&
            (fabsf(forward_mm) < fine_min_move_mm))
        {
            forward_mm = (forward_mm >= 0.0f) ?
                         fine_min_move_mm : -fine_min_move_mm;
        }

        /*
         * 极少数情况下二维映射交叉项互相抵消导致命令接近0：
         * 按原始像素误差方向补一个最小动作，避免“看到圆但不动”。
         */
        if ((right_mm == 0.0f) && (forward_mm == 0.0f))
        {
            if (fabsf(dx) >= fabsf(dy))
            {
                right_mm = (dx >= 0.0f) ?
                           fine_min_move_mm : -fine_min_move_mm;
            }
            else
            {
                forward_mm = (dy >= 0.0f) ?
                             fine_min_move_mm : -fine_min_move_mm;
            }
        }

        if (near_target != 0U)
        {
            /*
             * 近圆阶段统一走带HWT101锁Yaw的短距离闭环。
             * 这样2.5~12mm的小动作也能真正执行，并且不会为了微调把车头扭歪。
             */
            if (Chassis_MoveMMFinalHoldYawBlocking(
                    right_mm,
                    forward_mm,
                    130U,
                    1800U) == false)
            {
                Chassis_Stop();
                continue;
            }
        }
        else if (use_task1_position_mode != 0U)
        {
            if (Chassis_MoveMMPositionBlocking(right_mm,
                                               forward_mm,
                                               230U, 100U, 2500U) == false)
            {
                Chassis_Stop();
                continue;
            }
        }
        else
        {
            /*
             * 奖杯远距离视觉修正仍保留原来的低速带Yaw闭环移动。
             */
            right_mm = K230_MinExecutableMove(right_mm);
            forward_mm = K230_MinExecutableMove(forward_mm);

            if (Chassis_MoveMMBlocking(right_mm,
                                       forward_mm,
                                       K230_PLACE_SPEED,
                                       COMP_MOVE_ACC,
                                       5000U) == false)
            {
                Chassis_Stop();
                return 0U;
            }
        }

        yaw_error_deg = Route_NormalizeAngle(
            MAP_HOME_YAW_DEG - Chassis_GetCurrentMapYawDeg());

        if (fabsf(yaw_error_deg) > K230_PLACE_YAW_TOLERANCE_DEG)
        {
            if (Route_RotateToPrecise(MAP_HOME_YAW_DEG) == 0U)
            {
                Chassis_Stop();
                return 0U;
            }
        }

        if (AppControl_Delay(K230_PLACE_SETTLE_MS) == 0U)
        {
            Chassis_Stop();
            return 0U;
        }

        /* 下一轮重新读取全新K230帧。 */
    }

    Chassis_Stop();
    return 0U;
#else
    (void)use_task1_position_mode;
    return 1U;
#endif
}

/* 视觉复核通过后不再看圆，只执行槽位中心的固定机械补偿。 */
static uint8_t K230FinalPlacePush(void)
{
    return (Chassis_MoveMMFinalHoldYawBlocking(
                K230_PLACE_FIXED_RIGHT_MM,
                K230_PLACE_FIXED_FORWARD_MM,
                K230_PLACE_SPEED,
                2500U) != false) ? 1U : 0U;
}

/* 放好后的强制退出：保持当前航向，直着往后100mm。 */
static uint8_t RetreatAfterPlacement(float retreat_mm)
{
    return Route_MovePosition(0.0f, -retreat_mm,
                              POS_RETREAT_RPM, POS_RETREAT_ACC);
}

/* 亚军/冠军/季军共用：K230 -> PD15选杯 -> 放置 -> 直退100。 */



/*
 * 奖杯高位/低位共用的“视觉对中”阶段：
 * PD15先选对应奖杯 -> 精确Yaw=0° -> K230找圆并把车移动到圆心。
 * 成功返回后，调用方可以只动升降轴下降；期间不再转PD15、不再旋转底盘。
 */
static uint8_t AlignTrophyAtCurrentHeight(PodiumRank_t podium)
{
    char trophy_id = GetTrophyForPodium(podium);
    int8_t slot = FindTrophySlot(trophy_id);

    if (slot < 0) return 0U;

    if (TurnableAlignTask2PutBlocking((uint8_t)slot) == 0U) return 0U;

    if (Route_RotateToPrecise(MAP_HOME_YAW_DEG) == 0U) return 0U;

    if (K230AlignCameraOnce(0U) == 0U) return 0U;

    return 1U;
}

/*
 * K230已经对中后的最终放杯收尾：
 * do_final_push=1：先执行现有固定前送，再停留、直退（季军原流程）。
 * do_final_push=0：前送已经在高位提前完成，只停留、直退。
 *
 * 这样亚军/冠军可以：
 *   高位K230对中 -> 高位前送 -> 只降升降轴 -> 收尾直退，
 * 避免低位前送时后面的奖杯撞奖台。
 */
static uint8_t PlaceAlignedTrophy(PodiumRank_t podium,
                                  uint8_t do_final_push)
{
    char trophy_id = GetTrophyForPodium(podium);
    int8_t slot = FindTrophySlot(trophy_id);
    uint8_t result;

    if (slot < 0) return 0U;

    if (do_final_push != 0U)
    {
        if (K230FinalPlacePush() == 0U) return 0U;
    }

    if (AppControl_Delay(PLACE_DWELL_MS) == 0U) return 0U;

		if (podium == PODIUM_SECOND)
		{
				result = RetreatAfterPlacement(PLACE_RETREAT_SECOND_MM);
		}
		else if (podium == PODIUM_FIRST)
		{
				result = RetreatAfterPlacement(PLACE_RETREAT_FIRST_MM);
		}
		else
		{
				result = RetreatAfterPlacement(PLACE_RETREAT_MM);
		}

    Turnable_Put_Material_Task2((uint8_t)slot, 0U);
    return result;
}

/* 季军仍沿用原流程：当前高度直接选杯 -> Yaw -> K230 -> 前送放杯。 */
static uint8_t PlaceTrophy(PodiumRank_t podium)
{
    if (AlignTrophyAtCurrentHeight(podium) == 0U) return 0U;
    return PlaceAlignedTrophy(podium, 1U);
}

/* position: 0=A、1=B、2=C、3=D、4=E。 */
/*
 * 任务一单点放置失败阶段：
 * 1=K230对圆
 * 2=PD15选槽
 * 3=固定前送
 * 4=放置等待/中止
 * 5=后退
 * V2取消阶段6：退车后PD15不再转动。
 */
static uint8_t s_t1_place_fail_stage = 0U;

static const float s_t1_slot_place_right_mm[5] =
{
    T1_SLOT0_PLACE_RIGHT_MM,
    T1_SLOT1_PLACE_RIGHT_MM,
    T1_SLOT2_PLACE_RIGHT_MM,
    T1_SLOT3_PLACE_RIGHT_MM,
    T1_SLOT4_PLACE_RIGHT_MM
};

static const float s_t1_slot_place_forward_mm[5] =
{
    T1_SLOT0_PLACE_FORWARD_MM,
    T1_SLOT1_PLACE_FORWARD_MM,
    T1_SLOT2_PLACE_FORWARD_MM,
    T1_SLOT3_PLACE_FORWARD_MM,
    T1_SLOT4_PLACE_FORWARD_MM
};

static const float s_t1_ring_x_mm[5] =
{
    MAP_PLACE_A_RING_X_MM,
    MAP_PLACE_B_RING_X_MM,
    MAP_PLACE_C_RING_X_MM,
    MAP_PLACE_D_RING_X_MM,
    MAP_PLACE_E_RING_X_MM
};

static const float s_t1_ring_y_mm[5] =
{
    MAP_PLACE_A_RING_Y_MM,
    MAP_PLACE_B_RING_Y_MM,
    MAP_PLACE_C_RING_Y_MM,
    MAP_PLACE_D_RING_Y_MM,
    MAP_PLACE_E_RING_Y_MM
};

/*
 * K230已经把圆心锁进窗口后，这个圆就是比编码器更可靠的绝对地图地标。
 * 用“圆心坐标 - 当前物理槽最终机械补偿”重新锚定车体X/Y，
 * HWT101地图航向零点保持不变。
 *
 * 这样A/B视觉对准成功后，去C不再继续背着前面麦轮滑移造成的地图误差。
 */


/*
 * 放料并按已知距离退离后，再用同一个圆心做一次绝对XY锚定。
 *
 * K230对准 + 最终机械前送完成时：
 *   车体中心 = (ring_x, ring_y - ROBOT_CENTER_TO_PICKUP_SLOT_MM)
 * 再沿Yaw=0°后退retreat_mm：
 *   车体中心 = (ring_x,
 *               ring_y - ROBOT_CENTER_TO_PICKUP_SLOT_MM - retreat_mm)
 *
 * 这样去下一个圆和最终回Home都从真实地标重新开始，
 * 不再累计前面麦轮打滑/短距离动作的里程误差。
 */



/*
 * K230已经确认圆以后，Task1最后35mm量级机械前送使用更宽裕的阻塞超时。
 * 距离、方向、速度、加速度全部保持原值，只避免小距离闭环被通用超时误杀。
 */
static uint8_t Task1FinalPlacePush(int8_t slot)
{
    float right_mm;
    float forward_mm;

    if ((slot < 0) || (slot >= 5)) return 0U;

    right_mm = s_t1_slot_place_right_mm[(uint8_t)slot];
    forward_mm = s_t1_slot_place_forward_mm[(uint8_t)slot];

    /*
     * 最终2.5/35mm继续锁定K230确认时的0°航向，
     * 防止最后一小段又把车头扭偏。
     */
    if (Chassis_MoveMMFinalHoldYawBlocking(right_mm,
                                           forward_mm,
                                           130U,
                                           1800U) == false)
    {
        Chassis_Stop();
        return 0U;
    }

    return 1U;
}

/* 蓝牙打印：#TP A0 表示A圆本次实际使用物理槽0。 */


/*
 * A/B/C/D/E放完后，当前空槽在转场期间保持不动。
 * 到下一个圆粗点停车后，先精确Yaw并由K230完成找圆定位；
 * 车体完全定好位置以后，才把本次目标物料转到开口。
 */

static uint8_t PlaceMaterialAtPositionWithRetreat(uint8_t position,
                                                  float retreat_mm)
{
    GY33_CustomColor_t color;
    int8_t slot = -1;
    uint8_t result;

    s_t1_place_fail_stage = 0U;

    if (position >= 5U) return 0U;
    if (s_target_by_position[position] == MAT_UNKNOWN) return 0U;

    color = MaterialColorToGy33(s_target_by_position[position]);
    if (color == GY33_COLOR_UNKNOWN) return 0U;

    /*
     * A~E放料安全顺序：
     * 到粗点 -> 底盘停稳 -> 精确Yaw
     * -> K230识圆并完成全部车体位置微调
     * -> 底盘完全停止
     * -> 最后才PD15把目标物料槽转到外开口
     * -> 固定前送 -> 放料 -> 完整后退。
     *
     * 这样K230修正车体时，物料不会提前停在外开口处，
     * 避免车体移动时把物料晃出。
     */
    Chassis_Stop();

    /*
     * 不再额外白等50ms：
     * 下一步Route_RotateToPrecise本身是阻塞到位动作。
     */
    /* 先精确回到放料Yaw。 */
    if (Route_RotateToPrecise(MAP_TASK1_PLACE_YAW_DEG) == 0U)
    {
        s_t1_place_fail_stage = 1U;
        return 0U;
    }

    /*
     * PD15保持当前安全状态不动，
     * 先让K230完成识圆和全部车体位置修正。
     */
    if (K230AlignCameraOnce(1U) == 0U)
    {
        s_t1_place_fail_stage = 1U;
        return 0U;
    }

    /*
     * K230阻塞移动结束时底盘已经停止；
     * 这里不再额外白等50ms，直接开始PD15选槽。
     */
    Chassis_Stop();

    /*
     * 最后才按A~E目标颜色找到实际物理槽，
     * 把对应物料转到外开口。
     * 从这里开始不再执行K230修正或车体旋转。
     */
    if (TurnableAlignTask1PutBlocking(color, &slot) == 0U)
    {
        s_t1_place_fail_stage = 2U;
        return 0U;
    }

    if (Task1FinalPlacePush(slot) == 0U)
    {
        s_t1_place_fail_stage = 3U;
        return 0U;
    }

    if (AppControl_Delay(MATERIAL_PLACE_DWELL_MS) == 0U)
    {
        s_t1_place_fail_stage = 4U;
        return 0U;
    }

    /*
     * 先完整后退。后退期间不触碰任何PD15 put状态，
     * 让当前已经放空的槽保持正对外开口。
     */
    result = RetreatAfterPlacement(retreat_mm);
    if (result == 0U)
    {
        s_t1_place_fail_stage = 5U;
        return 0U;
    }

    Chassis_Stop();

    /*
     * 必须等后退完全结束以后才更新软件记录。
     * 下面两个调用本身不会发送新的PD15角度命令。
     */
    if (slot >= 0)
    {
        Turnable_MarkSlotEmpty((uint8_t)slot);
    }
    Turnable_Put_Material_By_Color_Task1(color, 0U);

    /* A->B->C->D->E转场期间PD15保持不动。 */
    return 1U;
}




/* ============================= Line collection ============================= */

typedef enum
{
    LINE_MISSION_TASK2 = 0,
    LINE_MISSION_TASK1
} LineMission_t;

typedef struct
{
    float x_mm;
    float y_mm;
} LinePickupObject_t;

/*
 * 这些是赛道地图中的“物体中心”，不是人为调出来的巡线Yaw阈值。
 * 任务二沿右侧黑线按 C -> B -> A 收三个奖杯；任务一沿左侧黑线
 * 按 M1 -> M2 -> M3 -> M4 -> M5 收五个圆柱。
 */
static const LinePickupObject_t s_task2_pickups[3] =
{
    {MAP_TROPHY_C_OBJECT_X_MM, MAP_TROPHY_C_OBJECT_Y_MM},
    {MAP_TROPHY_B_OBJECT_X_MM, MAP_TROPHY_B_OBJECT_Y_MM},
    {MAP_TROPHY_A_OBJECT_X_MM, MAP_TROPHY_A_OBJECT_Y_MM}
};

/*
 * 任务一M1~M5固定捕获进度角。
 *
 * 由图纸物料中心 + 左巡线圆心(1240,1220) + 巡线半径920 mm
 * + 车体中心到外开口槽位中心300 mm计算：
 *
 * 槽位前向机械补偿角 = atan(300/920) ≈ 18.06°
 *
 * M7人工巡线入口MapYaw=90°，因此五个捕获进度约为：
 * 原开口触发：M1 11.73° / M2 40.74° / M3 70.44° / M4 100.49° / M5 130.62°
 * 实车要求物料再深入槽位45 mm，因此先统一延后约2.80°，本次实车再统一延后约2.49°（40 mm）。
 *
 * 不再使用巡线过程中会累计漂移的MapX/MapY决定任务一捕获时刻。
 */
static const float s_task1_pick_progress_deg[5] =
{
    /*
     * 任务一现在改成“从下往上”收料，沿左圆顺时针运行。
     *
     * 旧路线从上往下（M1->M5）时，HWT101累计方向为+1；
     * 新路线从下往上（M5->M1）时，HWT101累计方向必须改为-1。
     *
     * 仍按左巡线圆心(1240,1220)、巡线半径920mm、车体中心到开口300mm，
     * 并保留原实车已经验证的统一机械延后补偿约5.29°重新计算：
     *
     * 第1个=底部M5 : 18.56°
     * 第2个=M4     : 48.67°
     * 第3个=M3     : 78.72°
     * 第4个=M2     : 108.42°
     * 第5个=顶部M1 : 137.44°
     *
     * 这样颜色识别仍严格跟在“物料进入50mm -> PD15转72°”之后，
     * 不再使用旧M7上方入口的17.02/46.03/...角度。
     */
    18.56f, 48.67f, 78.72f, 108.42f, 137.44f
};

static const LinePickupObject_t *Line_GetTask2PickupObject(uint8_t index)
{
    if (index >= 3U) return NULL;
    return &s_task2_pickups[index];
}

/**
 * @brief 计算圆盘外开口“槽位中心”在地图中的实时坐标。
 *
 * 用户最新实测：槽位中心距车体中心300 mm。圆盘最前缘333 mm仍只用于
 * 外形/避障，不能再拿来判断进料时刻。
 */
static uint8_t Line_GetDiscOpeningMapXY(float *x_mm, float *y_mm)
{
    ChassisPose_t pose;
    float sin_yaw;
    float cos_yaw;

    if ((x_mm == NULL) || (y_mm == NULL)) return 0U;

    pose = Chassis_GetPose();
    if (pose.valid == false) return 0U;

    /*
     * 仅任务二继续使用原有MapXY几何捕获。
     * 任务一已经改为HWT101固定进度角，不再走这里。
     */
    Chassis_SinCosDeg(pose.yaw_deg, &sin_yaw, &cos_yaw);
    *x_mm = pose.x_mm - ROBOT_CENTER_TO_PICKUP_SLOT_MM * sin_yaw;
    *y_mm = pose.y_mm + ROBOT_CENTER_TO_PICKUP_SLOT_MM * cos_yaw;
    return 1U;
}

/*
 * Compact map pickup geometry.
 *
 * For target vector T and opening vector O around the same line center:
 * sin(target_angle-opening_angle) =
 *     (Ty*Ox - Tx*Oy) / (|T|*|O|)
 *
 * The dot product rejects the opposite half-circle ambiguity.  The actual
 * pickup/prepare windows are all <=20 degrees, where signed sine is monotonic.
 */
#define LINE_PICK_CAPTURE_SIN   	  0.17364818f
#define LINE_PICK_CLEAR_SIN         0.09584575f
#define LINE_PICK_MISSED_SIN        0.17364818f
#define LINE_NEXT_SLOT_SLOW_SIN     0.34202014f
#define LINE_NEXT_SLOT_STOP_SIN     0.10452846f

/*
 * Task2高速巡线专用“提前切下一槽”触发角。
 * sin(15°)=0.25881905。
 * sin(16.5°)=0.28401534。
 * sin(17°)=0.25881905。
 * sin(17.5°)=0.30070580。 
 *
 * 原来即使 T2_TROPHY_SLOT_SWITCH_DELAY_MM=0，仍要等待
 * Turnable_Get_Material_Task2() 内部 flag_task2_get_done 才真正推进下一槽，
 * 高速巡线时就会拖到接近第二个奖杯才转。
 *
 * 现在第1、2个奖杯在约+20°进入窗口后，直接下发下一槽开口角，
 * 不再等待旧done握手；第三个奖杯仍保留原来的完成/安全位逻辑。
 *
 * 如果实车仍嫌晚，只改这里：
 *   18° -> 0.30901699f
 *   19° -> 0.32556815f 
 *   20° -> 0.34202014f
 */
#define T2_EARLY_NEXT_SLOT_CAPTURE_SIN  0.30070580f

static float Line_GetTask2PickupAngleSin(uint8_t index)
{
    const LinePickupObject_t *object = Line_GetTask2PickupObject(index);
    float opening_x;
    float opening_y;
    float tx;
    float ty;
    float ox;
    float oy;
    float dot;
    float cross;
    float denom;

    if ((object == NULL) ||
        (Line_GetDiscOpeningMapXY(&opening_x, &opening_y) == 0U))
    {
        return 1.0f;
    }

    tx = object->x_mm - MAP_LINE_RIGHT_CENTER_X_MM;
    ty = object->y_mm - MAP_LINE_CENTER_Y_MM;
    ox = opening_x - MAP_LINE_RIGHT_CENTER_X_MM;
    oy = opening_y - MAP_LINE_CENTER_Y_MM;

    dot = tx * ox + ty * oy;
    cross = ty * ox - tx * oy;

    if (dot <= 0.0f)
    {
        return (cross >= 0.0f) ? 1.0f : -1.0f;
    }

    denom = sqrtf((tx * tx + ty * ty) * (ox * ox + oy * oy));
    if (denom < 1.0f)
    {
        return 1.0f;
    }

    return cross / denom;
}

/**
 * @brief 停车确认八路模块连续检测到黑线。
 *
 * 连续多次在线才确认压线成功，防止单次干扰误判。
 */
static uint8_t ConfirmLineAtEntry(void)
{
    uint32_t start = HAL_GetTick();
    uint8_t count = 0U;

    Chassis_Stop();

    while ((HAL_GetTick() - start) < LINE_ENTRY_CONFIRM_MS)
    {
        AppControl_Service();
        if (AppControl_StopRequested() != 0U)
        {
            Chassis_Stop();
            return 0U;
        }
        LineSensor_t line;
        LineSensor_Read(&line);

        if (line.online != 0U)
        {
            count++;
            if (count >= LINE_ENTRY_CONFIRM_COUNT)
            {
                return 1U;
            }
        }
        else
        {
            count = 0U;
        }

        HAL_Delay(20U);
    }

    return 0U;
}

/**
 * @brief 保持当前航向直行，直到八路模块连续检测到黑线。
 *
 * 用于任务二二维码之后“不是死走固定距离，而是一直走到看见黑线”。
 * 行驶期间持续更新四轮编码器里程计；超过最大距离或超时立即停车。
 */



/**
 * @brief 八路寻迹 PD 控制。
 *
 * 在线时根据加权误差修正旋转速度；丢线时按上次偏差方向低速找线。
 */
static void LineApplyControl(const LineSensor_t *line,
                             float *last_error,
                             uint32_t now_ms,
                             uint32_t *last_line_seen_ms,
                             float forward_speed)
{
    float turn;

    if ((line != NULL) && (line->online != 0U))
    {
        float derivative = (float)line->error - *last_error;
        *last_line_seen_ms = now_ms;
        turn = LINE_STEER_SIGN *
               (LINE_KP * (float)line->error + LINE_KD * derivative);
        turn = Route_ClampFloat(turn, -LINE_TURN_MAX, LINE_TURN_MAX);
        *last_error = (float)line->error;
        Chassis_SetSpeed(0.0f, forward_speed, turn);
    }
    else
    {
        /*
         * 八路模块在黑线边缘可能偶发1~2帧online=0。
         * 旧逻辑每丢1帧就立即Chassis_Stop()，下一帧重新上线又立即前进，
         * 会把正常传感器抖动放大成“走一下、停一下”的卡顿。
         *
         * 这里不修改任何config参数：
         * 在主循环原有60ms丢线确认时间内，保持上一次纠偏方向低速连续前进；
         * 只有持续丢线超过确认时间，才真正停车并由主循环进入回退恢复。
         */
        const uint32_t transient_loss_grace_ms = 60U;
        uint32_t lost_ms = now_ms - *last_line_seen_ms;

        if (lost_ms <= transient_loss_grace_ms)
        {
            turn = LINE_STEER_SIGN * LINE_KP * (*last_error);
            turn = Route_ClampFloat(turn,
                                    -(LINE_TURN_MAX * 0.55f),
                                     (LINE_TURN_MAX * 0.55f));

            Chassis_SetSpeed(0.0f, forward_speed * 0.72f, turn);
        }
        else
        {
            Chassis_Stop();
        }
    }
}


/*
 * 丢线恢复：
 * 确认持续丢线后，按刚才离线时的运动趋势反向退回；
 * 倒退过程中持续读取八路传感器，一旦重新找到黑线立即停车，
 * 同步真实四轮里程计，然后回主巡线循环继续后续任务。
 */
static uint8_t LineRecoverBackToLine(float *last_error,
                                     uint32_t *last_line_seen_ms,
                                     float forward_speed)
{
    uint32_t start_ms;
    uint8_t phase = 0U;       /* 0=只倒退找线, 1=只向前重新骑线 */
    uint8_t seen_count = 0U;
    uint8_t stable_count = 0U;
    uint8_t lost_count = 0U;
    float recover_turn;
    const float recover_turn_max = LINE_TURN_MAX * 0.45f;
    const float reacquire_turn_max = LINE_TURN_MAX * 0.55f;
    const float reacquire_speed = 45.0f;

    if ((last_error == NULL) || (last_line_seen_ms == NULL))
    {
        Chassis_Stop();
        return 0U;
    }

    recover_turn = LINE_STEER_SIGN * LINE_KP * (*last_error);
    recover_turn = Route_ClampFloat(recover_turn,
                                    -recover_turn_max,
                                    recover_turn_max);
    start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < LINE_ACQUIRE_TIMEOUT_MS)
    {
        LineSensor_t line;

        AppControl_Service();
        if (AppControl_StopRequested() != 0U)
        {
            Chassis_Stop();
            return 0U;
        }

        LineSensor_Read(&line);

        if (phase == 0U)
        {
            /*
             * BACKTRACK：这个阶段绝不向前。
             *
             * 旧逻辑一看到线就立即Stop，但又要求连续3帧才切换；
             * 在线/离线边缘抖动时会形成“倒退->停->倒退->停”的明显卡动。
             *
             * 现在仍保留连续3帧确认，不改变原判定标准；
             * 只是确认期间继续用很小的倒退速度靠线，避免反复硬启停。
             */
            if (line.online != 0U)
            {
                if (seen_count < 3U) seen_count++;

                Chassis_SetSpeed(0.0f,
                                 -(forward_speed * 0.10f),
                                 -(recover_turn * 0.35f));

                if (seen_count >= 3U)
                {
                    phase = 1U;
                    stable_count = 0U;
                    lost_count = 0U;
                    *last_error = (float)line.error;
                    *last_line_seen_ms = HAL_GetTick();

                    /* 倒退切换为向前重新骑线时只停车一次。 */
                    Chassis_Stop();
                }
            }
            else
            {
                seen_count = 0U;
                Chassis_SetSpeed(0.0f,
                                 -(forward_speed * 0.35f),
                                 -recover_turn);
            }
        }
        else
        {
            /*
             * REACQUIRE：用45低速PD向前骑线。
             *
             * 旧逻辑在重新骑线阶段每丢1帧就Stop，连续5帧后又倒退，
             * 也会制造明显前后卡动。现在前4个短暂丢线帧继续低速向前，
             * 第5帧确认确实再次丢线后才停车并回BACKTRACK。
             */
            if (line.online != 0U)
            {
                float derivative = (float)line.error - *last_error;
                float turn = LINE_STEER_SIGN *
                             (LINE_KP * (float)line.error +
                              LINE_KD * derivative);

                turn = Route_ClampFloat(turn,
                                        -reacquire_turn_max,
                                        reacquire_turn_max);
                *last_error = (float)line.error;
                *last_line_seen_ms = HAL_GetTick();
                lost_count = 0U;

                Chassis_SetSpeed(0.0f, reacquire_speed, turn);

                if ((line.error >= -LINE_ACQUIRE_CENTER_ERROR) &&
                    (line.error <= LINE_ACQUIRE_CENTER_ERROR))
                {
                    if (stable_count < 255U) stable_count++;
                }
                else
                {
                    stable_count = 0U;
                }

                if (stable_count >= LINE_ACQUIRE_STABLE_COUNT)
                {
                    Chassis_Stop();
                    if (Chassis_OdometryUpdate() == false) return 0U;
                    *last_line_seen_ms = HAL_GetTick();
                    return 1U;
                }
            }
            else
            {
                float hold_turn;

                stable_count = 0U;
                if (lost_count < 5U) lost_count++;

                if (lost_count < 5U)
                {
                    hold_turn = LINE_STEER_SIGN * LINE_KP * (*last_error);
                    hold_turn = Route_ClampFloat(hold_turn,
                                                 -reacquire_turn_max,
                                                  reacquire_turn_max);
                    Chassis_SetSpeed(0.0f,
                                     reacquire_speed * 0.55f,
                                     hold_turn);
                }
                else
                {
                    Chassis_Stop();
                    phase = 0U;
                    seen_count = 0U;
                    lost_count = 0U;
                }
            }
        }

        HAL_Delay(LINE_CONTROL_PERIOD_MS);
    }

    Chassis_Stop();
    return 0U;
}


static uint8_t PrepareLinePickup(LineMission_t mission, uint8_t slot)
{
    if (mission == LINE_MISSION_TASK2)
    {
        return TurnablePrepareTask2GetBlocking(slot);
    }

    return TurnablePrepareTask1GetBlocking(slot);
}

/**
 * @brief 沿黑线行驶，并按地图物体坐标 + 实时四轮里程 + HWT101 + 圆盘开口位置触发收取。
 *
 * 任务二：逆时针收 C、B、A 三个奖杯；
 * 任务一：从左下方向上、沿左圆顺时针收 M5~M1 五个物料。
 *
 * 八路模块负责实时方向闭环；四轮编码器在寻迹中持续累计 X/Y。
 */
static uint8_t TurnableParkTask2ThirdOnLine(void)
{
    /*
     * 第三奖杯完整进入slot2后，不停车、不等待：
     * 巡线继续进行，同时PD15由196°转到232°槽间安全位。
     *
     * 这样奖杯会在“还沿黑线正常前进”的阶段就离开外部开口；
     * 到后面真正转Yaw/升圆盘/左移时，PD15已经不再动作。
     */
    Servo_Spin_360_SetAngle(
        (uint16_t)(task2_configs[2U].angle_opening +
                   TURNABLE_T2_TRANSPORT_INNER_OFFSET_DEG));

    return 1U;
}

static uint8_t TurnableParkTask1FifthOnLine(void)
{
    /*
     * 第五物料完整进入slot4后，把PD15从338°反向36°到302°槽间安全位。
     * 调用者紧接着停车并结束Task1巡线。
     */
    Servo_Spin_360_SetAngle(
        TURNABLE_TASK1_TRANSPORT_SAFE_ANGLE_DEG);

    return 1U;
}

static uint8_t FollowLineAndCollect(LineMission_t mission)
{
#if COMP_ENABLE_LINE_COLLECTION
    uint8_t pickup_total;
    int8_t direction;
    float stop_progress_deg;
    uint32_t start_ms;
    uint32_t last_line_seen_ms;
    uint32_t last_odometry_ms;
    float last_error = 0.0f;
    float last_yaw;
    float progress_deg = 0.0f;
    uint8_t pickup_index = 0U;
    uint8_t pickup_phase = 0U; /* Task2: 第1/2杯0/1直接提前切槽，第3杯保留0/1/2；Task1用0/1 */
    uint8_t progress_paused = 0U;
    float pickup_capture_x_mm = 0.0f;
    float pickup_capture_y_mm = 0.0f;
    float t2_last_capture_progress_deg = 0.0f;
    uint8_t t2_capture_anchor_valid = 0U;

    /*
     * Task2巡线入口缓速：
     * 配置里已有 T2_LINE_ENTRY_STABILIZE_SPEED / MM，
     * 这里只把这两个现成参数真正接入，不改任何已调参数。
     *
     * 从进入FollowLineAndCollect时记录底盘MapXY，
     * 前T2_LINE_ENTRY_STABILIZE_MM的实际位移使用入口缓速；
     * 超过后自动恢复原LINE_FORWARD_SPEED。
     */
    float t2_entry_start_x_mm = 0.0f;
    float t2_entry_start_y_mm = 0.0f;
    uint8_t t2_entry_stabilize_active = 0U;

    /*
     * Task1双进度：
     * pickup_index只管“机械上收到了第几个”；
     * t1_color_*只管“上一槽GY33识别到了哪一步”。
     *
     * 两者不再互相锁死。高速巡线时下一槽可以继续收料；
     * 若上一颜色真的还没完成，只在下一物料已经安全深入50mm后停车等待。
     */
    uint8_t t1_color_slot = 0U;
    uint8_t t1_color_phase = 0U; /* 0=idle, 1=wait servo, 2=vote */
    uint8_t t1_color_retry = 0U;
    uint8_t t1_color_failed = 0U;
    uint32_t t1_color_tick_ms = 0U;
    uint32_t t1_opening_tick_ms = 0U;

    /*
     * 只在Task1收真实物料期间使用的局部节奏。
     * 不修改competition_config.h / turnable2.c中的任何现有参数。
     */
    const float t1_real_color_line_speed = 85.0f;
    const uint32_t t1_color_settle_ms = 400U;
    const uint32_t t1_color_vote_first_ms = 500U;
    const uint32_t t1_color_vote_max_ms = 700U;
    const uint32_t line_loss_confirm_ms = 60U;

    if (HWT101_IsOnline() == 0U) return 0U;

    /*
     * 正常全流程/任务一整段调试时，PD14/PD15已经在T1_GO_LINE移动期间准备。
     * 只有直接从“任务一巡线收料”模块起跑时，才保留原1200ms安全等待。
     */
    if ((mission == LINE_MISSION_TASK1) &&
        (s_active_module == COMP_MODULE_T1_LINE_COLLECT))
    {
        Chassis_Stop();
        Servo_Pivot_270_SetAngle(DISC_SERVO_TASK1_LINE_PICK_DEG);
        if (AppControl_Delay(DISC_SERVO_SETTLE_MS) == 0U) return 0U;
    }

    if (mission == LINE_MISSION_TASK2)
    {
        pickup_total = 3U;
        direction = +1;
    }
    else
    {
        pickup_total = 5U;
        /* 新路线从下往上沿左圆顺时针，Yaw随前进总体递减。 */
        direction = -1;
        Turnable_ClearMaterialRecords();
        s_material_count = 0U;
    }

    /*
     * Task2保持原收奖杯状态机。
     * Task1从这里开始改成机械/颜色双进度：slot0直接对开口。
     */
    if (mission == LINE_MISSION_TASK2)
    {
        if (flag_task2_get_ready == 0U)
        {
            if (PrepareLinePickup(mission, 0U) == 0U) return 0U;
        }
    }
    else
    {
        if (s_active_module == COMP_MODULE_T1_LINE_COLLECT)
        {
            Servo_Spin_360_SetAngle(task1_configs[0U].angle_opening);
            if (AppControl_Delay(task1_configs[0U].time_wait_align) == 0U)
            {
                return 0U;
            }
        }

        /* 正常全流程中槽0已经在去黑线途中稳定完成，可立即进入收料。 */
        t1_opening_tick_ms =
            HAL_GetTick() - task1_configs[0U].time_wait_align;
    }

    if (Chassis_OdometryUpdate() == false) return 0U;
    {
        ChassisPose_t pose = Chassis_GetPose();

        if ((mission == LINE_MISSION_TASK2) && (pose.valid != false))
        {
            t2_entry_start_x_mm = pose.x_mm;
            t2_entry_start_y_mm = pose.y_mm;
            t2_entry_stabilize_active = 1U;
        }

        /*
         * 两个任务都禁止用旧stop_progress提前截断。
         * Task1必须机械收满5个后，由第五个物料的“50mm进入完成”分支直接结束；
         * Task2继续使用它自己的地图几何出口判定。
         */
        (void)pose;
        stop_progress_deg = 10000.0f;
    }

    start_ms = HAL_GetTick();
    last_line_seen_ms = start_ms;
    last_odometry_ms = start_ms;
    last_yaw = HWT101_GetYaw();

    while ((HAL_GetTick() - start_ms) < LINE_TIMEOUT_MS)
    {
        uint32_t now_ms;
        float current_yaw;
        float step_deg;
        LineSensor_t line;
        uint8_t slot_ready = 0U;
        uint8_t slot_done = 0U;
        float forward_speed =
            (mission == LINE_MISSION_TASK1) ?
            t1_real_color_line_speed :
            ((t2_entry_stabilize_active != 0U) ?
             T2_LINE_ENTRY_STABILIZE_SPEED : LINE_FORWARD_SPEED);
        uint8_t hold_for_slot = 0U;

        AppControl_Service();
        if (AppControl_StopRequested() != 0U)
        {
            Chassis_Stop();
            return 0U;
        }

        now_ms = HAL_GetTick();
        current_yaw = HWT101_GetYaw();
        step_deg = (float)direction *
                   Route_NormalizeAngle(current_yaw - last_yaw);
        last_yaw = current_yaw;

        /*
         * 只在底盘实际沿黑线前进时累计巡线进度。
         * 停车识色、PD15转动、等待下一槽ready期间，
         * HWT101的漂移/机械晃动不能算作已经沿赛道前进。
         */
        if ((progress_paused == 0U) &&
            (step_deg > 0.0f) &&
            (step_deg < LINE_PROGRESS_MAX_STEP_DEG))
        {
            progress_deg += step_deg;
        }

        /*
         * 巡线时持续读取真实1/2/3/4四个底盘编码器并更新MapX/MapY。
         * 任意一台读取失败，Chassis_OdometryUpdate()直接失败并STOP。
         */
        if ((now_ms - last_odometry_ms) >= LINE_ODOMETRY_PERIOD_MS)
        {
            last_odometry_ms = now_ms;
            if (Chassis_OdometryUpdate() == false)
            {
                Chassis_Stop();
                return 0U;
            }
        }

        /*
         * Task2一进入巡线马上就是弯道，不能第一拍就直接上145。
         * 用MapXY实际位移判断入口缓速距离，不假设前面存在直线。
         * 比较平方距离，避免引入sqrtf额外代码体积。
         */
        if (t2_entry_stabilize_active != 0U)
        {
            ChassisPose_t pose = Chassis_GetPose();

            if (pose.valid != false)
            {
                float dx = pose.x_mm - t2_entry_start_x_mm;
                float dy = pose.y_mm - t2_entry_start_y_mm;
                float stabilize_mm = T2_LINE_ENTRY_STABILIZE_MM;

                if ((dx * dx + dy * dy) >=
                    (stabilize_mm * stabilize_mm))
                {
                    t2_entry_stabilize_active = 0U;
                    forward_speed = LINE_FORWARD_SPEED;
                }
                else
                {
                    forward_speed = T2_LINE_ENTRY_STABILIZE_SPEED;
                }
            }
        }

        /*
         * Task2原状态机完全保留。
         * Task1不再调用旧的“识色done后才推进下一槽”的串行状态机。
         */
        if ((mission == LINE_MISSION_TASK2) && (pickup_index < pickup_total))
        {
            Turnable_Get_Material_Task2(pickup_index, 1U);
            slot_ready = flag_task2_get_ready;
            slot_done = flag_task2_get_done;
        }

#if !T1_BYPASS_COLOR_RANDOM_PLACE
        /*
         * Task1真实颜色后台识别：
         * 1) PD15到GY33后先稳定400ms；
         * 2) 连续投票500ms先判断一次；
         * 3) 若UNKNOWN/重复色，不清空已有样本，继续累计到700ms再判断；
         * 4) 700ms仍失败才交给原机械安全停车/报错兜底。
         */
        if ((mission == LINE_MISSION_TASK1) && (t1_color_phase != 0U))
        {
            if (t1_color_phase == 1U)
            {
                if ((now_ms - t1_color_tick_ms) >= t1_color_settle_ms)
                {
                    Turnable_Vote_Reset();
                    t1_color_tick_ms = now_ms;
                    t1_color_phase = 2U;
                    t1_color_retry = 0U;
                }
            }
            else
            {
                uint32_t vote_elapsed_ms;
                uint32_t vote_check_ms;

                Turnable_Vote_Process();

                vote_elapsed_ms = now_ms - t1_color_tick_ms;
                vote_check_ms = (t1_color_retry == 0U) ?
                                t1_color_vote_first_ms :
                                t1_color_vote_max_ms;

                if (vote_elapsed_ms >= vote_check_ms)
                {
                    GY33_CustomColor_t result = Turnable_Vote_GetResult();
                    uint8_t valid = (result != GY33_COLOR_UNKNOWN) ? 1U : 0U;
                    uint8_t used_slot;

                    if (valid != 0U)
                    {
                        for (used_slot = 0U;
                             used_slot < t1_color_slot;
                             used_slot++)
                        {
                            if (global_block_material_color_array[used_slot] ==
                                result)
                            {
                                valid = 0U;
                                break;
                            }
                        }
                    }

                    if (valid != 0U)
                    {
                        last_identified_color = result;
                        global_block_material_color_array[t1_color_slot] =
                            result;
                        t1_color_phase = 0U;
                        t1_color_retry = 0U;
                        t1_color_failed = 0U;
                    }
                    else if (t1_color_retry == 0U)
                    {
                        /*
                         * 第一次500ms结果不可靠：只延长观察到700ms。
                         * 不Reset投票、不重置起始时间，保留前面所有有效样本。
                         */
                        t1_color_retry = 1U;
                    }
                    else
                    {
                        t1_color_phase = 0U;
                        t1_color_failed = 1U;
                    }
                }
            }
        }
#endif

        if ((pickup_index < pickup_total) && (pickup_phase == 0U))
        {
            if (mission == LINE_MISSION_TASK1)
            {
                /*
                 * 任务一最新实车规则：
                 *
                 * M1~M5的HWT101触发角保持已经调好的机械触发点；
                 * 触发后继续寻迹50mm，再允许PD15转槽。
                 *
                 * GY33现在确认正好与外部开口相差一个槽距72°：
                 *   当前物料进入
                 *   -> PD15转72°
                 *   -> 当前物料到GY33
                 *   -> 下一个空槽同时正对外部开口
                 *   -> GY33识色期间底盘继续寻迹，不停车。
                 *
                 * 因此下一槽状态机短暂进入ready准备阶段时，也不再
                 * 为PD15停车；只要ready以后才允许正式触发下一物料。
                 */
                if (((now_ms - t1_opening_tick_ms) >=
                     task1_configs[pickup_index].time_wait_align) &&
                    (progress_deg >= s_task1_pick_progress_deg[pickup_index]))
                {
                    ChassisPose_t capture_pose;

                    if (Chassis_OdometryUpdate() == false)
                    {
                        Chassis_Stop();
                        return 0U;
                    }

                    capture_pose = Chassis_GetPose();
                    if (capture_pose.valid == false)
                    {
                        Chassis_Stop();
                        return 0U;
                    }

                    pickup_capture_x_mm = capture_pose.x_mm;
                    pickup_capture_y_mm = capture_pose.y_mm;
                    pickup_phase = 1U;

                    /* phase=1期间继续寻迹，不停车、不转PD15。 */
                }
            }
            else
            {
                uint8_t capture_now = 0U;

                /*
                 * 第1杯继续使用当前已经能工作的MapXY+20°窗口作为锚点。
                 * 后续不再让MapXY累计漂移拖慢第2/3杯：
                 * C->B按HWT累计25.25°，B->A按HWT累计24.98°触发。
                 * HWT只负责收杯时序，不覆盖四轮MapXY。
                 */
                if (pickup_index == 0U)
                {
                    float target_sin = Line_GetTask2PickupAngleSin(0U);

                    if ((target_sin <= T2_EARLY_NEXT_SLOT_CAPTURE_SIN) &&
                        (target_sin >= 0.0f))
                    {
                        capture_now = 1U;
                    }
                    else if (target_sin < -LINE_PICK_MISSED_SIN)
                    {
                        Chassis_Stop();
                        return 0U;
                    }
                }
                else if ((t2_capture_anchor_valid != 0U) &&
                         (slot_ready != 0U))
                {
                    float gap_deg = progress_deg - t2_last_capture_progress_deg;
                    float required_deg =
                        (pickup_index == 1U) ? 33.25f : 32.98f;

                    if (gap_deg >= required_deg) capture_now = 1U;
                }

                if (capture_now != 0U)
                {
                    ChassisPose_t capture_pose;

                    if (Chassis_OdometryUpdate() == false)
                    {
                        Chassis_Stop();
                        return 0U;
                    }

                    capture_pose = Chassis_GetPose();
                    if (capture_pose.valid == false)
                    {
                        Chassis_Stop();
                        return 0U;
                    }

                    pickup_capture_x_mm = capture_pose.x_mm;
                    pickup_capture_y_mm = capture_pose.y_mm;
                    pickup_phase = 1U;
                    t2_last_capture_progress_deg = progress_deg;
                    t2_capture_anchor_valid = 1U;
                }
            }
        }
        else if ((pickup_index < pickup_total) && (pickup_phase == 1U))
        {
            ChassisPose_t pose_now;
            float dx_mm;
            float dy_mm;
            float travel_mm;
            float delay_mm;

            /*
             * 从“物料/奖杯开始进入圆盘开口”的触发位置开始继续寻迹。
             *
             * 任务二：本轮实车改为80mm；
             * 任务一：50mm（V2确认不再额外晚3cm）。
             *
             * 延迟距离内PD15保持当前收料槽不动；达到对应距离后才置
             * entered标志，让内部圆盘开始后续动作。
             */
            if (Chassis_OdometryUpdate() == false)
            {
                Chassis_Stop();
                return 0U;
            }

            pose_now = Chassis_GetPose();
            if (pose_now.valid == false)
            {
                Chassis_Stop();
                return 0U;
            }

            dx_mm = pose_now.x_mm - pickup_capture_x_mm;
            dy_mm = pose_now.y_mm - pickup_capture_y_mm;
            travel_mm = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);

            delay_mm = (mission == LINE_MISSION_TASK2) ?
                       T2_TROPHY_SLOT_SWITCH_DELAY_MM :
                       T1_MATERIAL_SLOT_SWITCH_DELAY_MM;

            if (travel_mm >= delay_mm)
            {
                if (mission == LINE_MISSION_TASK2)
                {
                    /*
                     * 第1、2个奖杯：
                     * 当前config里的T2_TROPHY_SLOT_SWITCH_DELAY_MM仍原样使用。
                     * 当达到该距离（现在用户参数为0mm）后，直接停止旧槽get状态，
                     * 并立即把PD15命令到下一槽开口。
                     *
                     * 关键：不再等待flag_task2_get_done。
                     * 这正是之前“delay=0但看起来几乎没变化”的真正瓶颈。
                     */
                    if (pickup_index < 2U)
                    {
                        Turnable_Get_Material_Task2(pickup_index, 0U);

                        Servo_Spin_360_SetAngle(
                            task2_configs[pickup_index + 1U].angle_opening);

                        pickup_index++;
                        pickup_phase = 0U;
                        last_line_seen_ms = now_ms;

                        /*
                         * 立即启动下一槽的软件ready状态。
                         * 舵机角命令已经提前下发，所以ready计时不会再拖延实际转盘动作。
                         */
                        Turnable_Get_Material_Task2(pickup_index, 1U);
                    }
                    else
                    {
                        /*
                         * 第3杯仍走原done流程，保证最后奖杯确实完成后
                         * 才转到运输安全位并退出巡线。
                         */
                        flag_block_entered_Task2 = 1U;
                        pickup_phase = 2U;
                    }
                }
                else
                {
#if !T1_BYPASS_COLOR_RANDOM_PLACE
                    /*
                     * 到这里当前物料已经从触发点继续走满50mm。
                     * 如果上一槽颜色还在识别，才在这个机械安全位置停车等；
                     * 一旦识别结束立即转72°并继续高速巡线。
                     */
                    if (t1_color_failed != 0U)
                    {
#if T1_COLOR_FAIL_CONTINUE_ROUTE_TEST
                        /*
                         * 点位测试阶段：这一槽颜色识别失败也不停车。
                         * 槽内物料机械上已经安全进入50mm，继续转下一槽；
                         * 收满5个后统一用“剩余未用颜色”临时补齐失败槽。
                         */
                        t1_color_failed = 0U;
#else
                        Chassis_Stop();
                        return 0U;
#endif
                    }

                    if (t1_color_phase != 0U)
                    {
                        hold_for_slot = 1U;
                    }
                    else
#endif
                    {
                        uint8_t finished_slot = pickup_index;

                        /*
                         * slot0~3只转72°：
                         * 当前物料到GY33，同时下一个空槽来到外开口。
                         * 新实测开口54°后，对应126/198/270/342°。
                         */
                        if (finished_slot < 4U)
                        {
                            Servo_Spin_360_SetAngle(
                                task1_configs[finished_slot].angle_to_gy33);
                            t1_opening_tick_ms = now_ms;

#if !T1_BYPASS_COLOR_RANDOM_PLACE
                            t1_color_slot = finished_slot;
                            t1_color_tick_ms = now_ms;
                            t1_color_phase = 1U;
                            t1_color_retry = 0U;
                            t1_color_failed = 0U;
#endif
                        }
                        else
                        {
                            /*
                             * 新路线第五个（顶部M1）从触发点继续走满50mm：
                             * 先把PD15从338°反向36°到302°安全位，立即停车，
                             * 等内部圆盘稳定后直接结束巡线。
                             *
                             * 这里不再等待旧T1_LINE_STOP_YAW_DEG，也不再继续多走。
                             */
                            if (TurnableParkTask1FifthOnLine() == 0U)
                            {
                                Chassis_Stop();
                                return 0U;
                            }

                            pickup_index = pickup_total;
                            s_material_count = pickup_total;
                            pickup_phase = 0U;

                            Chassis_Stop();

                            if (s_active_module == COMP_MODULE_T1_LINE_COLLECT)
                            {
                                /* 单独巡线模块保持原停车稳定行为。 */
                                if (AppControl_Delay(TURNABLE_T2_TRANSPORT_SETTLE_MS) == 0U)
                                    return 0U;
                            }
                            else
                            {
                                /*
                                 * 正常全流程：PD15转安全位的同时让PD14从210°回220°。
                                 * 两个舵机并行动作，不再先白等一轮再开始PD14。
                                 */
                                DiscServo_ToFront();
                            }

                            return 1U;
                        }

                        pickup_index++;
                        s_material_count = pickup_index;
                        pickup_phase = 0U;
                    }
                }
            }
        }
        else if ((mission == LINE_MISSION_TASK2) &&
                 (pickup_index < pickup_total) &&
                 (pickup_phase == 2U))
        {
            if (slot_done != 0U)
            {
                Turnable_Get_Material_Task2(pickup_index, 0U);

                /*
                 * 第三杯一完成就在巡线状态内命令安全位：
                 * 底盘继续巡线，PD15同时转232°。
                 * 不再等到出口后Yaw转向时才转圆盘。
                 */
                if (pickup_index == 2U)
                {
                    if (TurnableParkTask2ThirdOnLine() == 0U) return 0U;
                    Chassis_Stop();
                    if (AppControl_Delay(TURNABLE_T2_TRANSPORT_SETTLE_MS) == 0U)
                        return 0U;
                    return 1U;
                }

                pickup_index++;
                pickup_phase = 0U;
                last_line_seen_ms = HAL_GetTick();

                if (pickup_index < pickup_total)
                {
                    Turnable_Get_Material_Task2(pickup_index, 1U);
                }
            }
            else if ((pickup_index + 1U) < pickup_total)
            {
                float next_sin =
                    Line_GetTask2PickupAngleSin((uint8_t)(pickup_index + 1U));

                if ((next_sin <= 0.0f) ||
                    (next_sin <= LINE_NEXT_SLOT_STOP_SIN))
                {
                    hold_for_slot = 1U;
                }
                else if (next_sin <= LINE_NEXT_SLOT_SLOW_SIN)
                {
                    forward_speed = LINE_FORWARD_SPEED_PREPARE;
                }
            }
        }

        /*
         * 先读取本周期八路巡线状态。
         * 同一帧数据既用于任务二稳定出口判定，也用于后面的PD巡线控制。
         */
        LineSensor_Read(&line);

        /*
         * 巡线终点策略分开：
         *
         * 任务二：
         *   必须先完整收完3个奖杯（pickup_index==3 && phase==0），
         *   其中每杯触发后本轮都继续走80mm，
         *   然后才检查底盘地图航向 MapYaw。
         *   第三杯完整收完后继续巡线；
         *   用实时MapX/MapY计算车体中心相对右侧圆心的位置角；
         *   位置角到13.72°才停车，不再拿瞬时车头Yaw当位置。
         *   这里禁止直接比较HWT101 RawYaw；RawYaw带上电零偏，
         *   而后续Route_RotateTo/Route_MoveToXY全部使用MapYaw/MapXY。
         *
         * 这样第三个奖杯处于“再走50mm”或PD15处理阶段时，
         * 绝不会因为终点条件先到而误报 Task2 line collect failed。
         *
         * 任务一：
         *   继续使用原来的累计progress终点。
         */
        if (mission == LINE_MISSION_TASK2)
        {
            if ((pickup_index >= pickup_total) && (pickup_phase == 0U))
            {
                ChassisPose_t pose = Chassis_GetPose();

                if (pose.valid != false)
                {
                    float rx = pose.x_mm - MAP_LINE_RIGHT_CENTER_X_MM;
                    float ry = pose.y_mm - MAP_LINE_CENTER_Y_MM;
                    float radius_sq = rx * rx + ry * ry;
                    float exit_cross;
#if T2_ANGLE_TEST_ENABLE
                    /*
                     * 测角版：第三杯后继续巡线到45°位置角。
                     * 这里测的是车体中心相对右圆心的几何位置角，
                     * 不是巡线时会左右抖的HWT瞬时车头角。
                     */
                    exit_cross =
                        T2_ANGLE_TEST_POS_COS * ry -
                        T2_ANGLE_TEST_POS_SIN * rx;
#else
                    exit_cross =
                        T2_LINE_EXIT_TARGET_COS * ry -
                        T2_LINE_EXIT_TARGET_SIN * rx;
#endif

                    /*
                     * T2_ANGLE_TEST_ENABLE=1时：
                     * 看车体中心MapXY是否越过45°测试射线；
                     * 正式版关闭测试宏后才恢复13.72°目标射线。
                     * 全程不看巡线纠偏造成的瞬时车头Yaw。
                     * 这里只用乘加和平方比较，避免额外数学库体积。
                     */
                    if ((radius_sq >= T2_LINE_EXIT_RADIUS_MIN_SQ) &&
                        (radius_sq <= T2_LINE_EXIT_RADIUS_MAX_SQ) &&
                        (exit_cross >= 0.0f))
                    {
                        Chassis_Stop();

                        if (AppControl_Delay(120U) == 0U) return 0U;
                        if (Chassis_OdometryUpdate() == false) return 0U;
                        return 1U;
                    }
                }
            }
        }
        else
        {
            if (progress_deg >= stop_progress_deg)
            {
                /*
                 * Task1出口位置只认原来的陀螺仪累计角。
                 * 到角立即停车；颜色再慢也只能原地等，绝不多走25cm
                 * 去“给软件追进度”，从而保证后面的-90°、412mm、QR和A~E
                 * 仍使用原来已经调好的几何关系。
                 */
                Chassis_Stop();

                /*
                 * 实车已确认M5在此处机械上已经进入槽内。
                 * 若刚好只差M5的50mm软件计数，不再改变出口位置，
                 * 原地把最后机械计数完成。
                 */
                if ((pickup_index == 4U) && (pickup_phase == 1U))
                {
                    pickup_index = 5U;
                    pickup_phase = 0U;
                    s_material_count = 5U;
                }

#if T1_BYPASS_COLOR_RANDOM_PLACE
                if ((pickup_index >= pickup_total) && (pickup_phase == 0U))
                {
                    if (AppControl_Delay(180U) == 0U) return 0U;
                    if (Chassis_OdometryUpdate() == false) return 0U;
                    return 1U;
                }
#else
                /*
                 * 将来打开真实颜色：
                 * 前4色后台识别若尚未完成，就保持车完全静止在出口角等待。
                 * 识色完整后再推断第5色，绝不影响后续二维码/A点位置。
                 */
                if (t1_color_phase != 0U)
                {
                    HAL_Delay(LINE_CONTROL_PERIOD_MS);
                    continue;
                }

                if (t1_color_failed != 0U)
                {
#if T1_COLOR_FAIL_CONTINUE_ROUTE_TEST
                    t1_color_failed = 0U;
#else
                    return 0U;
#endif
                }

                if ((pickup_index >= pickup_total) && (pickup_phase == 0U))
                {
#if T1_COLOR_FAIL_CONTINUE_ROUTE_TEST
                    (void)Turnable_FinalizeTask1FifthColor();
                    Task1_RepairCollectedColorsForRouteTest();
#else
                    if (Turnable_FinalizeTask1FifthColor() == 0U)
                    {
                        return 0U;
                    }
#endif

                    if (AppControl_Delay(180U) == 0U) return 0U;
                    if (Chassis_OdometryUpdate() == false) return 0U;
                    return 1U;
                }
#endif

                return 0U;
            }

            if (progress_deg > (stop_progress_deg + LINE_PROGRESS_OVER_DEG))
            {
                Chassis_Stop();
                return 0U;
            }
        }

        /*
         * 记录本周期是否停车；下一周期只在未停车时累计HWT巡线进度。
         */
        progress_paused = (hold_for_slot != 0U) ? 1U : 0U;

        if (hold_for_slot != 0U)
        {
            /* 停在黑线上等PD15时仍持续刷新“在线”时间，避免误报丢线。 */
            if (line.online != 0U) last_line_seen_ms = now_ms;
            Chassis_Stop();
        }
        else
        {
            LineApplyControl(&line,
                             &last_error,
                             now_ms,
                             &last_line_seen_ms,
                             forward_speed);
        }

        if ((hold_for_slot == 0U) &&
            ((now_ms - last_line_seen_ms) > line_loss_confirm_ms))
        {
            uint32_t recover_start_ms = HAL_GetTick();
            uint32_t recover_elapsed_ms;

            /*
             * 连续约60ms看不到线后进入柔和恢复：
             * 低速回退找线 -> 85速低速居中 -> 连续稳定后再回主巡线。
             */
            Chassis_Stop();

            if (LineRecoverBackToLine(&last_error,
                                      &last_line_seen_ms,
                                      forward_speed) == 0U)
            {
                Chassis_Stop();
                return 0U;
            }

            recover_elapsed_ms = HAL_GetTick() - recover_start_ms;

#if !T1_BYPASS_COLOR_RANDOM_PLACE
            /*
             * 丢线恢复期间后台GY33投票没有执行。
             * 把这段阻塞时间从颜色稳定/投票计时中扣掉，避免“时间到了但没采够帧”。
             */
            if ((mission == LINE_MISSION_TASK1) && (t1_color_phase != 0U))
            {
                t1_color_tick_ms += recover_elapsed_ms;
            }
#endif

            /*
             * 恢复过程产生的Yaw变化不计入正常巡线progress。
             */
            last_yaw = HWT101_GetYaw();
            progress_paused = 1U;

            HAL_Delay(LINE_CONTROL_PERIOD_MS);
            continue;
        }

        HAL_Delay(LINE_CONTROL_PERIOD_MS);
    }

    Chassis_Stop();
    return 0U;
#else
    (void)mission;
    return 1U;
#endif
}

/* ============================= Lifecycle ============================= */

/**
 * @brief 初始化状态机、转盘、舵机和 K230 参数。
 *
 * 本函数只初始化，不启动路线。
 */
void CompetitionTask_Init(void)
{
    HAL_StatusTypeDef turnable_status = HAL_OK;

    /* 任务二名次映射必须由二维码产生；上电时保持无效，禁止固定兜底。 */
    memset(&s_trophy_qr, 0, sizeof(s_trophy_qr));
    memset(s_target_by_position, 0, sizeof(s_target_by_position));
    s_material_count = 0U;
    s_init_ok = 0U;
    s_last_error = "-";
    s_selected_module = COMP_MODULE_FULL_ROUTE;
    s_active_module = COMP_MODULE_FULL_ROUTE;
    s_module_boundary_enabled = 0U;
    s_state = COMP_STATE_IDLE;
    s_state_enter_ms = HAL_GetTick();

    /*
     * 上电机械初态：整个圆盘位于车体上方，内部槽1正对圆盘开口。
     * 起步前进后先把圆盘翻到车前并下降，再去任务二二维码点扫码。
     */
    LineSensor_Init();
    Servo_Init();
    /*
     * 调点阶段临时初态：
     * DEBUG_INIT_L0_D220=1 时，上电直接D220并以当前位置作为L0，
     * 避免每次调试反复翻大圆盘，保护3D打印支架。
     */
#if DEBUG_INIT_L0_D220
    DiscServo_ToFront();
#else
    DiscServo_ToRear();
#endif

#if COMP_ENABLE_TURNTABLE
    turnable_status = Turnable_Init();
    Turnable_ResetAll();
    if (turnable_status != HAL_OK)
    {
        Route_EnterError(1U);
        return;
    }
#endif

#if COMP_ENABLE_LIFT
#if DEBUG_INIT_L0_D220
    /* 调试时上电前请先把机械升降实际放在L0；这里只建立坐标，不会主动找零。 */
    if (Lift_Init(LIFT_GROUND_HEIGHT_MM) == false)
#else
    if (Lift_Init(LIFT_HOME_HEIGHT_MM) == false)
#endif
    {
        Route_EnterError(2U);
        return;
    }
#endif

#if COMP_ENABLE_K230
    if (K230_Init() != HAL_OK)
    {
        Route_EnterError(3U);
        return;
    }
    K230_SetAlignmentConfig(K230_TARGET_CENTER_X,
                            K230_TARGET_CENTER_Y,
                            K230_PIXEL_TOLERANCE_X,
                            K230_PIXEL_TOLERANCE_Y,
                            K230_STABLE_FRAMES);
    K230_SetDataTimeout(K230_DATA_TIMEOUT_MS);
#endif

    s_init_ok = 1U;
    s_last_error = "-";

}

/**
 * @brief 根据模块设置独立调试起点姿态、入口状态与结束边界。
 */
static uint8_t ConfigureModuleStart(CompetitionModule_t module,
                                    CompetitionState_t *entry_state,
                                    float *start_x_mm,
                                    float *start_y_mm,
                                    float *start_yaw_deg)
{
    if ((entry_state == NULL) || (start_x_mm == NULL) ||
        (start_y_mm == NULL) || (start_yaw_deg == NULL)) return 0U;

    s_module_boundary_enabled = 1U;

    switch (module)
    {
        case COMP_MODULE_FULL_ROUTE:
            *entry_state = COMP_STATE_T2_INITIAL_FORWARD;
            *start_x_mm = MAP_HOME_X_MM;
            *start_y_mm = MAP_HOME_Y_MM;
            *start_yaw_deg = MAP_HOME_YAW_DEG;
            s_module_boundary_enabled = 0U;
            memset(&s_trophy_qr, 0, sizeof(s_trophy_qr));
            memset(s_target_by_position, 0, sizeof(s_target_by_position));
            s_material_count = 0U;
            break;

        case COMP_MODULE_T2_START_TO_QR:
            *entry_state = COMP_STATE_T2_INITIAL_FORWARD;
            *start_x_mm = MAP_HOME_X_MM;
            *start_y_mm = MAP_HOME_Y_MM;
            *start_yaw_deg = MAP_HOME_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T2_GO_LINE;
            memset(&s_trophy_qr, 0, sizeof(s_trophy_qr));
            break;

        case COMP_MODULE_T2_QR_TO_LINE:
            *entry_state = COMP_STATE_T2_GO_LINE;
            *start_x_mm = MAP_TASK2_QR_X_MM;
            *start_y_mm = MAP_TASK2_QR_Y_MM;
            *start_yaw_deg = MAP_TASK2_QR_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T2_FOLLOW_COLLECT;
            break;

        case COMP_MODULE_T2_LINE_COLLECT:
            *entry_state = COMP_STATE_T2_FOLLOW_COLLECT;
            *start_x_mm = DEBUG_T2_LINE_ENTRY_X_MM;
            *start_y_mm = DEBUG_T2_LINE_ENTRY_Y_MM;
            *start_yaw_deg = DEBUG_T2_LINE_ENTRY_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T2_EXIT_LINE;
            break;

        case COMP_MODULE_T2_EXIT_TO_PODIUM:
            *entry_state = COMP_STATE_T2_EXIT_LINE;
            *start_x_mm = DEBUG_T2_LINE_EXIT_X_MM;
            *start_y_mm = DEBUG_T2_LINE_EXIT_Y_MM;
            *start_yaw_deg = DEBUG_T2_LINE_EXIT_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T2_PLACE_SECOND;
            break;

        case COMP_MODULE_T2_PODIUMS:
#if COMP_QR_BYPASS_RANDOM
            /*
             * QR断线调试时M5允许独立启动：
             * 若当前还没有奖杯名次映射，现场生成一组合法随机映射。
             */
            if ((s_trophy_qr.valid == 0U) &&
                (ApplyRandomQrTask(1U) == 0U))
            {
                return 0U;
            }
#else
            if (s_trophy_qr.valid == 0U) return 0U;
#endif
            *entry_state = COMP_STATE_T2_PLACE_SECOND;
            *start_x_mm = MAP_PODIUM_SECOND_PRE_X_MM;
            *start_y_mm = MAP_PODIUM_SECOND_PRE_Y_MM;
            *start_yaw_deg = MAP_HOME_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T1_GO_QR;
            break;

        case COMP_MODULE_T1_GO_LINE:
            /*
             * MODE=6沿用纯底盘调试习惯：从季军结束位置开始后，
             * 连续执行完整任务一（入线→收料/识色→扫码→ABCDE→Home）。
             */
            *entry_state = COMP_STATE_T1_GO_QR;
            *start_x_mm = MAP_PODIUM_THIRD_PRE_X_MM;
            *start_y_mm = MAP_PODIUM_THIRD_PRE_Y_MM;
            *start_yaw_deg = MAP_HOME_YAW_DEG;
            s_module_boundary_enabled = 0U;
            break;

        case COMP_MODULE_T1_LINE_COLLECT:
            *entry_state = COMP_STATE_T1_FOLLOW_COLLECT;
            *start_x_mm = MAP_T1_LINE_ENTRY_EXPECT_X_MM;
            *start_y_mm = MAP_T1_LINE_ENTRY_EXPECT_Y_MM;
            *start_yaw_deg = T1_AFTER_THIRD_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T1_EXIT_LINE;
            break;

        case COMP_MODULE_T1_GO_QR:
            *entry_state = COMP_STATE_T1_EXIT_LINE;
            *start_x_mm = DEBUG_T1_LINE_EXIT_X_MM;
            *start_y_mm = DEBUG_T1_LINE_EXIT_Y_MM;
            *start_yaw_deg = DEBUG_T1_LINE_EXIT_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T1_PREPARE_PLACE;
            break;

        case COMP_MODULE_T1_PLACE_A_B:
            /* 独立几何调试：不依赖前一模块留下的扫码/颜色状态。 */
            Task1_AssignDummyUniqueSlotColors();
            s_material_count = 5U;
            if (ApplyRandomQrTask(0U) == 0U) return 0U;
            /*
             * 恢复最开始的模块分类：
             * M9 = 任务一二维码点 -> A -> B，B放完并后退120mm后停止。
             */
            *entry_state = COMP_STATE_T1_PREPARE_PLACE;
            *start_x_mm = MAP_TASK1_QR_SCAN_X_MM;
            *start_y_mm = MAP_TASK1_QR_SCAN_Y_MM;
            *start_yaw_deg = MAP_TASK1_QR_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_T1_PLACE_C;
            break;

        case COMP_MODULE_T1_PLACE_C_D_E:
            /* 独立几何调试：不依赖前一模块留下的扫码/颜色状态。 */
            Task1_AssignDummyUniqueSlotColors();
            s_material_count = 5U;
            if (ApplyRandomQrTask(0U) == 0U) return 0U;
            /*
             * M10：从B已经放完并后退150 mm的位置开始，保持Yaw0°，
             * 继续 C -> D -> E。这里绑定名义车体中心位置。
             */
            *entry_state = COMP_STATE_T1_PLACE_C;
            *start_x_mm = MAP_PLACE_B_POST_X_MM;
            *start_y_mm = MAP_PLACE_B_POST_Y_MM;
            *start_yaw_deg = MAP_PLACE_B_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_RETURN_HOME;
            break;

        case COMP_MODULE_RETURN_HOME:
            /* M11：E已经放完并直退120 mm，保持Yaw0°直接一条直线回Home。 */
            *entry_state = COMP_STATE_RETURN_HOME;
            *start_x_mm = MAP_PLACE_E_POST_X_MM;
            *start_y_mm = MAP_PLACE_E_POST_Y_MM;
            *start_yaw_deg = MAP_PLACE_E_YAW_DEG;
            s_module_stop_before_state = COMP_STATE_FINISHED;
            break;

        default:
            return 0U;
    }

    return 1U;
}

uint8_t CompetitionTask_StartModule(CompetitionModule_t module)
{
    CompetitionState_t entry_state;
    float start_x_mm;
    float start_y_mm;
    float start_yaw_deg;

    if (s_init_ok == 0U)
    {
        s_last_error = "init";
        return 0U;
    }

    if (s_state != COMP_STATE_IDLE)
    {
        s_last_error = "NI";
        return 0U;
    }

    if (module >= COMP_MODULE_COUNT)
    {
        s_last_error = "M!";
        return 0U;
    }

    if (HWT101_IsOnline() == 0U)
    {
        s_last_error = "HWT!";
        return 0U;
    }

    if (ConfigureModuleStart(module,
                             &entry_state,
                             &start_x_mm,
                             &start_y_mm,
                             &start_yaw_deg) == 0U)
    {
        s_last_error = "MP";
        return 0U;
    }

    AppControl_ClearStopRequest();
    Turnable_ResetStateMachines();

    if (Chassis_OdometryResetPose(start_x_mm,
                                  start_y_mm,
                                  start_yaw_deg) == false)
    {
        Route_EnterError(4U);
        return 0U;
    }

    s_selected_module = module;
    s_active_module = module;
    s_state = entry_state;
    s_state_enter_ms = HAL_GetTick();
    s_last_error = "-";
    AppControl_SetRunning(1U);
    return 1U;
}

uint8_t CompetitionTask_StartSelectedModule(void)
{
    return CompetitionTask_StartModule(s_selected_module);
}

void CompetitionTask_Start(void)
{
    (void)CompetitionTask_StartModule(COMP_MODULE_FULL_ROUTE);
}

void CompetitionTask_Stop(void)
{
    AppControl_RequestStop();
    Route_StopToIdle();
    AppControl_ClearStopRequest();
}

/**
 * @brief 比赛状态机周期函数。
 *
 * main循环持续调用；每个状态完成一个明确动作，失败即进入ERROR。
 */
void CompetitionTask_Update(void)
{
    AppControl_Service();
    K230_Process();
    Lift_Update();

    if ((AppControl_StopRequested() != 0U) && (s_state != COMP_STATE_IDLE))
    {
        Route_StopToIdle();
        AppControl_ClearStopRequest();
        return;
    }

    switch (s_state)
    {
        case COMP_STATE_IDLE:
            break;

        case COMP_STATE_T2_INITIAL_FORWARD:
            /*
             * 最终实车流程：
             * Home -> 任务二二维码改为直接斜线。
             *
             * 这里不再执行旧的“先前进一段”。
             * 保留这个状态仅为了不改状态枚举/蓝牙#S编号。
             */
            Route_ChangeState(COMP_STATE_T2_DEPLOY_DISC);
            break;

        case COMP_STATE_T2_DEPLOY_DISC:
#if DEBUG_INIT_L0_D220
            /* 保留旧调点分支；当前正式配置为0，不会进入这里。 */
            Route_ChangeState(COMP_STATE_T2_TURN_TO_QR);
            break;
#else
            /*
             * 正式比赛启动：
             *   上电初态已经按 L200 + D240 建立；
             *   按开始后立即命令PD14转到现有正前方角度；
             *   同时启动5号升降从L200下降到现有L4；
             *   不再等待DISC_SERVO_SETTLE_MS，下一状态立即开始现有
             *   HOME -> Task2二维码路线，因此舵机/升降/底盘动作并行。
             *
             * 到二维码点后仍保留原来的 Lift_WaitUntilTarget()，
             * 确保扫码前升降最终已经到L4。
             */
            DiscServo_ToFront();
#if COMP_ENABLE_LIFT
            if (Lift_StartGotoMM(LIFT_GROUND_HEIGHT_MM) == false)
            { Route_EnterError(6U); break; }
#endif
            Route_ChangeState(COMP_STATE_T2_TURN_TO_QR);
            break;
#endif

        case COMP_STATE_T2_TURN_TO_QR:
#if COMP_ENABLE_TASK2_QR
            /*
             * 只在开始去二维码点之前清一次旧缓存。
             * 后续接近途中扫到的第一帧要一直保留到SCAN_QR直接解析。
             */
            MJ6000_ClearRx();
#endif
            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(7U); break; }

            /*
             * HOME -> Task2二维码：Flash轻量版偏前斜线。
             *
             * 不新增软件速度闭环，直接复用现有0xFD位置模式，避免32KB超限。
             * 第一段：右移较少、前进较多，起步先明显往前但不是纯直行；
             * 第二段：接着斜到当前原始二维码总目标X/Y。
             *
             * 原T2_LOCAL_HOME_TO_QR_X/Y、RPM、ACC等现有参数全部不改。
             */
            if (Route_MovePosition(T2_HOME_QR_ENTRY_RIGHT_MM,
                                   T2_HOME_QR_ENTRY_FORWARD_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(8U); break; }

            if (Route_MovePosition(
                    T2_LOCAL_HOME_TO_QR_X_MM - T2_HOME_QR_ENTRY_RIGHT_MM,
                    T2_LOCAL_HOME_TO_QR_Y_MM - T2_HOME_QR_ENTRY_FORWARD_MM,
                    POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(8U); break; }

            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(9U); break; }

#if COMP_ENABLE_LIFT
#if !DEBUG_INIT_L0_D220
            if (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false)
            { Route_EnterError(10U); break; }
#endif
#endif
            Route_ChangeState(COMP_STATE_T2_GO_QR);
            break;

        case COMP_STATE_T2_GO_QR:
            /*
             * 上一状态已经斜线到达二维码点并完成最终Yaw=0°且停车。
             * V2不再重复发送一次四轮STOP，只保留状态编号兼容完整M0流程。
             */
            Route_ChangeState(COMP_STATE_T2_SCAN_QR);
            break;

        case COMP_STATE_T2_SCAN_QR:
            /* 当前位置停车后执行MJ6000真实任务二二维码识别。 */
            if (ReadQrWithRetry(1U) == 0U)
            { Route_EnterError(11U); break; }

            Route_ChangeState(COMP_STATE_T2_GO_LINE);
            break;

        case COMP_STATE_T2_GO_LINE:
            /*
             * 最新实车入口：
             * 二维码点 -> 整车快速转到-90°
             * -> 保持-90°直接斜移 body右65mm + body前90mm
             * -> 到位后直接进入巡线。
             *
             * 入口后前T2_LINE_ENTRY_STABILIZE_MM按现有入口缓速巡线，
             * 随后自动恢复LINE_FORWARD_SPEED；不再额外低速直行找线。
             */
            if (Route_RotateToFast(-90.0f) == 0U)
            { Route_EnterError(12U); break; }

            DiscServo_ToLinePickup();
            Turnable_Get_Material_Task2(0U, 1U);

            if (Route_MovePosition(T2_LOCAL_QR_TO_LINE_RIGHT_MM,
                                   T2_LOCAL_QR_TO_LINE_FORWARD_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(13U); break; }

            /*
             * 不额外停车确认黑线；
             * FollowLineAndCollect内部先用现有T2入口缓速跑固定距离，
             * 再自动恢复原LINE_FORWARD_SPEED。
             */
            Route_ChangeState(COMP_STATE_T2_FOLLOW_COLLECT);
            break;

        case COMP_STATE_T2_FOLLOW_COLLECT:
            /* Task2入口先短距离缓速稳定过弯，随后恢复原主巡线速度，PD15依次收3个奖杯。 */
            if (FollowLineAndCollect(LINE_MISSION_TASK2) == 0U)
            { Route_EnterError(14U); break; }
            Route_ChangeState(COMP_STATE_T2_EXIT_LINE);
            break;

        case COMP_STATE_T2_EXIT_LINE:
            Chassis_Stop();

            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(15U); break; }

            DiscServo_ToFront();

#if COMP_ENABLE_LIFT
            if (Lift_IsReferenced() == false)
            { Route_EnterError(16U); break; }
            if (Lift_IsBusy() == false)
            {
                if (Lift_StartGotoMM(LIFT_PODIUM_SECOND_APPROACH_MM) == false)
                { Route_EnterError(17U); break; }
            }
#endif

            if (Route_MovePosition(T2_LOCAL_POST3_TO_SECOND_X_MM,
                                   T2_LOCAL_POST3_TO_SECOND_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(18U); break; }

            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(19U); break; }

#if COMP_ENABLE_LIFT
            if (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false)
            { Route_EnterError(20U); break; }
#endif
            Route_ChangeState(COMP_STATE_T2_PLACE_SECOND);
            break;

        case COMP_STATE_T2_PLACE_SECOND:
            /*
             * 亚军（先前送，再下降）：
             *   -> 保持当前规避高度 LIFT_PODIUM_SECOND_APPROACH_MM；
             *   -> PD15选亚军奖杯 + 精确Yaw0；
             *   -> 在规避高度K230找圆并把车移动到圆心；
             *   -> 仍保持高位，先执行现有K230固定机械前送；
             *   -> 前送到位后底盘停车，只下降到现有放杯高度；
             *   -> 高度到位后直接停留并直退，不再二次前送。
             *
             * 所有高度、前送距离、速度、K230和路线参数均保持当前值。
             */
#if COMP_ENABLE_LIFT
            if ((Lift_StartGotoMM(LIFT_PODIUM_SECOND_APPROACH_MM) == false) ||
                (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false))
            { Route_EnterError(21U); break; }
#endif

            if (AlignTrophyAtCurrentHeight(PODIUM_SECOND) == 0U)
            { Route_EnterError(23U); break; }

            /* 高位先把奖杯送进奖台范围，避免低位前送时后排奖杯碰台。 */
            if (K230FinalPlacePush() == 0U)
            { Route_EnterError(23U); break; }

#if COMP_ENABLE_LIFT
            if ((Lift_StartGotoMM(LIFT_PODIUM_SECOND_HEIGHT_MM) == false) ||
                (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false))
            { Route_EnterError(22U); break; }
#endif

            /* 前送已经完成，这里只停留并后退。 */
            if (PlaceAlignedTrophy(PODIUM_SECOND, 0U) == 0U)
            { Route_EnterError(23U); break; }

            Route_ChangeState(COMP_STATE_T2_PLACE_FIRST);
            break;

        case COMP_STATE_T2_PLACE_FIRST:
            /*
             * 亚军 -> 冠军（先前送，再下降）：
             *   -> 先升到现有 LIFT_PODIUM_FIRST_APPROACH_MM 并等待到位；
             *   -> 保持该规避高度移动到冠军区域；
             *   -> PD15选冠军奖杯 + 精确Yaw0；
             *   -> 在规避高度K230找圆并移动到圆心；
             *   -> 仍保持高位，先执行现有K230固定机械前送；
             *   -> 前送到位后底盘停车，只下降到现有放杯高度；
             *   -> 高度到位后直接停留并直退，不再二次前送。
             */
#if COMP_ENABLE_LIFT
            if ((Lift_StartGotoMM(LIFT_PODIUM_FIRST_APPROACH_MM) == false) ||
                (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false))
            { Route_EnterError(24U); break; }
#endif

            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(25U); break; }

            if (Route_MovePosition(T2_LOCAL_SECOND_TO_FIRST_X_MM,
                                   T2_LOCAL_SECOND_TO_FIRST_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(26U); break; }

            if (AlignTrophyAtCurrentHeight(PODIUM_FIRST) == 0U)
            { Route_EnterError(28U); break; }

            /* 高位先把奖杯送进奖台范围，避免低位前送时后排奖杯碰台。 */
            if (K230FinalPlacePush() == 0U)
            { Route_EnterError(28U); break; }

#if COMP_ENABLE_LIFT
            if ((Lift_StartGotoMM(LIFT_PODIUM_FIRST_HEIGHT_MM) == false) ||
                (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false))
            { Route_EnterError(27U); break; }
#endif

            /* 前送已经完成，这里只停留并后退。 */
            if (PlaceAlignedTrophy(PODIUM_FIRST, 0U) == 0U)
            { Route_EnterError(28U); break; }

            Route_ChangeState(COMP_STATE_T2_PLACE_THIRD);
            break;

        case COMP_STATE_T2_PLACE_THIRD:
            /*
             * 冠军 -> 季军最终流程：
             *
             *   冠军放完时保持L42；
             *   -> 先确认Yaw=0°；
             *   -> 保持L42纯左移到季军位置；
             *   -> 到季军位置后再下降 L42 -> L0；
             *   -> L0到位以后K230 + 放季军。
             *
             * 不能在冠军位置提前降L0，避免低位横移碰奖台。
             */
            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(29U); break; }

            if (Route_MovePosition(T2_LOCAL_FIRST_TO_THIRD_X_MM,
                                   T2_LOCAL_FIRST_TO_THIRD_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(30U); break; }

#if COMP_ENABLE_LIFT
            if ((Lift_StartGotoMM(LIFT_PODIUM_THIRD_HEIGHT_MM) == false) ||
                (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false))
            { Route_EnterError(31U); break; }
#endif

            /*
             * 到这里已经到季军点且L0到位。
             * PlaceTrophy内部此时才执行唯一一次Yaw=0°，紧接着K230。
             */
            if (PlaceTrophy(PODIUM_THIRD) == 0U)
            { Route_EnterError(32U); break; }

            Route_ChangeState(COMP_STATE_T1_GO_QR);
            break;

        case COMP_STATE_T1_GO_QR:
#if COMP_ENABLE_TASK1_QR
            /*
             * 从季军区开始去任务一二维码前清一次旧缓存；
             * 接近二维码过程中收到的第一帧不再在正式读取时被清掉。
             */
            MJ6000_ClearRx();
#endif
#if COMP_ENABLE_LIFT
            if ((Lift_StartGotoMM(LIFT_GROUND_HEIGHT_MM) == false) ||
                (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false))
            { Route_EnterError(33U); break; }
#endif
            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(34U); break; }

            if (Route_MovePosition(T1_LOCAL_THIRD_POST_TO_QR_X_MM,
                                   T1_LOCAL_THIRD_POST_TO_QR_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(35U); break; }

            if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
            { Route_EnterError(36U); break; }

            /*
             * Route_RotateToFast结束时底盘已经停车，不再重复发送STOP。
             * 当前点位调试只保留很短的100ms稳定时间。
             */
            if (AppControl_Delay(T1_QR_STOP_DWELL_MS) == 0U)
            { Route_EnterError(37U); break; }

            Route_ChangeState(COMP_STATE_T1_SCAN_QR);
            break;

        case COMP_STATE_T1_SCAN_QR:
            /* 上一状态已经0°停稳，执行MJ6000真实任务一二维码识别。 */
            if (ReadQrWithRetry(0U) == 0U)
            { Route_EnterError(38U); break; }

            Route_ChangeState(COMP_STATE_T1_GO_LINE);
            break;

        case COMP_STATE_T1_GO_LINE:
            /*
             * 底盘去黑线的同时提前准备Task1巡线机械姿态。
             * 这样PD14的210°和PD15第0槽稳定时间与底盘运动并行，
             * 到线后不再额外原地白等1200ms+槽位稳定时间。
             */
            Servo_Pivot_270_SetAngle(DISC_SERVO_TASK1_LINE_PICK_DEG);
            Servo_Spin_360_SetAngle(task1_configs[0U].angle_opening);

            if (Route_MovePosition(T1_LOCAL_QR_PRELINE_RIGHT_MM,
                                   -T1_LOCAL_QR_PRELINE_BACK_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(39U); break; }

            if (Route_RotateToFast(90.0f) == 0U)
            { Route_EnterError(40U); break; }

            if (ConfirmLineAtEntry() == 0U)
            { Route_EnterError(41U); break; }

            Route_ChangeState(COMP_STATE_T1_FOLLOW_COLLECT);
            break;

        case COMP_STATE_T1_FOLLOW_COLLECT:
            /*
             * 最新实车规则：
             * 新路线从下往上：底部M5 -> M4 -> M3 -> M2 -> 顶部M1。
             * PD15槽位仍按50/122/194/266/338°依次收5个；
             * 前4个从触发点继续走50mm后转+72°到GY33并同步把下一空槽送到开口；
             * GY33按舵机稳定时间+投票时间识色，不再依赖旧上方入口角。
             * 第5个从触发点继续走50mm后PD15反向36°到302°并立即停车。
             */
            if (FollowLineAndCollect(LINE_MISSION_TASK1) == 0U)
            {
                Route_EnterError(42U);
                break;
            }

            if (s_material_count != 5U)
            {
                Route_EnterError(43U);
                break;
            }

#if T1_BYPASS_COLOR_RANDOM_PLACE
            /*
             * 完全旁路颜色模式：5个物料机械收完后写5个唯一虚拟标签。
             */
            Task1_AssignDummyUniqueSlotColors();
#else
#if T1_COLOR_FAIL_CONTINUE_ROUTE_TEST
            /*
             * 当前先测剩下的A~E点位：
             * GY33失败/重复色不允许在这里停车。
             * 能推断第五色就先推断；仍有失败槽则临时补齐未用颜色，
             * 只保证后面的“按颜色找槽”有唯一映射，继续跑完整路线。
             */
            (void)Turnable_FinalizeTask1FifthColor();
            Task1_RepairCollectedColorsForRouteTest();
#else
            if (Turnable_FinalizeTask1FifthColor() == 0U)
            {
                Route_EnterError(44U);
                break;
            }

            if (ValidateCollectedMaterialColors() == 0U)
            {
                Route_EnterError(45U);
                break;
            }
#endif
#endif

            /* 蓝牙一次性上报5槽真实颜色，例如 #MC R,G,B,K,W */
            Task1_ReportCollectedColors();

            /*
             * 五个槽此时都有物料，任何一个槽都不能正对外开口。
             * 第五物料在巡线途中已经把slot4从340°转到304°半槽安全位；
             * 这里必须保持304°，不能再调用历史Turnable_ParkForPlacement()
             * 把某个真实槽口重新转到外面。
             */
            Servo_Spin_360_SetAngle(
                TURNABLE_TASK1_TRANSPORT_SAFE_ANGLE_DEG);
            Route_ChangeState(COMP_STATE_T1_EXIT_LINE);
            break;

        case COMP_STATE_T1_EXIT_LINE:
            Servo_Spin_360_SetAngle(TURNABLE_TASK1_TRANSPORT_SAFE_ANGLE_DEG);
            DiscServo_ToFront();

            if (s_active_module == COMP_MODULE_T1_GO_QR)
            {
                /* 从本状态单独起跑时，仍保留全量1200ms机械稳定。 */
                if (AppControl_Delay(DISC_SERVO_SETTLE_MS) == 0U)
                { Route_EnterError(46U); break; }
            }
            else
            {
                /*
                 * 正常全流程中PD14已在第5物料收完时提前开始210°->220°，
                 * 这里只留100ms小角度保险，不再固定白等250/1200ms。
                 */
                if (AppControl_Delay(100U) == 0U)
                { Route_EnterError(46U); break; }
            }

            if (Route_RotateToFast(MAP_TASK1_PLACE_YAW_DEG) == 0U)
            { Route_EnterError(47U); break; }

            Route_ChangeState(COMP_STATE_T1_PREPARE_PLACE);
            break;

        case COMP_STATE_T1_PREPARE_PLACE:
            /*
             * V2：上一状态已经把整车快速转到0°，这里不再重复Yaw=0°等待，
             * 直接用位置模式去A圆。
             */

            /*
             * 第五物料停车并Yaw=0°后，A圆应在车体右下方：
             * X正值=右移，Y负值=后退/向场地下方。
             */
            if (Route_MovePosition(T1_LOCAL_PICK5_TO_A_X_MM,
                                   T1_LOCAL_PICK5_TO_A_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(48U); break; }

            Route_ChangeState(COMP_STATE_T1_PLACE_A);
            break;

        case COMP_STATE_T1_PLACE_A:
            /*
             * A：K230 -> (100,60) -> 前送35 mm，使开口中心落到A圆心。
             */
            if (PlaceMaterialAtPositionWithRetreat(
                    0U, MATERIAL_PLACE_A_RETREAT_MM) == 0U)
            {
                switch (s_t1_place_fail_stage)
                {
                    case 1U: Route_EnterError(49U); break;
                    case 2U: Route_EnterError(50U); break;
                    case 3U: Route_EnterError(51U); break;
                    case 4U: Route_EnterError(52U); break;
                    case 5U: Route_EnterError(53U); break;
                    default: Route_EnterError(54U); break;
                }
                break;
            }
            /*
             * 放完A并退离以后PD15保持当前空槽不动；
             * 去B的底盘转场期间禁止转PD15。
             */
            Route_ChangeState(COMP_STATE_T1_PLACE_B);
            break;

        case COMP_STATE_T1_PLACE_B:
            /*
             * A放好并直退70mm后，直接斜线到B PRE。
             * 移动与放置分开报错，便于区分底盘问题还是K230/PD15问题。
             */
            if (Route_MovePosition(T1_LOCAL_A_POST_TO_B_X_MM,
                                   T1_LOCAL_A_POST_TO_B_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(55U); break; }

            if (PlaceMaterialAtPositionWithRetreat(
                    1U, MATERIAL_PLACE_B_RETREAT_MM) == 0U)
            {
                switch (s_t1_place_fail_stage)
                {
                    case 1U: Route_EnterError(56U); break;
                    case 2U: Route_EnterError(57U); break;
                    case 3U: Route_EnterError(58U); break;
                    case 4U: Route_EnterError(59U); break;
                    case 5U: Route_EnterError(60U); break;
                    default: Route_EnterError(61U); break;
                }
                break;
            }

            Route_ChangeState(COMP_STATE_T1_PLACE_C);
            break;

        case COMP_STATE_T1_PLACE_C:
            if (Route_MovePosition(T1_LOCAL_B_POST_TO_C_X_MM,
                                   T1_LOCAL_B_POST_TO_C_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(62U); break; }

            if (PlaceMaterialAtPositionWithRetreat(
                    2U, MATERIAL_PLACE_C_RETREAT_MM) == 0U)
            { Route_EnterError(63U); break; }

            Route_ChangeState(COMP_STATE_T1_PLACE_D);
            break;

        case COMP_STATE_T1_PLACE_D:
            if (Route_MovePosition(T1_LOCAL_C_POST_TO_D_X_MM,
                                   T1_LOCAL_C_POST_TO_D_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(64U); break; }

            if (PlaceMaterialAtPositionWithRetreat(
                    3U, MATERIAL_PLACE_D_RETREAT_MM) == 0U)
            { Route_EnterError(65U); break; }

            Route_ChangeState(COMP_STATE_T1_PLACE_E);
            break;

        case COMP_STATE_T1_PLACE_E:
            /*
             * E是最后一个物料。E放好后同样直退120 mm，
             * 给90 mm开口留出更稳的侧向余量；随后不再绕点，直接一条
             * 地图直线回Home。
             */
            if (Route_MovePosition(T1_LOCAL_D_POST_TO_E_X_MM,
                                   T1_LOCAL_D_POST_TO_E_Y_MM,
                                   POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
            { Route_EnterError(66U); break; }

            if (PlaceMaterialAtPositionWithRetreat(4U,
                                                   MATERIAL_PLACE_E_RETREAT_MM) == 0U)
            { Route_EnterError(67U); break; }
            Route_ChangeState(COMP_STATE_RETURN_HOME);
            break;

case COMP_STATE_RETURN_HOME:

#if COMP_ENABLE_LIFT
    /* E放完后立即启动5号升降，和E→Home底盘移动并行。 */
    if (Lift_IsBusy() == false)
    {
        if (Lift_StartGotoMM(LIFT_HOME_HEIGHT_MM) == false)
        {
            Route_EnterError(68U);
            break;
        }
    }
#endif

    /* 5号上升的同时，底盘直接E→Home。 */
    if (Route_MovePosition(T1_LOCAL_E_POST_TO_HOME_X_MM,
                           T1_LOCAL_E_POST_TO_HOME_Y_MM,
                           POS_ROUTE_RPM, POS_ROUTE_ACC) == 0U)
    {
        Route_EnterError(69U);
        break;
    }

    if (Route_RotateToFast(MAP_HOME_YAW_DEG) == 0U)
    {
        Route_EnterError(70U);
        break;
    }

#if COMP_ENABLE_LIFT
    /* 到Home后只确认升降最终完成，不再重新发一次高度命令。 */
    if (Lift_WaitUntilTarget(LIFT_MOVE_TIMEOUT_MS) == false)
    {
        Route_EnterError(71U);
        break;
    }
#endif

            DiscServo_ToRear();
            if (AppControl_Delay(DISC_SERVO_SETTLE_MS) == 0U)
            { Route_EnterError(72U); break; }
            Route_ChangeState(COMP_STATE_FINISHED);
            break;

        case COMP_STATE_FINISHED:
            AppControl_SetRunning(0U);
            break;

        case COMP_STATE_ERROR:
            AppControl_SetRunning(0U);
            break;

        default:
            Route_EnterError(73U);
            break;
    }
}

