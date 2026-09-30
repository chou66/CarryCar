#ifndef __COMPETITION_TASK_H
#define __COMPETITION_TASK_H

#include <stdint.h>

typedef enum
{
    COMP_STATE_IDLE = 0,

    COMP_STATE_T2_INITIAL_FORWARD,
    COMP_STATE_T2_DEPLOY_DISC,
    COMP_STATE_T2_TURN_TO_QR,
    COMP_STATE_T2_GO_QR,
    COMP_STATE_T2_SCAN_QR,
    COMP_STATE_T2_GO_LINE,
    COMP_STATE_T2_FOLLOW_COLLECT,
    COMP_STATE_T2_EXIT_LINE,
    COMP_STATE_T2_PLACE_SECOND,
    COMP_STATE_T2_PLACE_FIRST,
    COMP_STATE_T2_PLACE_THIRD,

    COMP_STATE_T1_GO_LINE,
    COMP_STATE_T1_FOLLOW_COLLECT,
    COMP_STATE_T1_EXIT_LINE,
    COMP_STATE_T1_GO_QR,
    COMP_STATE_T1_SCAN_QR,
    COMP_STATE_T1_PREPARE_PLACE,
    COMP_STATE_T1_PLACE_B,
    COMP_STATE_T1_PLACE_D,
    COMP_STATE_T1_PLACE_E,
    COMP_STATE_T1_PLACE_C,
    COMP_STATE_T1_PLACE_A,

    COMP_STATE_RETURN_HOME,
    COMP_STATE_FINISHED,
    COMP_STATE_ERROR
} CompetitionState_t;

/*
 * 可独立调用的路线模块。
 * 正式全流程保留M0~M11；M0整场，M6任务一整段，M7任务一巡线，M8~M11分段调试。
 * 模块运行结束后自动停车并回到IDLE，便于继续选择下一模块。
 */
typedef enum
{
    COMP_MODULE_FULL_ROUTE = 0,
    COMP_MODULE_T2_START_TO_QR,
    COMP_MODULE_T2_QR_TO_LINE,
    COMP_MODULE_T2_LINE_COLLECT,
    COMP_MODULE_T2_EXIT_TO_PODIUM,
    COMP_MODULE_T2_PODIUMS,
    COMP_MODULE_T1_GO_LINE,
    COMP_MODULE_T1_LINE_COLLECT,
    COMP_MODULE_T1_GO_QR,
    COMP_MODULE_T1_PLACE_A_B,
    COMP_MODULE_T1_PLACE_C_D_E,
    COMP_MODULE_RETURN_HOME,
    COMP_MODULE_COUNT
} CompetitionModule_t;

void CompetitionTask_Init(void);
void CompetitionTask_Start(void);
uint8_t CompetitionTask_StartModule(CompetitionModule_t module);
uint8_t CompetitionTask_StartSelectedModule(void);
void CompetitionTask_Update(void);
void CompetitionTask_Stop(void);

CompetitionState_t CompetitionTask_GetState(void);

void CompetitionTask_SelectModule(CompetitionModule_t module);
void CompetitionTask_SelectNextModule(void);
void CompetitionTask_SelectPreviousModule(void);
CompetitionModule_t CompetitionTask_GetSelectedModule(void);
CompetitionModule_t CompetitionTask_GetActiveModule(void);
uint8_t CompetitionTask_IsRunning(void);
uint8_t CompetitionTask_IsInitialized(void);
const char *CompetitionTask_GetLastError(void);

#endif
