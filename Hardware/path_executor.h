#ifndef __PATH_EXECUTOR_H
#define __PATH_EXECUTOR_H

#include <stdint.h>
#include "path_planner.h"


typedef enum
{
    PATH_EXEC_IDLE = 0,
    PATH_EXEC_RUNNING,
    PATH_EXEC_PAUSED,
    PATH_EXEC_DONE,
    PATH_EXEC_ERROR

} PathExecState_t;


typedef struct
{
    PathExecState_t state;

    /* 当前正在执行的完整路径 */
    PathResult_t path;

    /*
     * 当前节点在 path.node[] 中的位置。
     *
     * 例如：
     * path = 9 -> 8 -> 5 -> 2 -> 3
     *
     * index = 0:
     * current = 9
     * next    = 8
     *
     * index = 1:
     * current = 8
     * next    = 5
     */
    uint8_t index;

    uint8_t current_node;
    uint8_t next_node;

} PathExecutor_t;


/* 初始化 */
void PathExecutor_Init(PathExecutor_t *exec);


/*
 * 装载并开始执行一条路径。
 *
 * 返回：
 * 1 = 成功
 * 0 = 路径非法
 */
uint8_t PathExecutor_Start(PathExecutor_t *exec,
                           const PathResult_t *path);


/*
 * 通知 Executor：
 *
 * current_node -> next_node
 *
 * 这一段已经实际完成。
 *
 * 返回：
 * 1 = 状态成功推进
 * 0 = 当前不能推进
 */
uint8_t PathExecutor_EdgeReached(PathExecutor_t *exec);


/* 暂停 */
void PathExecutor_Pause(PathExecutor_t *exec);


/* 继续 */
void PathExecutor_Resume(PathExecutor_t *exec);


/* 查询状态 */
PathExecState_t PathExecutor_GetState(
    const PathExecutor_t *exec
);


/* 当前节点 */
uint8_t PathExecutor_GetCurrentNode(
    const PathExecutor_t *exec
);


/*
 * 下一节点。
 *
 * 如果当前没有下一节点，返回0。
 */
uint8_t PathExecutor_GetNextNode(
    const PathExecutor_t *exec
);


/*
 * 当前路径是否已经完成
 */
uint8_t PathExecutor_IsDone(
    const PathExecutor_t *exec
);


#endif
