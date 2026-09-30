#include "path_executor.h"
#include <string.h>


static uint8_t PathExecutor_NodeValid(uint8_t node)
{
    return (node >= 1U && node <= 9U);
}


void PathExecutor_Init(PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return;
    }

    memset(exec, 0, sizeof(PathExecutor_t));

    exec->state = PATH_EXEC_IDLE;
}


uint8_t PathExecutor_Start(PathExecutor_t *exec,
                           const PathResult_t *path)
{
    uint8_t i;

    if ((exec == 0) || (path == 0))
    {
        return 0;
    }

    /*
     * Dijkstra没有找到路径
     */
    if (path->valid == 0U)
    {
        exec->state = PATH_EXEC_ERROR;
        return 0;
    }

    /*
     * 长度非法
     */
    if ((path->length == 0U) ||
        (path->length > PATH_MAX_LEN))
    {
        exec->state = PATH_EXEC_ERROR;
        return 0;
    }

    /*
     * 检查路径中的所有节点
     */
    for (i = 0; i < path->length; i++)
    {
        if (!PathExecutor_NodeValid(path->node[i]))
        {
            exec->state = PATH_EXEC_ERROR;
            return 0;
        }
    }

    /*
     * 保存Dijkstra给出的路径。
     */
    exec->path = *path;

    exec->index = 0U;

    exec->current_node = path->node[0];

    /*
     * 特殊情况：
     *
     * 起点 == 终点
     *
     * 例如Dijkstra结果：
     * 5
     *
     * 不需要执行任何边。
     */
    if (path->length == 1U)
    {
        exec->next_node = 0U;
        exec->state = PATH_EXEC_DONE;

        return 1;
    }

    /*
     * 正常情况。
     */
    exec->next_node = path->node[1];

    exec->state = PATH_EXEC_RUNNING;

    return 1;
}


uint8_t PathExecutor_EdgeReached(PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return 0;
    }

    /*
     * 只有RUNNING状态才能推进。
     *
     * PAUSED / DONE / IDLE / ERROR
     * 都不能修改路径进度。
     */
    if (exec->state != PATH_EXEC_RUNNING)
    {
        return 0;
    }

    /*
     * 当前 next_node 已经实际到达。
     */
    exec->index++;

    exec->current_node =
        exec->path.node[exec->index];

    /*
     * 是否已经到达路径最后一个节点？
     */
    if (exec->index >= (exec->path.length - 1U))
    {
        exec->next_node = 0U;

        exec->state = PATH_EXEC_DONE;

        return 1;
    }

    /*
     * 还有下一段。
     */
    exec->next_node =
        exec->path.node[exec->index + 1U];

    return 1;
}


void PathExecutor_Pause(PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return;
    }

    if (exec->state == PATH_EXEC_RUNNING)
    {
        exec->state = PATH_EXEC_PAUSED;
    }
}


void PathExecutor_Resume(PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return;
    }

    if (exec->state == PATH_EXEC_PAUSED)
    {
        exec->state = PATH_EXEC_RUNNING;
    }
}


PathExecState_t PathExecutor_GetState(
    const PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return PATH_EXEC_ERROR;
    }

    return exec->state;
}


uint8_t PathExecutor_GetCurrentNode(
    const PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return 0U;
    }

    return exec->current_node;
}


uint8_t PathExecutor_GetNextNode(
    const PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return 0U;
    }

    return exec->next_node;
}


uint8_t PathExecutor_IsDone(
    const PathExecutor_t *exec)
{
    if (exec == 0)
    {
        return 0U;
    }

    return (exec->state == PATH_EXEC_DONE);
}
