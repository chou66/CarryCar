#ifndef __PATH_PLANNER_H
#define __PATH_PLANNER_H

#include <stdint.h>

#define PATH_MAX_LEN    9

typedef struct
{
    // 路径节点序列
    // 例如：
    // 3 -> 2 -> 5 -> 8
    // 则 node[] = {3,2,5,8}
    uint8_t node[PATH_MAX_LEN];

    // 实际路径长度
    uint8_t length;

    // 1 = 找到路径
    // 0 = 无路可走
    uint8_t valid;

    // 总路径代价
    uint16_t total_cost;

} PathResult_t;


//====================================================
// Dijkstra寻路
//====================================================
PathResult_t PathPlanner_Dijkstra(uint8_t start_node,
                                  uint8_t target_node);

#endif
