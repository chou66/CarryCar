#ifndef __MAP_GRAPH_H
#define __MAP_GRAPH_H

#include <stdint.h>

#define NODE_NUM        9
#define INF_COST        0x3FFF

//====================================================
// 节点定义
// 节点编号使用 1~9
//====================================================
typedef struct
{
    float x;
    float y;

    // 后续可扩展：任务类型、朝向、动作参数等
    uint8_t task_id;

} MapNode_t;


//====================================================
// 地图数据
//====================================================

// 节点数组
extern MapNode_t g_nodes[NODE_NUM + 1];

// 邻接矩阵
// 下标仍然使用 1~9
extern uint16_t g_graph[NODE_NUM + 1][NODE_NUM + 1];


//====================================================
// 对外函数
//====================================================

// 根据障碍掩码重新建立地图
void MapGraph_Build(uint32_t obstacle_mask);

// 判断某条边是否连通
uint8_t MapGraph_IsConnected(uint8_t node_a, uint8_t node_b);

// 获取边代价
uint16_t MapGraph_GetCost(uint8_t node_a, uint8_t node_b);

#endif
