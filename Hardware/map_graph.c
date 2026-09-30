#include "map_graph.h"


//====================================================
// 节点参数
//
// 实测场地：相邻节点中心距 = 0.85 m。
// 黄色禁行方块本身为 0.45 m x 0.45 m，但节点中心距包含道路宽度，
// 因此导航边的物理运动距离必须使用 0.85 m。
//====================================================
MapNode_t g_nodes[NODE_NUM + 1] =
{
    {0},
    {0.00f, 0.00f, 0},  /* N1 */
    {0.85f, 0.00f, 0},  /* N2 */
    {1.70f, 0.00f, 0},  /* N3 */
    {0.00f, 0.85f, 0},  /* N4 */
    {0.85f, 0.85f, 0},  /* N5 */
    {1.70f, 0.85f, 0},  /* N6 */
    {0.00f, 1.70f, 0},  /* N7 */
    {0.85f, 1.70f, 0},  /* N8 */
    {1.70f, 1.70f, 0}   /* N9 */
};


//====================================================
// 邻接矩阵
//====================================================
uint16_t g_graph[NODE_NUM + 1][NODE_NUM + 1];


//====================================================
// 内部函数
//====================================================
static void MapGraph_Clear(void);
static void MapGraph_AddEdge(uint8_t a, uint8_t b, uint16_t cost);
static void MapGraph_RemoveEdge(uint8_t a, uint8_t b);
static void MapGraph_BlockNode(uint8_t node);


//====================================================
// 清空地图
//====================================================
static void MapGraph_Clear(void)
{
    uint8_t i;
    uint8_t j;

    for(i = 0; i <= NODE_NUM; i++)
    {
        for(j = 0; j <= NODE_NUM; j++)
        {
            if(i == j)
            {
                g_graph[i][j] = 0;
            }
            else
            {
                g_graph[i][j] = INF_COST;
            }
        }
    }
}


//====================================================
// 添加双向边
//====================================================
static void MapGraph_AddEdge(uint8_t a, uint8_t b, uint16_t cost)
{
    g_graph[a][b] = cost;
    g_graph[b][a] = cost;
}


//====================================================
// 删除双向边
//====================================================
static void MapGraph_RemoveEdge(uint8_t a, uint8_t b)
{
    g_graph[a][b] = INF_COST;
    g_graph[b][a] = INF_COST;
}


//====================================================
// 禁用某个节点
//
// 做法：切断该节点和所有其他节点的连接
//====================================================
static void MapGraph_BlockNode(uint8_t node)
{
    uint8_t i;

    for(i = 1; i <= NODE_NUM; i++)
    {
        if(i != node)
        {
            g_graph[node][i] = INF_COST;
            g_graph[i][node] = INF_COST;
        }
    }
}


//====================================================
// 根据 obstacle_mask 建图
//
// bit0~bit8   : Node1~Node9
// bit9~bit20  : 12条边
//====================================================
void MapGraph_Build(uint32_t obstacle_mask)
{
    //------------------------------------------------
    // 1. 清空旧图
    //------------------------------------------------
    MapGraph_Clear();


    //------------------------------------------------
    // 2. 先建立完整拓扑
    //
    // 当前所有边代价都设为100
    // 后续可按真实距离修改
    //------------------------------------------------

    // 水平边
    MapGraph_AddEdge(1, 2, 850);
    MapGraph_AddEdge(2, 3, 850);

    MapGraph_AddEdge(4, 5, 850);
    MapGraph_AddEdge(5, 6, 850);

    MapGraph_AddEdge(7, 8, 850);
    MapGraph_AddEdge(8, 9, 850);

    // 垂直边
    MapGraph_AddEdge(1, 4, 850);
    MapGraph_AddEdge(4, 7, 850);

    MapGraph_AddEdge(2, 5, 850);
    MapGraph_AddEdge(5, 8, 850);

    MapGraph_AddEdge(3, 6, 850);
    MapGraph_AddEdge(6, 9, 850);


    //------------------------------------------------
    // 3. 处理9个节点障碍
    //------------------------------------------------

    if(obstacle_mask & (1UL << 0))
    {
        MapGraph_BlockNode(1);
    }

    if(obstacle_mask & (1UL << 1))
    {
        MapGraph_BlockNode(2);
    }

    if(obstacle_mask & (1UL << 2))
    {
        MapGraph_BlockNode(3);
    }

    if(obstacle_mask & (1UL << 3))
    {
        MapGraph_BlockNode(4);
    }

    if(obstacle_mask & (1UL << 4))
    {
        MapGraph_BlockNode(5);
    }

    if(obstacle_mask & (1UL << 5))
    {
        MapGraph_BlockNode(6);
    }

    if(obstacle_mask & (1UL << 6))
    {
        MapGraph_BlockNode(7);
    }

    if(obstacle_mask & (1UL << 7))
    {
        MapGraph_BlockNode(8);
    }

    if(obstacle_mask & (1UL << 8))
    {
        MapGraph_BlockNode(9);
    }


    //------------------------------------------------
    // 4. 处理12条边障碍
    //------------------------------------------------

    // bit9：1-2
    if(obstacle_mask & (1UL << 9))
    {
        MapGraph_RemoveEdge(1, 2);
    }

    // bit10：2-3
    if(obstacle_mask & (1UL << 10))
    {
        MapGraph_RemoveEdge(2, 3);
    }

    // bit11：4-5
    if(obstacle_mask & (1UL << 11))
    {
        MapGraph_RemoveEdge(4, 5);
    }

    // bit12：5-6
    if(obstacle_mask & (1UL << 12))
    {
        MapGraph_RemoveEdge(5, 6);
    }

    // bit13：7-8
    if(obstacle_mask & (1UL << 13))
    {
        MapGraph_RemoveEdge(7, 8);
    }

    // bit14：8-9
    if(obstacle_mask & (1UL << 14))
    {
        MapGraph_RemoveEdge(8, 9);
    }

    // bit15：1-4
    if(obstacle_mask & (1UL << 15))
    {
        MapGraph_RemoveEdge(1, 4);
    }

    // bit16：4-7
    if(obstacle_mask & (1UL << 16))
    {
        MapGraph_RemoveEdge(4, 7);
    }

    // bit17：2-5
    if(obstacle_mask & (1UL << 17))
    {
        MapGraph_RemoveEdge(2, 5);
    }

    // bit18：5-8
    if(obstacle_mask & (1UL << 18))
    {
        MapGraph_RemoveEdge(5, 8);
    }

    // bit19：3-6
    if(obstacle_mask & (1UL << 19))
    {
        MapGraph_RemoveEdge(3, 6);
    }

    // bit20：6-9
    if(obstacle_mask & (1UL << 20))
    {
        MapGraph_RemoveEdge(6, 9);
    }
}


//====================================================
// 判断两个节点是否连通
//====================================================
uint8_t MapGraph_IsConnected(uint8_t node_a, uint8_t node_b)
{
    if(node_a == 0 || node_a > NODE_NUM)
    {
        return 0;
    }

    if(node_b == 0 || node_b > NODE_NUM)
    {
        return 0;
    }

    return (g_graph[node_a][node_b] < INF_COST);
}


//====================================================
// 获取边代价
//====================================================
uint16_t MapGraph_GetCost(uint8_t node_a, uint8_t node_b)
{
    if(node_a == 0 || node_a > NODE_NUM)
    {
        return INF_COST;
    }

    if(node_b == 0 || node_b > NODE_NUM)
    {
        return INF_COST;
    }

    return g_graph[node_a][node_b];
}
