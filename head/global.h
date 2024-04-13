#ifndef CODECRAFTSDK_GLOBAL_H
#define CODECRAFTSDK_GLOBAL_H

#ifdef __APPLE__
#include <queue>
#include <vector>
#include <cstdio>
#include <algorithm>
#include <set>
#include <random>
#include <cstring>
#include <ctime>
#include <fstream>
#include <thread>
#include <mutex>
#include <cassert>
#include <cmath>
#else
#include <bits/stdc++.h>
#endif

using namespace std;

inline const int MAX_FRAME = 15000;

inline const unsigned MAX_ROBOT_NUM = 100;
extern unsigned int RobotNum;

inline const unsigned int MAX_BERTH_NUM = 10;
extern unsigned int BerthNum;

inline const int MAX_BOAT_NUM = 100;
extern unsigned int BoatNum;

inline const int MAX_DELIVERY_NUM = 10;
extern unsigned int DeliveryNum;

inline const int MAX_ROBOT_BUYING_NUM = 10;
extern unsigned int RobotBuyingNum; // 机器人购买点数量

inline const int MAX_BOAT_BUYING_NUM = 10;
extern unsigned int BoatBuyingNum;

inline const int N = 200;
inline const int MAX_GOODS_NUM = 150010;
inline const int MAX_LENGTH = 40000;

extern int GoodsValue;

extern int robot_buy_link_to_berth[MAX_ROBOT_BUYING_NUM];

// 可以修改的参数 每个机器人BFS找的最多路径与长度 离货物距离的比重
inline const int MAX_ROAD_NUM = 20;
inline const int MAX_ROAD_LEN = 400;
inline const double TO_GOODS_WEIGHT = 1.0;
inline const double H_VALUE_WEIGHT = 2.0;

// 最大购买机器人和船的数量，购买机器人和船的价格
extern int MAX_BUY_ROBOT_NUM;
extern int MAX_BUY_BOAT_NUM;

inline const int MAX_BUY_BOAT_NUM_ARRAY[10] = {0, 1, 1, 2, 2, 2, 2, 2, 3, 3};  //参数可修改

inline const int ROBOT_TYPE_NUM = 2;
inline const int ROBOT_BUY_MONEY[ROBOT_TYPE_NUM] = {2000, 5000};
inline const int BOAT_BUY_MONEY = 8000;

inline const int DX[4] = {-1, 1, 0, 0};     // 每个方向x轴的偏移
inline const int DY[4] = {0, 0, -1, 1};     // 每个方向y轴的偏移
inline const int REV_DIR[4] = {1, 0, 3, 2}; // 上下左右的反方向（用于BFS记录路径）：下上右左

// 0 1 2 ->
// 3 4 5

//    5 4 3
// <- 2 1 0

// ↑ 2 5
//   1 4
//   0 3

//   3 0
//   4 1
//   5 2 ↓
// 船体位置标号，核心点为0
// 使用下面的数组可以从核心点开始遍历0, 1, 2, 3, 4, 5号位置
inline const int DX_BOAT[4][6] = {{0, 0, 0, 1, 1, 1},     // 右
                           {0, 0, 0, -1, -1, -1},  // 左
                           {0, -1, -2, 0, -1, -2}, // 上
                           {0, 1, 2, 0, 1, 2}};    // 下

inline const int DY_BOAT[4][6] = {{0, 1, 2, 0, 1, 2},     // 右
                           {0, -1, -2, 0, -1, -2}, // 左
                           {0, 0, 0, 1, 1, 1},     // 上
                           {0, 0, 0, -1, -1, -1}}; // 下

// 船：0右 1左 2上 3下，顺逆时针后各个方向对应的方向
inline const int ROT_DIR[2][4] = {{3, 2, 0, 1},
                                  {2, 3, 1, 0}};
// 船：0顺时针 1逆时针 转之后核心点移动到的位置 2号位置 ：核心点变到4号位置
inline const int ROT_POS[2] = {2, 4};

// 每个方向左上角/右下角对应几号位置 0右 1左 2上 3下
inline const int BOAT_LEFT_TOP[4] = {0, 5, 2, 3};
inline const int BOAT_RIGHT_DOWN[4] = {5, 0, 3, 2};

extern int OurMoney;           // 自己算出来的金钱
extern int RobotMoney;             // 机器人拿的的钱
extern int Money, BoatCapacity, Frame; // 金钱，船的容量，当前帧数
extern unsigned int World[N][N];       // 地图
extern int WorldGoods[N][N];           // 地图上每个位置的商品

extern vector<int> BerthPath[MAX_BERTH_NUM][N][N]; // 机器人泊位到每个点的最短路径第一步的可能方向(0: 上，1: 下，2: 左，3: 右)
extern int BerthPathLength[MAX_BERTH_NUM][N][N];   // 机器人泊位到每个点的最短路径长度

extern int DeliveryToBerthTime[MAX_DELIVERY_NUM][MAX_BERTH_NUM][4]; // 交货点到港口的最短时间(每个方向)
extern int BerthToDeliveryTime[MAX_BERTH_NUM][MAX_DELIVERY_NUM][4]; // 港口到交货点最短时间(每个方向)
extern int BerthToDeliveryTimeMax[MAX_BERTH_NUM][MAX_DELIVERY_NUM]; // 港口到交货点最短时间(不知道方向，所以这个存的是四个方向的最大值)
extern int BuyingToBerthTime[MAX_BOAT_BUYING_NUM][MAX_BERTH_NUM];   // 船舶购买点到港口的最短时间(只有正方向)
extern int BerthToBerthTime[MAX_BERTH_NUM][MAX_BERTH_NUM][4];       // 港口到港口的最短时间(每个方向)
extern int BerthNearestBuying[MAX_BERTH_NUM];                       // 港口最近的购买点
extern int BerthNearestDelivery[MAX_BERTH_NUM];                     // 港口最近的交货点

// 估算的到交货点或者港口的最短时间（用于启发函数）
extern int ToBerthEstimateTime[MAX_BERTH_NUM][N][N];
extern int ToDeliveryEstimateTime[MAX_DELIVERY_NUM][N][N];

// 记录初始购买机器人数目和船的数目，用于调参
extern int InitBuyRobotNum;
extern int InitBuyBoatNum;

// 初始机器人购买点购买的机器人数量
extern int InitRobotToBuy[MAX_ROBOT_BUYING_NUM];

// 初始要去的港口、购买船的位置
extern int InitBerthToGo[MAX_BOAT_BUYING_NUM];
extern int InitBuyingToBuy[MAX_BOAT_BUYING_NUM];

extern int AllocateRobotNum[MAX_ROBOT_NUM]; // 每个购买点分配的机器人数
extern int AreaBuying[MAX_ROBOT_NUM]; // 每个购买点占据面积大小
extern int Area; // 总面积大小

extern int test;
extern int LinkMaxBoatBuying;
extern int AllocateBoatNum[MAX_BERTH_NUM];

extern vector<int> AStarSearchNodeNum; // 每次Astar搜索的节点数

extern double length; // 统计港口到虚拟点的平均


// ---------------------结构体--------------------
struct BoatRouteState
{
    int x;
    int y;
    int dir;
    int action;
    BoatRouteState() : x(0), y(0), dir(0), action(0){};
    BoatRouteState(int x, int y, int dir, int action) : x(x), y(y), dir(dir), action(action){};
};
// 保存每艘船的固定航线
extern vector<BoatRouteState> BoatRoutes[MAX_BOAT_NUM];
// 保存每艘船找不到航道的次数
extern int BoatCanNotFindRoute[MAX_BOAT_NUM];

struct Goods
{
    int x, y;     // 货物的坐标
    int val;      // 货物的价值
    int fresh;    // 货物刷新时间
    int robot_id; // 要拿他的机器人

    Goods() {}
    Goods(int x, int y, int val, int fresh)
    {
        this->x = x;
        this->y = y;
        this->val = val;
        this->fresh = fresh;
        this->robot_id = -1;
    }
} extern AllGoods[MAX_GOODS_NUM];
extern int NextGoodsIndex; // 下一个货物的编号

struct Robot
{
    int x, y;           // 机器人的坐标
    int is_goods;       // 机器是否携带货物 (0：没有，1：有, 2:两个)
    int dir;            // 机器人下一步运动的方向
    int goods_index;    // 机器人携带/想携带的货物编号
    int goods_distance; // 机器人和货物的距离
    int berth_index;    // 机器人要去的泊位编号
    int action;         // 0:get()拿货物， 1:pull()放物品, -1没有动作
    int type;           // 0:只能拿一个货，1:能拿两个货

    Robot() {}
    Robot(int x, int y, int type)
    {
        this->x = x;
        this->y = y;
        this->type = type;
        this->is_goods = 0;
        this->dir = -1;
        this->goods_index = -1;
        this->goods_distance = MAX_LENGTH;
        this->berth_index = -1;
        this->action = -1;
    }

} extern Robots[MAX_ROBOT_NUM];

struct Berth
{
    int x, y;               // 泊位的坐标
    int transport_time;     // 货物运输时间
    int velocity;           // 货物装卸速度
    int boat_id;            // 要去或者已经在泊位上的船（-1为没有）
    queue<int> goods_queue; // 泊位上的货物队列
    int total_goods_num;    // 从第1帧开始，该港口堆积的货物数量（用于估计港口的货物增长速率）
    int focus;              // 最后的标记——耶稣(0:无标记；1：有标记)

    Berth() {}
    Berth(int x, int y, int velocity)
    {
        this->x = x;
        this->y = y;
        this->velocity = velocity;
        this->boat_id = -1;
        this->total_goods_num = 0;
        this->focus = 0;
    }
} extern Berths[MAX_BERTH_NUM];

struct Boat
{
    int x, y;      // 船的位置
    int dir;       // 船的方向
    int status;    // 船的状态（0：正常行驶，1：恢复状态，2：装载状态）
    int goods_num; // 船上货物的数量

    int goods_value;    // 船上已经装了的货物价值
    int dest_delivery;  // 目的交货点
    int dest_berth;     // 目的港口
    int pre_dest_berth; // 上一个目的港口

    Boat() {}
    Boat(int x, int y, int dir, int status)
    {
        this->x = x;
        this->y = y;
        this->dir = dir;
        this->status = status;
        this->goods_num = 0;

        this->goods_value = 0;
        this->dest_delivery = -1;
        this->dest_berth = -1;
        this->pre_dest_berth = -1;
    }
} extern Boats[MAX_BOAT_NUM];

struct RobotBuying
{
    int x, y; // 机器人租赁点位置
    RobotBuying() {}
    RobotBuying(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
} extern RobotBuyings[MAX_ROBOT_BUYING_NUM];

struct BoatBuying
{
    int x, y; // 船舶租赁点位置
    BoatBuying() {}
    BoatBuying(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
} extern BoatBuyings[MAX_BOAT_BUYING_NUM];

struct Delivery
{
    int x, y; // 交货点位置
    Delivery() {}
    Delivery(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
} extern Deliveries[MAX_DELIVERY_NUM];

struct BoatStateNode
{
    int x;           // 状态位置的x坐标
    int y;           // 状态位置的y坐标
    int dir;         // 状态的方向
    int action;      // 该状态的上一步动作 -1不动 0顺时针转 1逆时针转 2前进
    bool is_on_main; // 到这个节点是否会进入1帧恢复状态（主航道）
    int g_value;     // 已经花费代价 g值
    double h_value;  // 启发代价 h值
    int list_state;  // 该节点的状态（0不存在 1在开放列表中 2在关闭列表中）

    shared_ptr<BoatStateNode> pre_state; // 上一步状态

    double f_value() const
    { // 计算f值，通过调整h值权重优化性能与最优解的平衡
        return this->g_value + H_VALUE_WEIGHT * this->h_value;
    }

    bool operator<(const BoatStateNode &o) const
    {
        return this->f_value() < o.f_value();
    }

    BoatStateNode(int x, int y, int dir, int action, bool is_on_main, int g_value, int h_value,
                  int list_state, shared_ptr<BoatStateNode> pre_state)
            : x(x), y(y), dir(dir), action(action), is_on_main(is_on_main), g_value(g_value), h_value(h_value),
              list_state(list_state), pre_state(pre_state) {}
};

struct CompareBoatStateNode
{ // 自定义比较器
    bool operator()(const shared_ptr<BoatStateNode> &a, const shared_ptr<BoatStateNode> &b) const
    {
        return a->f_value() > b->f_value();
    }
};

// 用于自定义船状态节点哈希表，考虑时间维度上的A*，以位置，方向，时间(也就是G值)作为键
struct BoatStateNodeKey
{
    int p, d, t;

    BoatStateNodeKey(int p, int d, int t) : p(p), d(d), t(t) {}

    bool operator==(const BoatStateNodeKey &o) const
    {
        return p == o.p && d == o.d && t == o.t;
    }
};

struct Road
{
    int robot_index;    // 路径对应的机器人id
    int goods_index;    // 路径对应的物品id
    int goods_distance; // 离物品的距离
    int berth_index;    // 最近港口id
    int next_dir;       // 机器人下一步方向
    double value;       // 性价比

    struct Comparator
    { // 内部类作为比较器
        bool operator()(const Road &a, const Road &b) const
        {
            return a.value < b.value;
        }
    };
    Road(int robot_index, int goods_index, int goods_distance, int berth_index, int next_dir, double value)
    {
        this->robot_index = robot_index;
        this->goods_index = goods_index;
        this->goods_distance = goods_distance;
        this->berth_index = berth_index;
        this->next_dir = next_dir;
        this->value = value;
    }
};

// 比较最佳购买
struct BestBuy
{
    int best_buy_cost = MAX_LENGTH;
    int best_buy_bot;
    int best_buy_boat;
    int best_buy_berth;

    // 重载 < 运算符，根据 best_buy_cost 比较
    bool operator<(const BestBuy &other) const
    { // 小的cost优先级高
        return this->best_buy_cost > other.best_buy_cost;
    }

    BestBuy(int cost, int bot, int boat, int berth)
    {
        this->best_buy_cost = cost;
        this->best_buy_bot = bot;
        this->best_buy_boat = boat;
        this->best_buy_berth = berth;
    }
};

struct BoatState
{
    int x;
    int y;
    int dir;
    BoatState(int x, int y, int dir) : x(x), y(y), dir(dir) {}
    BoatState() : x(0), y(0), dir(0) {}
};

namespace std
{
    template <>
    struct hash<BoatStateNodeKey>
    {
        size_t operator()(const BoatStateNodeKey &k) const
        {
            // 这里的hash<int>()可以直接使用k.p, k.d, k.t，因为它们本身就是int
            // 我们使用异或(^)和位移(<< 和 >>)来混合哈希值,位移的数量（这里选择5和10）是任意的，但应该保证它们分布在不同的位范围
            return (hash<int>()(k.p) ^ (hash<int>()(k.d) << 5) ^ (hash<int>()(k.d) >> 3)) ^ (hash<int>()(k.t) << 10);
        }
    };
}

#endif //CODECRAFTSDK_GLOBAL_H
