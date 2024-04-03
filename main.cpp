#ifdef _WIN32
#include <bits/stdc++.h>
#include <iostream>
#include <fstream>
#else
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
#endif

// #define DEBUG

using namespace std;

const int MAX_FRAME = 15000;

const unsigned MAX_ROBOT_NUM = 100;
unsigned int RobotNum = 0;

const unsigned int MAX_BERTH_NUM = 10;
unsigned int BerthNum = 0;

const int MAX_BOAT_NUM = 100;
unsigned int BoatNum = 0;

const int MAX_DELIVERY_NUM = 100;
unsigned int DeliveryNum = 0;

const int MAX_ROBOT_BUYING_NUM = 100;
unsigned int RobotBuyingNum = 0;

const int MAX_BOAT_BUYING_NUM = 100;
unsigned int BoatBuyingNum = 0;

const int N = 200;
const int MAX_GOODS_NUM = 150010;
const int MAX_LENGTH = 40000;

const int MAX_ROAD_NUM = 10;
const int MAX_ROAD_LEN = 350;
const double TO_GOODS_WHIGHT = 3.0;

const int DX[4] = {-1, 1, 0, 0};     // 每个方向x轴的偏移
const int DY[4] = {0, 0, -1, 1};     // 每个方向y轴的偏移
const int REV_DIR[4] = {1, 0, 3, 2}; // 上下左右的反方向（用于BFS记录路径）：下上右左

// 4 2 0 ->
// 5 3 1

//    1 3 5
// <- 0 2 4

// ↑ 0 1
//   2 3
//   4 5

//   5 4
//   3 2
//   1 0 ↓
// 船体位置标号，核心点为4
// 使用下面的数组可以从核心点开始遍历0, 1, 2, 3, 4, 5号位置
const int DX_BOAT[4][6] = {{0, 1, 0, 1, 0, 1},     // 右
                           {0, -1, 0, -1, -1},     // 左
                           {-2, -2, -1, -1, 0, 0}, // 上
                           {2, 2, 1, 1, 0, 0}};    // 下

const int DY_BOAT[4][6] = {{2, 2, 1, 1, 0, 0},     // 右
                           {-2, -2, -1, -1, 0, 0}, // 左
                           {0, 1, 0, 1, 0, 1},     // 上
                           {0, -1, 0, -1, 0, -1}}; // 下
// 顺时针转：核心点变到0号位置
// 逆时针转：核心点变到3号位置
// 船：0右 1左 2上 3下，顺逆时针对应的方向
const int CLOCKWISE[4] = {3, 2, 0, 1};
const int ANTI_CLOCKWISE[4] = {2, 3, 1, 0};

int OurMoney = 0;               // 自己算出来的金钱
int RobotMoney = 0;             // 机器人拿的的钱
int Money, BoatCapacity, Frame; // 金钱，船的容量，当前帧数
unsigned int World[N][N];       // 地图
int WorldGoods[N][N];           // 地图上每个位置的商品

vector<int> BerthPath[MAX_BERTH_NUM][N][N]; // 机器人泊位到每个点的最短路径第一步的可能方向(0: 上，1: 下，2: 左，3: 右)
int BerthPathLength[MAX_BERTH_NUM][N][N];   // 机器人泊位到每个点的最短路径长度

int DeliveryToBerthTime[MAX_DELIVERY_NUM][MAX_BERTH_NUM];  // 交货点到港口的最短时间
int BuyingToBerthTime[MAX_BOAT_BUYING_NUM][MAX_BERTH_NUM]; // 船舶购买点到港口的最短时间

struct Goods
{
    int x, y; // 货物的坐标
    int val;  // 货物的价值

    Goods() {}
    Goods(int x, int y, int val)
    {
        this->x = x;
        this->y = y;
        this->val = val;
    }
} AllGoods[MAX_GOODS_NUM];
int NextGoodsIndex = 1; // 下一个货物的编号

struct Robot
{
    int x, y;           // 机器人的坐标
    int is_goods;       // 机器是否携带货物 (0：没有，1：有)
    int dir;            // 机器人下一步运动的方向
    int goods_index;    // 机器人携带/想携带的货物编号
    int goods_distance; // 机器人和货物的距离
    int berth_index;    // 机器人要去的泊位编号
    int action;         // 0:get()拿货物， 1:pull()放物品, -1没有动作

    Robot() {}
    Robot(int x, int y)
    {
        this->x = x;
        this->y = y;
        this->is_goods = 0;
        this->dir = -1;
        this->goods_index = -1;
        this->goods_distance = MAX_LENGTH;
        this->berth_index = -1;
        this->action = -1;
    }

} Robots[MAX_ROBOT_NUM];

struct Berth
{
    int x, y;               // 泊位的坐标
    int transport_time;     // 货物运输时间
    int loading_speed;      // 货物装卸速度
    int boat_num;           // 泊位上的船（正要去这里装货或者正在这里装货）的数量
    queue<int> goods_queue; // 泊位上的货物队列
    int total_goods_num;    // 从第一帧开始，该港口堆积的货物数量（用于估计港口的货物增长速率）
    int focus;              // 最后的标记——耶稣(0:无标记；1：有标记)

    Berth() {}
    Berth(int x, int y, int loading_speed)
    {
        this->x = x;
        this->y = y;
        this->loading_speed = loading_speed;
        this->boat_num = 0;
        this->total_goods_num = 0;
        this->focus = 0;
    }
} Berths[MAX_BERTH_NUM];

struct Boat
{
    int x, y;      // 船的位置
    int dir;       // 船的方向
    int status;    // 船的状态（0：正常行驶，1：恢复状态，2：装载状态）
    int goods_num; // 船上货物的数量

    int goods_value; // 船上已经装了的货物价值
    int dest;        // 船的目的地

    Boat() {}
    Boat(int x, int y, int dir, int status, int goods_num)
    {
        this->x = x;
        this->y = y;
        this->dir = dir;
        this->status = status;
        this->goods_num = 0;

        this->goods_value = 0;
        this->dest = 0;
    }
} Boats[MAX_BOAT_NUM];

struct RobotBuying
{
    int x, y; // 机器人租赁点位置
    RobotBuying() {}
    RobotBuying(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
} RobotBuyings[MAX_ROBOT_BUYING_NUM];

struct BoatBuying
{
    int x, y; // 船舶租赁点位置
    BoatBuying() {}
    BoatBuying(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
} BoatBuyings[MAX_BOAT_BUYING_NUM];

struct Delivery
{
    int x, y; // 交货点位置
    Delivery() {}
    Delivery(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
} Deliveries[MAX_DELIVERY_NUM];

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

// 海洋部分 10000000
// 主航道   11000000
// 靠泊区   11100000
// 泊位	    11010101
// 陆地部分 00000001
// 陆地障碍 00000011
// 主干道   00000101

// 00000001  '.' ： 空地
// 00000101  '>' ： 陆地主干道
// 10000000  '*' ： 海洋
// 11000000  '~' ： 海洋主航道
// 00000011  '#' ： 障碍
// 00000101  'R' ： 机器人购买地块，同时该地块也是主干道
// 11000000  'S' ： 船舶购买地块，同时该地块也是主航道
// 11010101  'B' ： 泊位
// 11100000  'K' ： 靠泊区
// 10000001  'C' ： 海陆立体交通地块
// 11000101  'c' ： 海陆立体交通地块，同时为主干道和主航道
// 11000000  'T' ： 交货点

// 检查坐标(x, y)是否为机器人能走的合法块 xxxxxx01
bool IsLandValid(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00000011) == 0b00000001);
}

// 检查坐标(x, y)是否为船舶的合法块 1xxxxxxx
bool IsOceanValid(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b10000000) == 0b10000000);
}

// 判断机器人位置是否在泊位内 xxx1xxxx，如果在返回港口编号，否则返回-1
int IsOnBerth(int x, int y)
{
    if ((x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00010000) == 0b00010000))
    {
        return int(World[x][y] >> 8);
    }
    else
    {
        return -1;
    }
}

// 判断船位置是否在泊位停靠范围内 xxx1xxxx xx1xxxxx，如果在返回港口编号，否则返回-1
int IsInBerthRange(int x, int y)
{
    if ((x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00110000) != 0))
    {
        return int(World[x][y] >> 8);
    }
    else
    {
        return -1;
    }
}

// 判断机器人位置是否在主干道上 xxxxx1xx
bool IsOnMainRoad(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00000100) == 0b00000100);
}

// 判断船是否在主航道上 x1xxxxxx
bool IsOnMainChannel(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b01000000) == 0b01000000);
}

// 判断机器人是否在商品处,如果在则返回物品ID，否则返回0
int IsOnGoods(int x, int y)
{
    return WorldGoods[x][y];
}

// 判断位置(x, y)与所有港口的可达性
int CanGetBerth(int x, int y)
{
    int can_get = 0;
    for (int b = 0; b < BerthNum; b++)
    { // 对每个港口进行检查
        if (BerthPathLength[b][x][y] >= 0)
        {
            can_get++;
        }
    }
    return can_get;
}

// 找到所有被标记港口中的最短距离港口
int LastMinBerth(int x, int y)
{
    int min_berth = -1; // 最短标记港口；无标记就返回-1
    int min_len = MAX_LENGTH;
    for (int i = 0; i < BerthNum; i++)
    {
        if (Berths[i].focus == 1)
        {
            if (BerthPathLength[i][x][y] >= 0 && BerthPathLength[i][x][y] < min_len)
            {
                min_len = BerthPathLength[i][x][y];
                min_berth = i;
            }
        }
    }
    return min_berth;
}

// 地图的点到港口最短路径的可能方向（按照当前帧随机取一个）
int PsbDirToBerth(int berth_id, int x, int y)
{
    if (BerthPath[berth_id][x][y].size() > 0)
    {
        return BerthPath[berth_id][x][y][Frame % BerthPath[berth_id][x][y].size()];
    }
    return -1;
}

// 计算物品性价比，并确定最接近的港口
double CalculateGoodsValue(int goods_index, int step_num, int &to_berth_index)
{
    int goods_value;               // 物品价值
    int goods_x, goods_y;          // 物品坐标
    int to_berth_len = MAX_LENGTH; // 物品到港口距离
    goods_value = AllGoods[goods_index].val;
    goods_x = AllGoods[goods_index].x;
    goods_y = AllGoods[goods_index].y;

    // 确定最近港口
    to_berth_index = LastMinBerth(goods_x, goods_y);
    if (to_berth_index == -1)
    { // 没标记
        for (int i = 0; i < BerthNum; i++)
        {
            int length = BerthPathLength[i][goods_x][goods_y];
            if (length >= 0 && length <= to_berth_len)
            {
                to_berth_len = length;
                to_berth_index = i;
            }
        }
    }
    else
    { // 有标记
        to_berth_len = BerthPathLength[to_berth_index][goods_x][goods_y];
    }

    double cost_value = (double)goods_value / (step_num * TO_GOODS_WHIGHT + to_berth_len);
    return cost_value;
}

// 方向随机排列
vector<int> GetRandomDirection()
{
    vector<int> random_dir = {0, 1, 2, 3};
    random_device rd;
    mt19937 g(rd());
    shuffle(random_dir.begin(), random_dir.end(), g);
    return random_dir;
}

// 记录港口和靠泊区对应哪个港口
void InitBerthInfo(int id)
{ // 对于id号港口，BFS更新其连通的靠泊区和泊位对应的地图表示
    queue<pair<int, int>> q;
    q.push({Berths[id].x, Berths[id].y});
    World[Berths[id].x][Berths[id].y] += (id << 8);
    while (!q.empty())
    {
        pair<int, int> cur_pos = q.front();
        q.pop();
        for (int i = 0; i < 4; i++)
        {
            int dir = i;
            int nx = cur_pos.first + DX[dir];
            int ny = cur_pos.second + DY[dir];
            int berth_id = IsInBerthRange(nx, ny);
            if (berth_id != -1 && berth_id != id)
            { // 更新地图表示
                q.push({nx, ny});
                World[nx][ny] += (id << 8);
            }
        }
    }
}

// 读取地图和初始信息
void InitWorldInfo()
{ // 读取地图
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            char ch;
            scanf("%c", &ch);
            if (ch == '.')
            { // 空地
                World[i][j] = 0b00000001;
            }
            else if (ch == '>')
            { // 陆地主干道
                World[i][j] = 0b00000101;
            }
            else if (ch == '*')
            { // 海洋
                World[i][j] = 0b10000000;
            }
            else if (ch == '~')
            { // 海洋主航道
                World[i][j] = 0b11000000;
            }
            else if (ch == '#')
            { // 陆地障碍
                World[i][j] = 0b00000011;
            }
            else if (ch == 'R')
            { // 机器人购买地块
                World[i][j] = 0b00000101;
                // 记录机器人购买地块
                RobotBuyings[RobotBuyingNum] = RobotBuying(i, j);
                RobotBuyingNum++;
            }
            else if (ch == 'S')
            { // 船舶购买地块
                World[i][j] = 0b11000000;
                // 记录船舶购买地块
                BoatBuyings[BoatBuyingNum] = BoatBuying(i, j);
                BoatBuyingNum++;
            }
            else if (ch == 'B')
            { // 泊位
                World[i][j] = 0b11010101;
            }
            else if (ch == 'K')
            { // 靠泊区
                World[i][j] = 0b11100000;
            }
            else if (ch == 'C')
            { // 海陆立体交通地块
                World[i][j] = 0b10000001;
            }
            else if (ch == 'c')
            { // 海陆立体交通地块，同时为主干道和主航道
                World[i][j] = 0b11000101;
            }
            else if (ch == 'T')
            { // 交货点
                World[i][j] = 0b11000000;
                // 记录交货点
                Deliveries[DeliveryNum] = Delivery(i, j);
                DeliveryNum++;
            }
        }
        char ch;
        scanf("%c", &ch);
    }

    // 读入泊位的信息
    scanf("%d", &BerthNum);
    for (int i = 0; i < BerthNum; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d", &Berths[id].x, &Berths[id].y, &Berths[id].loading_speed);
    }
    // 通过泊位信息再次更新地图（将地图的港口对应的ID更新）
    for (int i = 0; i < BerthNum; i++)
    { // 对每一个泊位，更新其连通区域对应的泊位编号
        InitBerthInfo(i);
    }

    // 读入船的容量
    scanf("%d", &BoatCapacity);
    // 读入ok
    char ok[100];
    scanf("%s", ok);
}

// BFS存储每个泊位到地图每个点最短路径和距离
void InitToBerthBFS()
{
    // 初始化所有泊位到每个点的最短路径长度，均为-1（memset支持初始化-1）
    memset(BerthPathLength, -1, sizeof(BerthPathLength));
    // 对每个泊位开始做BFS
    for (int b = 0; b < BerthNum; b++)
    {
        int berth_x = Berths[b].x;
        int berth_y = Berths[b].y;
        queue<pair<int, int>> q;
        // 将本位置加入
        q.push({berth_x, berth_y});
        BerthPathLength[b][berth_x][berth_y] = 0;
        while (!q.empty())
        { // 从队列中取出一个点
            pair<int, int> cur_pos = q.front();
            q.pop();
            // 四个方向随机遍历
            vector<int> random_dir = GetRandomDirection();
            for (int i = 0; i < 4; i++)
            { // 遍历到下一个点
                int dir = random_dir[i];
                // int dir = i;
                int nx = cur_pos.first + DX[dir];
                int ny = cur_pos.second + DY[dir];
                if (IsLandValid(nx, ny))
                { // 判断该点机器人是否可以到达
                    int next_path_length = BerthPathLength[b][cur_pos.first][cur_pos.second] + 1;
                    if (BerthPathLength[b][nx][ny] < 0)
                    { // 这个点之前没有遍历过
                        if (IsOnBerth(nx, ny) == b)
                        { // 如果在当前泊位内，路径长度为0
                            BerthPathLength[b][nx][ny] = 0;
                        }
                        else
                        { // 否则路径加1
                            BerthPathLength[b][nx][ny] = next_path_length;
                        }
                        // 记录路径的方向
                        BerthPath[b][nx][ny].push_back(REV_DIR[dir]);
                        // 将该点加入队列
                        q.push({nx, ny});
                    }
                    else if (BerthPathLength[b][nx][ny] == next_path_length)
                    { // 这个点之前遍历过，如果路径长度一样就加入方向
                        BerthPath[b][nx][ny].push_back(REV_DIR[dir]);
                    }
                    else if (BerthPathLength[b][nx][ny] > next_path_length)
                    { // 这个点的路径更短，清空之前的路径方向，把新方向加进去
                        BerthPath[b][nx][ny].clear();
                        BerthPath[b][nx][ny].push_back(REV_DIR[dir]);
                        BerthPathLength[b][nx][ny] = next_path_length;
                    }
                }
            }
        }
    }
}

// 记录交货点到泊位之间的最短时间
void InitDeliveryToBerth()
{
    //
}

// 记录船舶购买点到泊位的最短时间
void InitBuyingToBerth()
{
    //
}

// 初始化函数
void Init()
{
    InitWorldInfo();
    InitToBerthBFS();
    InitDeliveryToBerth();
    InitBuyingToBerth();
    // 输出OK
    printf("OK\n");
    fflush(stdout);
}

// 更新每一帧输入信息
void Input()
{
    // 更新当前帧数和分数
    scanf("%d%d", &Frame, &Money);
    // 变化的物品
    int changed_goods_num;
    scanf("%d", &changed_goods_num);
    for (int i = 0; i < changed_goods_num; i++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        if (val == 0)
        { // 消失或者被拿走的物品
            WorldGoods[x][y] = 0;
        }
        else
        { // 新增的物品
            AllGoods[NextGoodsIndex] = Goods(x, y, val);
            WorldGoods[x][y] = NextGoodsIndex;
            NextGoodsIndex++;
        }
    }
    // 同步机器人的位置
    int robot_num;
    scanf("%d", &robot_num);
    for (int i = 0; i < robot_num; i++)
    {
        int id, is_goods, x, y;
        scanf("%d%d%d%d", &id, &is_goods, &x, &y);
        // 更新机器人的状态
        Robots[id].x = x;
        Robots[id].y = y;
        Robots[id].is_goods = is_goods;
        // 重置动作，商品id以及方向
        Robots[id].action = -1;
        Robots[id].dir = -1;
    }
    // 同步船的状态和位置
    int boat_num;
    for (int i = 0; i < boat_num; i++)
    {
        int id, goods_num, x, y, dir, status;
        scanf("%d%d%d%d%d\n", &id, &goods_num, &x, &y, &dir, &status);

        Boats[id].goods_num = goods_num;
        Boats[id].x = x;
        Boats[id].y = y;
        Boats[id].dir = dir;
        Boats[id].status = status;
    }

    // 读取ok
    char ok[100];
    scanf("%s", ok);
}

// 没拿物品的机器人之间比较要拿物品的性价比
bool NoGoodsRobotsCompair(int ri, int rj)
{
    int goodsi = Robots[ri].goods_index; // 机器人i要拿的物品
    int goodsj = Robots[rj].goods_index; // 机器人j要拿的物品
    // 考虑没拿物品的机器人本身没有匹配到物品（但是在移动）
    if (goodsi < 0 && goodsj >= 0)
    { // i没有要拿的物品而j有，则j优先
        return false;
    }
    else if (goodsi >= 0 && goodsj < 0)
    { // j没有要拿的物品而i有，则i优先
        return true;
    }
    else if (goodsi < 0 && goodsj < 0)
    { // 都没有要拿的物品则，id大的优先
        return ri > rj;
    }
    double ri_val = (double)AllGoods[goodsi].val / (Robots[ri].goods_distance * TO_GOODS_WHIGHT + BerthPathLength[Robots[ri].berth_index][AllGoods[goodsi].x][AllGoods[goodsi].y]); // 机器人i要拿物品的性价比
    double rj_val = (double)AllGoods[goodsj].val / (Robots[rj].goods_distance * TO_GOODS_WHIGHT + BerthPathLength[Robots[rj].berth_index][AllGoods[goodsj].x][AllGoods[goodsj].y]); // 机器人j要拿物品的性价比
    if (ri_val >= rj_val)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// 都拿物品的机器人之间比较所拿物品的性价比
bool GetGoodsRobotsCompair(int ri, int rj)
{
    int goodsi = Robots[ri].goods_index;                                                                                // 机器人i拿的物品
    int goodsj = Robots[rj].goods_index;                                                                                // 机器人j拿的物品
    double ri_val = (double)AllGoods[goodsi].val / BerthPathLength[Robots[ri].berth_index][Robots[ri].x][Robots[ri].y]; // 机器人i物品的性价比
    double rj_val = (double)AllGoods[goodsj].val / BerthPathLength[Robots[rj].berth_index][Robots[rj].x][Robots[rj].y]; // 机器人j物品的性价比
    if (ri_val >= rj_val)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// 机器人多线程BFS
void RobotBFSToGoods(int ri, priority_queue<Road, vector<Road>, Road::Comparator> &roads_pq, mutex &roads_pq_mutex)
{
    int robot_x = Robots[ri].x;
    int robot_y = Robots[ri].y;
    // 定义BFS最短路径及长度，坐标值以及队列
    vector<vector<int>> goods_path(200, vector<int>(200, -1));
    vector<vector<int>> goods_path_length(200, vector<int>(200, -1));
    queue<pair<int, int>> q;
    int ri_road_num = 0;

    q.push({robot_x, robot_y});
    goods_path_length[robot_x][robot_y] = 0;
    while (!q.empty())
    {
        // 从队列中取出一个点
        pair<int, int> cur_pos = q.front();
        q.pop();

        int goods_id = IsOnGoods(cur_pos.first, cur_pos.second);
        int cur_path_length = goods_path_length[cur_pos.first][cur_pos.second];
        if (goods_id > 0)
        { // 如果现在的位置有物品，计算最近港口和性价比，并将路径加入
            int to_berth_index = -1;
            double value = CalculateGoodsValue(goods_id, cur_path_length, to_berth_index);
            {
                lock_guard<mutex> lock(roads_pq_mutex); // 锁住互斥锁
                roads_pq.push(Road(ri, goods_id, cur_path_length, to_berth_index, goods_path[cur_pos.first][cur_pos.second], value));
            }
            ri_road_num++;
        }
        // 限制搜索的范围以控制时间
        if (cur_path_length >= MAX_ROAD_LEN || ri_road_num > MAX_ROAD_NUM)
        {
            break;
        }
        // 四个方向随机遍历
        for (int i = 0; i < 4; i++)
        { // 遍历到下一个点
            int dir = (ri + i) % 4;
            int nx = cur_pos.first + DX[dir];
            int ny = cur_pos.second + DY[dir];
            if (IsLandValid(nx, ny) && goods_path_length[nx][ny] < 0)
            { // 判断该点是否可以到达(没有越界&&为空地或者泊位&&之前没有到达过)
                // 路径长度+1
                goods_path_length[nx][ny] = cur_path_length + 1;
                if (cur_path_length == 0)
                { // 记录路径的方向(只记录路径第一步的方向)
                    goods_path[nx][ny] = dir;
                }
                else
                {
                    goods_path[nx][ny] = goods_path[cur_pos.first][cur_pos.second];
                }
                // 将该点加入队列
                q.push({nx, ny});
            }
        }
    }
}

// 机器人调度
void RobotDsipatchGreedy()
{
    vector<int> robots_match(RobotNum, 0); // 机器人是否匹配
    set<int> goods_match;                  // 被匹配的商品
    int robot_match_num = 0;               // 被匹配的机器人数

    for (int ri = 0; ri < RobotNum; ri++)
    { // 对于每个机器人
        // if (Robots[ri].is_dead == 1 || Robots[ri].status == 0)
        // { // 如果机器人不能动（被困或者处于恢复状态），就不考虑
        //     robot_match_num++;
        //     robots_match[ri] = 1;
        //     Robots[ri].dir = -1;
        //     continue;
        // }
        if (Robots[ri].is_goods == 1)
        { // 如果机器人拿着物品就直接找港口
            int now_berth_id = IsOnBerth(Robots[ri].x, Robots[ri].y);
            if (now_berth_id >= 0 && now_berth_id == Robots[ri].berth_index)
            { // 如果到达港口，放下货物，继续找其他货物（要判断是不是自己要去的港口）
                RobotMoney += AllGoods[Robots[ri].goods_index].val;
                Robots[ri].action = 1;
                Berths[Robots[ri].berth_index].goods_queue.push(Robots[ri].goods_index); // 港口放入货物
                Berths[Robots[ri].berth_index].total_goods_num++;                        // 港口放过的总货物数量
                Robots[ri].goods_index = -1;
                Robots[ri].goods_distance = MAX_LENGTH;
                Robots[ri].is_goods = 0;
            }
            else
            { // 没到港口（或者不是自己要去的港口），沿着图走（先看有没有找到被标记的优先港口）
                int berth_id = LastMinBerth(Robots[ri].x, Robots[ri].y);
                // int berth_id = -1;
                if (berth_id == -1)
                { // 没标记
                    Robots[ri].dir = PsbDirToBerth(Robots[ri].berth_index, Robots[ri].x, Robots[ri].y);
                }
                else
                {                                                                                       // 有标记
                    Robots[ri].berth_index = berth_id;                                                  // 重置要去港口号
                    Robots[ri].dir = PsbDirToBerth(Robots[ri].berth_index, Robots[ri].x, Robots[ri].y); // 重置机器人方向
                }
                robot_match_num++;
                robots_match[ri] = 1;
                continue;
            }
        }
    }

    // 以上情况之外，其他机器人都要BFS找货物(使用多线程)
    priority_queue<Road, vector<Road>, Road::Comparator> roads_pq; // 维护一个Road优先队列，每次找一条最有价值的路径匹配
    mutex roads_pq_mutex;                                          // 多线程互斥锁
    vector<thread> robots_threads;                                 // 线程
    for (int ri = 0; ri < RobotNum; ri++)
    { // 对没有匹配的机器人进行BFS
        if (robots_match[ri] == 0)
        {
            robots_threads.emplace_back(RobotBFSToGoods, ri, ref(roads_pq), ref(roads_pq_mutex));
        }
    }
    // 等待所有线程结束
    for (auto &thread : robots_threads)
    {
        thread.join();
    }

    // 开始机器人和物品的匹配
    while (!roads_pq.empty())
    { // 价值最大的路径
        if (robot_match_num >= RobotNum)
        {
            break;
        }
        Road road = roads_pq.top();
        int ri = road.robot_index;
        if (robots_match[ri] == 0 && goods_match.find(road.goods_index) == goods_match.end())
        { // 如果该机器人没被匹配并且该商品没被匹配，则匹配该机器人和商品
            Robots[ri].goods_index = road.goods_index;
            Robots[ri].berth_index = road.berth_index;
            Robots[ri].dir = road.next_dir;
            Robots[ri].goods_distance = road.goods_distance;
            // 如果该机器人在自己想要拿的物品上，就拿起并且朝港口走
            int goods_id = IsOnGoods(Robots[ri].x, Robots[ri].y);
            if (goods_id == Robots[ri].goods_index)
            {
                Robots[ri].action = 0;
                Robots[ri].is_goods = 1;
                Robots[ri].dir = PsbDirToBerth(Robots[ri].berth_index, Robots[ri].x, Robots[ri].y);
                World[Robots[ri].x][Robots[ri].y] = 0;
            }
            robot_match_num++;
            robots_match[ri] = 1;
            goods_match.insert(road.goods_index);
        }
        roads_pq.pop();
    }

    for (int ri = 0; ri < RobotNum; ri++)
    { // 没匹配上的机器人向远离港口(上一次放货的港口)的方向移动
        if (robots_match[ri] == 0 && Robots[ri].berth_index >= 0)
        { // 没匹配上（没有找到目标商品）
            for (int i = 0; i < 4; i++)
            {
                int new_dir = (ri + i) % 4;
                int nx_ri = Robots[ri].x + DX[new_dir];
                int ny_ri = Robots[ri].y + DY[new_dir];
                if (BerthPathLength[Robots[ri].berth_index][nx_ri][ny_ri] > BerthPathLength[Robots[ri].berth_index][Robots[ri].x][Robots[ri].y])
                { // 这个位置去港口的路程更大，就相当于远离港口
                    Robots[ri].dir = new_dir;
                    break;
                }
            }
        }
        else if (robots_match[ri] == 0 && Robots[ri].berth_index < 0)
        { // TODO一开始就没找到货物的机器人（原地不动？）
        }
    }
}

// 让位函数
bool CrashAvoid(int ri)
{
    // ri优先让两边
    int new_dir = Robots[ri].dir; // 新方向

    if (new_dir == 0 || new_dir == 1)
    {
        // 当前方向是上或下，所以选择左或右
        int nx1 = Robots[ri].x + DX[2]; // 向左的坐标
        int ny1 = Robots[ri].y + DY[2];
        int nx2 = Robots[ri].x + DX[3]; // 向右的坐标
        int ny2 = Robots[ri].y + DY[3];
        int nx3 = Robots[ri].x + DX[(new_dir + 1) % 2]; // 向后的坐标
        int ny3 = Robots[ri].y + DY[(new_dir + 1) % 2];
        if (IsLandValid(nx1, ny1) && IsLandValid(nx2, ny2))
        {                         // 两个方向都可行,随机选一个
            new_dir = 2 + ri % 2; // 生成 2 或 3
        }
        else if (IsLandValid(nx1, ny1))
        {
            new_dir = 2; // 左
        }
        else if (IsLandValid(nx2, ny2))
        {
            new_dir = 3; // 右
        }
        else if (IsLandValid(nx3, ny3))
        {
            new_dir = (new_dir + 1) % 2; // 后退
        }
        else
        {
            return false;
        }
    }
    else
    {
        // 当前方向是左或右，所以选择上或下
        int nx1 = Robots[ri].x + DX[0]; // 向上的坐标
        int ny1 = Robots[ri].y + DY[0];
        int nx2 = Robots[ri].x + DX[1]; // 向下的坐标
        int ny2 = Robots[ri].y + DY[1];
        int nx3 = Robots[ri].x + DX[5 - new_dir]; // 向后的坐标
        int ny3 = Robots[ri].y + DY[5 - new_dir];
        if (IsLandValid(nx1, ny1) && IsLandValid(nx2, ny2))
        {                     // 两个方向都可行,随机选一个
            new_dir = ri % 2; // 生成 0 或 1
        }
        else if (IsLandValid(nx1, ny1))
        {
            new_dir = 0; // 上
        }
        else if (IsLandValid(nx2, ny2))
        {
            new_dir = 1; // 下
        }
        else if (IsLandValid(nx3, ny3))
        { // 后退
            new_dir = 5 - new_dir;
        }
        else
        {
            return false;
        }
    }

    Robots[ri].dir = new_dir; // 改变ri方向
    return true;
}

// 对冲让位函数
void HedgeAvoid(int ri, int rj, bool (&is_collision_robot)[MAX_ROBOT_NUM])
{
    // 性价比低的机器人避让（优先让两边，实在不行往后面退，再不行就让性价比高的让）
    if (Robots[ri].is_goods && Robots[rj].is_goods)
    { // 都拿物品
        if ((is_collision_robot[ri] && is_collision_robot[rj]) || (!is_collision_robot[ri] && !is_collision_robot[rj]))
        { // 若都有/都没有碰撞过
            if (GetGoodsRobotsCompair(ri, rj))
            { // 我拿的好
                if (!CrashAvoid(rj))
                {
                    CrashAvoid(ri);
                    is_collision_robot[ri] = true;
                }
                else
                {
                    is_collision_robot[rj] = true;
                }
            }
            else
            { // 别人拿的好
                if (!CrashAvoid(ri))
                {
                    CrashAvoid(rj);
                    is_collision_robot[rj] = true;
                }
                else
                {
                    is_collision_robot[ri] = true;
                }
            }
        }
        else if (is_collision_robot[ri])
        { // 只有我有冲突过
            if (!CrashAvoid(rj))
            {
                CrashAvoid(ri);
                is_collision_robot[ri] = true;
            }
            else
            {
                is_collision_robot[rj] = true;
            }
        }
        else
        { // 只有他冲突过
            if (!CrashAvoid(ri))
            {
                CrashAvoid(rj);
                is_collision_robot[rj] = true;
            }
            else
            {
                is_collision_robot[ri] = true;
            }
        }
    }
    else if (!Robots[ri].is_goods && !Robots[rj].is_goods)
    { // 都没拿物品
        if ((is_collision_robot[ri] && is_collision_robot[rj]) || (!is_collision_robot[ri] && !is_collision_robot[rj]))
        { // 若都有/都没有碰撞过
            if (NoGoodsRobotsCompair(ri, rj))
            { // 我拿的好
                if (!CrashAvoid(rj))
                {
                    CrashAvoid(ri);
                    is_collision_robot[ri] = true;
                }
                else
                {
                    is_collision_robot[rj] = true;
                }
            }
            else
            { // 别人拿的好
                if (!CrashAvoid(ri))
                {
                    CrashAvoid(rj);
                    is_collision_robot[rj] = true;
                }
                else
                {
                    is_collision_robot[ri] = true;
                }
            }
        }
        else if (is_collision_robot[ri])
        { // 只有我有冲突过
            if (!CrashAvoid(rj))
            {
                CrashAvoid(ri);
                is_collision_robot[ri] = true;
            }
            else
            {
                is_collision_robot[rj] = true;
            }
        }
        else
        { // 只有他冲突过
            if (!CrashAvoid(ri))
            {
                CrashAvoid(rj);
                is_collision_robot[rj] = true;
            }
            else
            {
                is_collision_robot[ri] = true;
            }
        }
    }
    else if (!Robots[ri].is_goods && Robots[rj].is_goods)
    { // 我没拿别人拿了，我让，ri优先让两边
        if ((is_collision_robot[ri] && is_collision_robot[rj]) || (!is_collision_robot[ri] && !is_collision_robot[rj]))
        { // 若都有/都没有碰撞过
            if (!CrashAvoid(ri))
            {
                CrashAvoid(rj);
                is_collision_robot[rj] = true;
            }
            else
            {
                is_collision_robot[ri] = true;
            }
        }
        else if (is_collision_robot[ri])
        { // 只有我有冲突过
            if (!CrashAvoid(rj))
            {
                CrashAvoid(ri);
                is_collision_robot[ri] = true;
            }
            else
            {
                is_collision_robot[rj] = true;
            }
        }
        else
        { // 只有他冲突过
            if (!CrashAvoid(ri))
            {
                CrashAvoid(rj);
                is_collision_robot[rj] = true;
            }
            else
            {
                is_collision_robot[ri] = true;
            }
        }
    }
    else
    { // 别人没拿我拿了，别人让
        if ((is_collision_robot[ri] && is_collision_robot[rj]) || (!is_collision_robot[ri] && !is_collision_robot[rj]))
        { // 若都有/都没有碰撞过
            if (!CrashAvoid(rj))
            {
                CrashAvoid(ri);
                is_collision_robot[ri] = true;
            }
            else
            {
                is_collision_robot[rj] = true;
            }
        }
        else if (is_collision_robot[ri])
        { // 只有我有冲突过
            if (!CrashAvoid(rj))
            {
                CrashAvoid(ri);
                is_collision_robot[ri] = true;
            }
            else
            {
                is_collision_robot[rj] = true;
            }
        }
        else
        { // 只有他冲突过
            if (!CrashAvoid(ri))
            {
                CrashAvoid(rj);
                is_collision_robot[rj] = true;
            }
            else
            {
                is_collision_robot[ri] = true;
            }
        }
    }
}

// 抢位让位函数
void RushPosisionAvoid(int ri, int rj, bool (&is_collision_robot)[MAX_ROBOT_NUM])
{
    if ((is_collision_robot[ri] && is_collision_robot[rj]) || (!is_collision_robot[ri] && !is_collision_robot[rj]))
    { // 若都有/都没有碰撞过
        // 有货物的优先走，另一个停下
        if (Robots[ri].is_goods && !Robots[rj].is_goods)
        {
            Robots[rj].dir = -1;
            is_collision_robot[rj] = true;
        }
        else if (Robots[rj].is_goods && !Robots[ri].is_goods)
        {
            Robots[ri].dir = -1;
            is_collision_robot[ri] = true;
        }
        // 都有货物或者都没有货物则比较性价比
        else if (Robots[ri].is_goods && Robots[rj].is_goods)
        {
            if (GetGoodsRobotsCompair(ri, rj))
            { // 我拿的好，别人停
                Robots[rj].dir = -1;
                is_collision_robot[rj] = true;
            }
            else
            { // 别人拿的好，我停
                Robots[ri].dir = -1;
                is_collision_robot[ri] = true;
            }
        }
        else
        {
            if (NoGoodsRobotsCompair(ri, rj))
            { // 我要拿的好，别人停
                Robots[rj].dir = -1;
                is_collision_robot[rj] = true;
            }
            else
            { // 别人要拿的好，我停
                Robots[ri].dir = -1;
                is_collision_robot[ri] = true;
            }
        }
    }
    else if (is_collision_robot[ri])
    { // 只有我有冲突过
        Robots[rj].dir = -1;
        is_collision_robot[rj] = true;
    }
    else
    { // 只有他有冲突过
        Robots[ri].dir = -1;
        is_collision_robot[ri] = true;
    }
}

// 碰撞检测与规避
void AvoidCollision()
{
    bool is_collision = true;                         // 本次是否有碰撞
    int detect_num = 0;                               // 检测的次数
    bool is_collision_robot[MAX_ROBOT_NUM] = {false}; // 对每个机器人判断本轮是否有冲突

    while (is_collision)
    {
        detect_num++;
        is_collision = false;
        // 对每个机器人进行碰撞检测
        for (int ri = 0; ri < RobotNum; ri++)
        {
            // 如果这个机器人没有被困死并且下一步会移动（对于每个机器人只考虑主动碰撞，如果是被碰，会被其他机器人主动碰）
            if (Robots[ri].dir >= 0)
            {
                // 机器人i下一步的位置
                int nx_ri = Robots[ri].x + DX[Robots[ri].dir];
                int ny_ri = Robots[ri].y + DY[Robots[ri].dir];
                // 检测除自己之外其他机器人会不会在这个位置上（不移动的机器人看当前位置，会移动的机器人分析是对冲还是抢位置）
                for (int rj = 0; rj < RobotNum; rj++)
                {
                    if (ri != rj)
                    {
                        // 碰上不移动的机器人j
                        if (Robots[rj].dir < 0 && Robots[rj].x == nx_ri && Robots[rj].y == ny_ri)
                        {
                            is_collision = true;
                            // if (Robots[rj].status == 0)
                            // { // ？若不动的处于恢复状态，我垂直移动，不行我就罚站
                            //     if (!CrashAvoid(ri))
                            //     {
                            //         Robots[ri].dir = -1;
                            //     };
                            //     is_collision_robot[ri] = true;
                            // }
                            // else
                            // 若不动的只是在罚站，他垂直移动
                            if (is_collision_robot[ri] && is_collision_robot[rj])
                            {                   // 若都有碰撞过
                                CrashAvoid(rj); // 他垂直移动
                                is_collision_robot[rj] = true;
                            }
                            else if (is_collision_robot[ri])
                            {                   // 若我有碰撞过
                                CrashAvoid(rj); // 他垂直移动
                                is_collision_robot[rj] = true;
                            }
                            else if (is_collision_robot[rj])
                            {                   // 若他有碰撞过
                                CrashAvoid(ri); // 我垂直移动
                                is_collision_robot[ri] = true;
                            }
                            else
                            {                   // 若都没有碰撞过
                                CrashAvoid(rj); // 他垂直移动
                                is_collision_robot[rj] = true;
                            }
                            break;
                        }
                        // 碰上移动的机器人j
                        if (Robots[rj].dir >= 0)
                        {
                            int nx_rj = Robots[rj].x + DX[Robots[rj].dir];
                            int ny_rj = Robots[rj].y + DY[Robots[rj].dir];
                            // 对冲
                            if (Robots[rj].x == nx_ri && Robots[rj].y == ny_ri && Robots[ri].x == nx_rj && Robots[ri].y == ny_rj)
                            {
                                is_collision = true;
                                HedgeAvoid(ri, rj, is_collision_robot);
                                break;
                            }
                            // 抢位
                            if (nx_ri == nx_rj && ny_ri == ny_rj)
                            {
                                is_collision = true;
                                // 判断中间有没有机器人
                                int rk = 0;
                                for (rk = 0; rk < RobotNum; rk++)
                                {
                                    if (Robots[rk].x == nx_ri && Robots[rk].y == ny_ri)
                                    {
                                        break;
                                    }
                                }
                                if (rk < RobotNum)
                                {                                                  // 有机器人
                                    int nx_rk = Robots[rk].x + DX[Robots[rk].dir]; // rk下一步位置
                                    int ny_rk = Robots[rk].y + DY[Robots[rk].dir];

                                    if (Robots[rk].x == nx_ri && Robots[rk].y == ny_ri && Robots[ri].x == nx_rk && Robots[ri].y == ny_rk)
                                    { // k和i对冲
                                        HedgeAvoid(ri, rk, is_collision_robot);
                                    }
                                    else if (Robots[rk].x == nx_rj && Robots[rk].y == ny_rj && Robots[rj].x == nx_rk && Robots[rj].y == ny_rk)
                                    { // k和j对冲
                                        HedgeAvoid(rj, rk, is_collision_robot);
                                    }
                                    else
                                    { // k都不对冲，判断i,j抢位
                                        RushPosisionAvoid(ri, rj, is_collision_robot);
                                    }
                                }
                                else
                                { // 没机器人，判断i，j抢位
                                    RushPosisionAvoid(ri, rj, is_collision_robot);
                                }
                                break;
                            }
                        }
                    }
                }
                // 如果有碰撞并且规避了，就跳出，需要重新检测碰撞
                if (is_collision)
                {
                    break;
                }
            }
        }
        // 检测100次就不检测了
        if (detect_num >= 100)
        {
            break;
        }
    }
}

// 打印机器人指令
void PrintRobotsIns()
{
    // move:0:右，1:左，2:上，3:下
    for (int i = 0; i < RobotNum; i++)
    {
        if (Robots[i].action >= 0)
        {
            if (Robots[i].action == 0)
            { // get
                printf("get %d\n", i);
            }
            else if (Robots[i].action == 1)
            { // pull
                printf("pull %d\n", i);
            }
        }
        if (Robots[i].dir == 0)
        { // 上
            printf("move %d 2\n", i);
        }
        else if (Robots[i].dir == 1)
        { // 下
            printf("move %d 3\n", i);
        }
        else if (Robots[i].dir == 2)
        { // 左
            printf("move %d 1\n", i);
        }
        else if (Robots[i].dir == 3)
        { // 右
            printf("move %d 0\n", i);
        }
    }
}

// 在交货点找一个最好的港口出发
int FindBestBerthFromDelivery(int boat_index)
{
}

// 在港口选择其他港口或者返回交货点
int FindBestBerthOrGoFromBerth(int boat_index)
{
}

// 船的调度
void BoatDispatch()
{
}

// 装载货物
void LoadGoods()
{
}

// 购买机器人
void BuyRobots()
{
}

// 购买船舶
void BuyBoats()
{
}

// 输出实际金钱与我们自己算的金钱
void PrintMoney(ofstream &out_file)
{
    out_file << "Real money: " << Money << endl;
    out_file << "Our  money: " << OurMoney << endl;
}

// 输出每个港口的货物量，货物总价值以及是否聚焦
void PrintBerthGoodsInfo(ofstream &out_file)
{
    int berth_goods_value[BerthNum] = {0};
    int berth_goods_num[BerthNum] = {0};
    for (int i = 0; i < BerthNum; i++)
    {
        queue<int> tmp = Berths[i].goods_queue;
        for (int j = 0; j < Berths[i].goods_queue.size(); j++)
        {
            int goods_index = tmp.front();
            tmp.pop();
            berth_goods_value[i] += AllGoods[goods_index].val;
            berth_goods_num[i]++;
        }
        out_file << "Berth " << i << " has " << left << setw(3) << berth_goods_num[i] << " goods, value is " << setw(5) << berth_goods_value[i] << ", focus is " << Berths[i].focus << endl;
    }
}

// 输出每艘船的信息
void PrintBoatInfo(ofstream &out_file)
{
    out_file << "Boat Capacity: " << BoatCapacity << endl;
    for (int i = 0; i < BoatNum; i++)
    {
        out_file << "Boat " << i << " status " << Boats[i].status << ", is at (" << setw(3) << Boats[i].x << "," << setw(3) << Boats[i].y << Boats[i].y << "dir " << Boats[i].dir << ", dest "
                 << setw(2) << Boats[i].dest << ", goods num " << setw(3) << Boats[i].goods_num << endl;
    }
}

// 输出机器人所获得的总金额
void PrintRobotsMoney(ofstream &out_file)
{
    out_file << "----------------------------Robot Money-------------------------------" << endl;
    out_file << "Robot Money: " << RobotMoney << endl;
}

// 输出所有的虚拟点到港口时间
void PrintVirtualToBerthTime(ofstream &out_file)
{
    out_file << "------------------------Virtual To Berth Time-------------------------" << endl;
    for (int i = 0; i < BerthNum; i++)
    {
        out_file << "Virtual " << i << " to Berth Time: " << left << setw(6) << DeliveryToBerthTime[i] << endl;
    }
}

// 输出信息
void Print(ofstream &out_file, int interval)
{
    if (Frame == 0)
    {
        PrintVirtualToBerthTime(out_file);
    }

    if (Frame % interval != 0)
    {
        return;
    }

    if (out_file.is_open())
    {
        out_file << "----------------------------Frame: " << left << setw(5) << Frame << "------------------------------" << endl;
        PrintMoney(out_file);
        PrintBerthGoodsInfo(out_file);
        PrintBoatInfo(out_file);
    }
    if (Frame == MAX_FRAME - 1)
    {
        PrintRobotsMoney(out_file);
    }
}

string GetTimeString()
{
    // 获得现在的时间
    time_t currentTime = time(nullptr);
    struct tm *localTime = localtime(&currentTime);
    // 将时间转换为字符串形式
    char time_string[100];                                                      // 用于存储时间的字符数组
    strftime(time_string, sizeof(time_string), "%Y-%m-%d-%H-%M-%S", localTime); // 格式化时间字符串time

    return {time_string};
}

ofstream CreateFile()
{
    string time_string = GetTimeString();
    ofstream out_file(string("./output/output") + time_string + ".txt", ios::app); // 打开文件 output.txt，如果不存在则创建

    return out_file;
}

int main()
{
    Init();

#ifdef DEBUG
    ofstream out_file = CreateFile();
#endif

    for (int frame = 1; frame <= MAX_FRAME; frame++)
    {

#ifdef DEBUG
        Print(out_file, 1);
#endif

        Input();
        RobotDsipatchGreedy();
        AvoidCollision();
        PrintRobotsIns();
        BoatDispatch();
        LoadGoods();
        BuyRobots();
        BuyBoats();
        puts("OK");
        fflush(stdout);
    }

#ifdef DEBUG
    out_file.close();
#endif

    return 0;
}