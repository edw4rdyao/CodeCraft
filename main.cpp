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
#include <cassert>
#endif

#define DEBUG

using namespace std;

const int MAX_FRAME = 15000;

const unsigned MAX_ROBOT_NUM = 100;
unsigned int RobotNum = 0;

const unsigned int MAX_BERTH_NUM = 10;
unsigned int BerthNum = 0;

const int MAX_BOAT_NUM = 100;
unsigned int BoatNum = 0;

const int MAX_DELIVERY_NUM = 10;
unsigned int DeliveryNum = 0;

const int MAX_ROBOT_BUYING_NUM = 10;
unsigned int RobotBuyingNum = 0;

const int MAX_BOAT_BUYING_NUM = 10;
unsigned int BoatBuyingNum = 0;

const int N = 200;
const int MAX_GOODS_NUM = 150010;
const int MAX_LENGTH = 40000;

// 可以修改的参数 每个机器人BFS找的最多路径与长度 离货物距离的比重
const int MAX_ROAD_NUM = 10;
const int MAX_ROAD_LEN = 350;
const double TO_GOODS_WEIGHT = 3.0;
const double H_VALUE_WEIGHT = 2.0;

// 最大购买机器人和船的数量，购买机器人和船的价格
const int ROBOT_BOAT_PROPORTION = 2;
int MAX_BUY_ROBOT_NUM = 20;
int MAX_BUY_BOAT_NUM = 10;
const int ROBOT_BUY_MONEY = 2000;
const int BOAT_BUY_MONEY = 8000;

const int DX[4] = {-1, 1, 0, 0};     // 每个方向x轴的偏移
const int DY[4] = {0, 0, -1, 1};     // 每个方向y轴的偏移
const int REV_DIR[4] = {1, 0, 3, 2}; // 上下左右的反方向（用于BFS记录路径）：下上右左

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
const int DX_BOAT[4][6] = {{0, 0, 0, 1, 1, 1},     // 右
                           {0, 0, 0, -1, -1, -1},  // 左
                           {0, -1, -2, 0, -1, -2}, // 上
                           {0, 1, 2, 0, 1, 2}};    // 下

const int DY_BOAT[4][6] = {{0, 1, 2, 0, 1, 2},     // 右
                           {0, -1, -2, 0, -1, -2}, // 左
                           {0, 0, 0, 1, 1, 1},     // 上
                           {0, 0, 0, -1, -1, -1}}; // 下

// 船：0右 1左 2上 3下，顺逆时针后各个方向对应的方向
const int ROT_DIR[2][4] = {{3, 2, 0, 1},
                           {2, 3, 1, 0}};
// 船：0顺时针 1逆时针 转之后核心点移动到的位置 2号位置 ：核心点变到4号位置
const int ROT_POS[2] = {2, 4};

int OurMoney = 25000;           // 自己算出来的金钱
int RobotMoney = 0;             // 机器人拿的的钱
int Money, BoatCapacity, Frame; // 金钱，船的容量，当前帧数
unsigned int World[N][N];       // 地图
int WorldGoods[N][N];           // 地图上每个位置的商品

vector<int> BerthPath[MAX_BERTH_NUM][N][N]; // 机器人泊位到每个点的最短路径第一步的可能方向(0: 上，1: 下，2: 左，3: 右)
int BerthPathLength[MAX_BERTH_NUM][N][N];   // 机器人泊位到每个点的最短路径长度

int DeliveryToBerthTime[MAX_DELIVERY_NUM][MAX_BERTH_NUM];  // 交货点到港口的最短时间
int BuyingToBerthTime[MAX_BOAT_BUYING_NUM][MAX_BERTH_NUM]; // 船舶购买点到港口的最短时间
int BerthNearestBuying[MAX_BERTH_NUM];                     // 港口最近的购买点
int BerthNearestDelivery[MAX_BERTH_NUM];                   // 港口最近的交货点

// 估算的到交货点或者港口的最短时间（用于启发函数）
int ToBerthEstimateTime[MAX_BERTH_NUM][N][N];
int ToDeliveryEstimateTime[MAX_DELIVERY_NUM][N][N];
// int ToBerthEstimateTimeForBoat[MAX_BERTH_NUM][N][N][4];
// int ToDeliveryEstimateTimeForBoat[MAX_DELIVERY_NUM][N][N][4];

// 记录初始购买机器人数目和船的数目，用于调参
int InitBuyRobotNum = 6;
int InitBuyBoatNum = (25000 - 2000 * InitBuyRobotNum) / 8000;

// 初始要去的港口、购买船的位置
int InitBerthToGo[MAX_BOAT_NUM];
int InitBuyingToBuy[MAX_BOAT_NUM];

// 预估每个机器人、船每帧拿多少价值，用于调参
double RobotEveryFrame = 2;
double BoatEveryFrame = 3;
// 港口上屯小于多少货再买机器人 或 有多少钱再买机器人 或 机器人数不能太多，用于调参
int RobotBuyMoney = 10000;
int RobotBuyGoods = 50;
int RobotBuyNum = 15;

// 港口上屯大于多少货再买船 或 有多少钱再买船 或 船数不能太多，用于调参
int BoatBuyMoney = 8000;
int BoatBuyGoods = 50;

//买东西策略选择
int BuyChoose = 2; //xmc:1; cxh:2;

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
vector<BoatRouteState> BoatRoutes[MAX_BOAT_NUM];
// 保存每艘船找不到航道的次数
int BoatCanNotFindRoute[MAX_BOAT_NUM];

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
} Berths[MAX_BERTH_NUM];

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
        return World[x][y] >> 8;
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

// 判断船的当前状态每个位置是否合法以及是否在主航道上(0表示不合法，1表示合法，2表示合法并且有部分在主航道上)
int JudgeBoatState(int x, int y, int dir)
{
    bool is_on_main = false;
    for (int i = 0; i < 6; i++)
    { // 对0-5号位置检查
        int nx = x + DX_BOAT[dir][i];
        int ny = y + DY_BOAT[dir][i];
        if (IsOceanValid(nx, ny))
        {
            if (IsOnMainChannel(nx, ny))
            {
                is_on_main = true;
            }
        }
        else
        {
            return 0;
        }
    }
    return is_on_main ? 2 : 1;
}

// 每个方向左上角/右下角对应几号位置 0右 1左 2上 3下
const int BOAT_LEFT_TOP[4] = {0, 5, 2, 3};
const int BOAT_RIGHT_DOWN[4] = {5, 0, 3, 2};

// 判断两艘船状态是否会碰 会返回true
bool JudgeBoatCollision(int xa, int ya, int dira, int xb, int yb, int dirb)
{
    // 两艘船的左上角坐标
    int lt_xa = xa + DX_BOAT[dira][BOAT_LEFT_TOP[dira]];
    int lt_ya = ya + DY_BOAT[dira][BOAT_LEFT_TOP[dira]];
    int lt_xb = xb + DX_BOAT[dirb][BOAT_LEFT_TOP[dirb]];
    int lt_yb = yb + DY_BOAT[dirb][BOAT_LEFT_TOP[dirb]];
    // 两艘船的右下角坐标
    int rd_xa = xa + DX_BOAT[dira][BOAT_RIGHT_DOWN[dira]];
    int rd_ya = ya + DY_BOAT[dira][BOAT_RIGHT_DOWN[dira]];
    int rd_xb = xb + DX_BOAT[dirb][BOAT_RIGHT_DOWN[dirb]];
    int rd_yb = yb + DY_BOAT[dirb][BOAT_RIGHT_DOWN[dirb]];
    // 得到两艘船重叠位置的边界坐标
    int start_x = max(lt_xa, lt_xb);
    int start_y = max(lt_ya, lt_yb);
    int end_x = min(rd_xa, rd_xb);
    int end_y = min(rd_ya, rd_yb);
    // 遍历重叠部分
    if (start_x <= end_x && start_y <= end_y)
    {
        for (int x = start_x; x <= end_x; x++)
        {
            for (int y = start_y; y <= end_y; y++)
            { // 如果该位置不在主航道上
                if (!IsOnMainChannel(x, y))
                    return true;
            }
        }
    }
    return false;
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

    double cost_value = (double)goods_value / (step_num * TO_GOODS_WEIGHT + to_berth_len);
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
    MAX_BUY_BOAT_NUM = BerthNum;
    MAX_BUY_ROBOT_NUM = ROBOT_BOAT_PROPORTION * MAX_BUY_BOAT_NUM;
    for (int i = 0; i < BerthNum; i++)
    {
        int id;
        scanf("%d", &id);
        int x, y, velocity;
        scanf("%d%d%d", &x, &y, &velocity);
        Berths[id] = Berth(x, y, velocity);
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

struct BoatState
{
    int x;
    int y;
    int dir;
    BoatState(int x, int y, int dir) : x(x), y(y), dir(dir) {}
    BoatState() : x(0), y(0), dir(0) {}
};

// struct BoatStateCompare
// {
//     bool operator()(const pair<int, BoatState> &a, const pair<int, BoatState> &b) const
//     {
//         return a.first > b.first;
//     }
// };

// void InitToDeliveryEstimateTimeForBoat()
// {
//     // 初始化所有交货点到每个点的最短时间，均为-1
//     memset(ToDeliveryEstimateTimeForBoat, -1, sizeof(ToDeliveryEstimateTimeForBoat));
//     for (int d = 0; d < DeliveryNum; d++)
//     { // 对每个交货点，核心点在此的船
//         int sx = Deliveries[d].x;
//         int sy = Deliveries[d].y;
//         // 找一个方向合法的状态，如果没有合法的状态则返回-1
//         int sdir = -1;
//         for (int i = 0; i < 4; i++)
//         {
//             if (JudgeBoatState(sx, sy, i))
//             {
//                 sdir = i;
//                 break;
//             }
//         }
//         if (sdir == -1)
//         {
//             continue; // 这个交货点不合法
//         }
//         // 将本位置加入优先队列
//         priority_queue<pair<int, BoatState>, vector<pair<int, BoatState>>, BoatStateCompare> pq;
//         ToDeliveryEstimateTimeForBoat[d][sx][sy][sdir] = 0;
//         pq.push({0, BoatState(sx, sy, sdir)});
//         while (!pq.empty())
//         {

//             auto cur_state = pq.top();
//             pq.pop();
//             int cur_time = cur_state.first;
//             int x = cur_state.second.x;
//             int y = cur_state.second.y;
//             int dir = cur_state.second.dir;
//             if (ToDeliveryEstimateTimeForBoat[d][x][y][dir] != -1 && cur_time > ToDeliveryEstimateTimeForBoat[d][x][y][dir])
//                 continue;

//             for (int move = 0; move < 3; move++)
//             { // 下一步动作(3种动作 0顺时针转 1逆时针转 2正方向前进)
//                 int nx, ny, ndir;
//                 if (move == 0 || move == 1)
//                 { // 顺时针转或者逆时针转，方向变化ROT_DIR ，核心点也变化ROT_POS
//                     nx = x + DX_BOAT[dir][ROT_POS[move]];
//                     ny = y + DY_BOAT[dir][ROT_POS[move]];
//                     ndir = ROT_DIR[move][dir];
//                 }
//                 else
//                 { // 正方向前进一步，方向不变，核心点变化为1号位置
//                     nx = x + DX_BOAT[dir][1];
//                     ny = y + DY_BOAT[dir][1];
//                     ndir = dir;
//                 }
//                 int boat_state = JudgeBoatState(nx, ny, ndir);
//                 if (boat_state == 0)
//                 { // 该位置状态不合法
//                     continue;
//                 }
//                 else
//                 {
//                     int next_time = cur_time + (boat_state == 1 ? 1 : 2);
//                     if (ToDeliveryEstimateTimeForBoat[d][nx][ny][ndir] < 0 || next_time < ToDeliveryEstimateTimeForBoat[d][nx][ny][ndir])
//                     {
//                         ToDeliveryEstimateTimeForBoat[d][nx][ny][ndir] = next_time;
//                         pq.push({next_time, BoatState(nx, ny, ndir)});
//                     }
//                 }
//             }
//         }
//     }
// }

// void InitToBerthEstimateTimeForBoat()
// {
//     // 初始化所有交货点到每个点的最短时间，均为-1
//     memset(ToBerthEstimateTimeForBoat, -1, sizeof(ToBerthEstimateTimeForBoat));
//     for (int b = 0; b < BerthNum; b++)
//     { // 对每个交货点，核心点在此的船
//         int sx = Berths[b].x;
//         int sy = Berths[b].y;
//         // 找一个方向合法的状态，如果没有合法的状态则返回-1
//         int sdir = -1;
//         for (int i = 0; i < 4; i++)
//         {
//             if (JudgeBoatState(sx, sy, i))
//             {
//                 sdir = i;
//                 break;
//             }
//         }
//         if (sdir == -1)
//         {
//             continue; // 这个港口不合法
//         }
//         // 将本位置加入优先队列
//         priority_queue<pair<int, BoatState>, vector<pair<int, BoatState>>, BoatStateCompare> pq;
//         ToBerthEstimateTimeForBoat[b][sx][sy][sdir] = 0;
//         pq.push({0, BoatState(sx, sy, sdir)});
//         while (!pq.empty())
//         {
//             auto cur_state = pq.top();
//             pq.pop();
//             int cur_time = cur_state.first;
//             int x = cur_state.second.x;
//             int y = cur_state.second.y;
//             int dir = cur_state.second.dir;
//             if (ToBerthEstimateTimeForBoat[b][x][y][dir] != -1 && cur_time > ToBerthEstimateTimeForBoat[b][x][y][dir])
//                 continue;

//             for (int move = 0; move < 3; move++)
//             { // 下一步动作(3种动作 0顺时针转 1逆时针转 2正方向前进)
//                 int nx, ny, ndir;
//                 if (move == 0 || move == 1)
//                 { // 顺时针转或者逆时针转，方向变化ROT_DIR ，核心点也变化ROT_POS
//                     nx = x + DX_BOAT[dir][ROT_POS[move]];
//                     ny = y + DY_BOAT[dir][ROT_POS[move]];
//                     ndir = ROT_DIR[move][dir];
//                 }
//                 else
//                 { // 正方向前进一步，方向不变，核心点变化为1号位置
//                     nx = x + DX_BOAT[dir][1];
//                     ny = y + DY_BOAT[dir][1];
//                     ndir = dir;
//                 }
//                 int boat_state = JudgeBoatState(nx, ny, ndir);
//                 if (boat_state == 0)
//                 { // 该位置状态不合法
//                     continue;
//                 }
//                 else
//                 {
//                     int next_time = cur_time + (boat_state == 1 ? 1 : 2);
//                     if (ToBerthEstimateTimeForBoat[b][nx][ny][dir] < 0 || next_time < ToBerthEstimateTimeForBoat[b][nx][ny][dir])
//                     {
//                         ToBerthEstimateTimeForBoat[b][nx][ny][dir] = next_time;
//                         pq.push({next_time, BoatState(nx, ny, ndir)});
//                     }
//                 }
//             }
//         }
//     }
// }

void InitToDeliveryEstimateTimeDijkstra()
{
    // 初始化所有交货点到每个点的最短路径长度，均为-1
    memset(ToDeliveryEstimateTime, -1, sizeof(ToDeliveryEstimateTime));
    for (int d = 0; d < DeliveryNum; d++)
    {
        int delivery_x = Deliveries[d].x;
        int delivery_y = Deliveries[d].y;
        priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<>> pq;
        // 将本位置加入
        ToDeliveryEstimateTime[d][delivery_x][delivery_y] = 0;
        pq.push({0, {delivery_x, delivery_y}});
        while (!pq.empty())
        {
            auto cur_pos = pq.top();
            pq.pop();

            int cur_dist = cur_pos.first;
            int x = cur_pos.second.first;
            int y = cur_pos.second.second;
            if (ToDeliveryEstimateTime[d][x][y] != -1 && cur_dist > ToDeliveryEstimateTime[d][x][y])
                continue;

            for (int i = 0; i < 4; i++)
            {
                int dir = i;
                int nx = x + DX[dir];
                int ny = y + DY[dir];
                if (IsOceanValid(nx, ny))
                {
                    int weight = IsOnMainChannel(nx, ny) ? 2 : 1; // 主航道上
                    int next_dist = cur_dist + weight;
                    if (ToDeliveryEstimateTime[d][nx][ny] < 0 || next_dist < ToDeliveryEstimateTime[d][nx][ny])
                    {
                        ToDeliveryEstimateTime[d][nx][ny] = next_dist;
                        pq.push({next_dist, {nx, ny}});
                    }
                }
            }
        }
    }
}

void InitToBerthEstimateTimeDijkstra()
{
    // 初始化所有交货点到每个点的最短路径长度，均为-1
    memset(ToBerthEstimateTime, -1, sizeof(ToBerthEstimateTime));
    for (int b = 0; b < BerthNum; b++)
    {
        int berth_x = Berths[b].x;
        int berth_y = Berths[b].y;
        priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<>> pq;
        // 将本位置加入
        ToBerthEstimateTime[b][berth_x][berth_y] = 0;
        pq.push({0, {berth_x, berth_y}});
        while (!pq.empty())
        {
            auto cur_pos = pq.top();
            pq.pop();

            int cur_dist = cur_pos.first;
            int x = cur_pos.second.first;
            int y = cur_pos.second.second;
            if (ToBerthEstimateTime[b][x][y] != -1 && cur_dist > ToBerthEstimateTime[b][x][y])
                continue;

            for (int i = 0; i < 4; i++)
            {
                int dir = i;
                int nx = x + DX[dir];
                int ny = y + DY[dir];
                if (IsOceanValid(nx, ny))
                {
                    int weight = IsOnMainChannel(nx, ny) ? 2 : 1; // 主航道上
                    int next_dist = cur_dist + weight;
                    if (ToBerthEstimateTime[b][nx][ny] < 0 || next_dist < ToBerthEstimateTime[b][nx][ny])
                    {
                        ToBerthEstimateTime[b][nx][ny] = next_dist;
                        pq.push({next_dist, {nx, ny}});
                    }
                }
            }
        }
    }
}

// 计算一个位置状态到目的地的H值
double GetHValue(int x, int y, int dir, int action, int d_type, int d_id)
{
    int dx = d_type ? Berths[d_id].x : Deliveries[d_id].x;
    int dy = d_type ? Berths[d_id].y : Deliveries[d_id].y;
    double h_value = 0;
    // 距离的估算价值
    for (int i = 0; i < 6; i++)
    { // 对于船的6个位置
        if (d_type)
        { // 目的地是港口
            h_value += ToBerthEstimateTime[d_id][x + DX_BOAT[dir][i]][y + DY_BOAT[dir][i]];
            // h_value += ToBerthEstimateTime[d_id][x + DX_BOAT[dir][0]][y + DY_BOAT[dir][0]];
        }
        else
        { // 目的地是交货点
            h_value += ToDeliveryEstimateTime[d_id][x + DX_BOAT[dir][i]][y + DY_BOAT[dir][i]];
            // h_value += ToDeliveryEstimateTime[d_id][x + DX_BOAT[dir][0]][y + DY_BOAT[dir][0]];
        }
    }
    h_value /= 6;

    // 尽量不转弯（这样会错过最优路径）
    // if (action == 0 || action == 1)
    // {
    //     h_value += 1;
    // }

    return h_value;
}

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

// 二叉小根堆上滤
void PercolateUp(vector<shared_ptr<BoatStateNode>> &heap, shared_ptr<BoatStateNode> state_node)
{
    // 先找到这个更新节点的下标
    size_t idx = 0;
    for (; idx < heap.size(); idx++)
    {
        if (heap[idx] == state_node)
        {
            break;
        }
    }
    if (idx == heap.size())
    {
        assert(false);
    }
    // 执行二叉小根堆上滤
    while (idx > 0)
    {
        size_t parent = (idx - 1) / 2;
        if (heap[idx]->f_value() < heap[parent]->f_value())
        {
            swap(heap[idx], heap[parent]);
            idx = parent;
        }
        else
        {
            return;
        }
    }
}

vector<int> AStarSearchNodeNum; // 每次Astar搜索的节点数

// 返回一个位置到另一个位置最短时间，-1表示不可达
int PositionToPositionAStar(int sx, int sy, int d_type, int d_id)
{
    int search_node_num = 0;
    int dx = d_type ? Berths[d_id].x : Deliveries[d_id].x;
    int dy = d_type ? Berths[d_id].y : Deliveries[d_id].y;
    // 首先初始化船起点的状态（核心点在起点，找一个方向合法的状态），如果没有合法的状态则返回-1
    int sdir = -1;
    for (int i = 0; i < 4; i++)
    {
        if (JudgeBoatState(sx, sy, i))
        {
            sdir = i;
            break;
        }
    }
    if (sdir == -1)
    {
        AStarSearchNodeNum.push_back(search_node_num);
        return -1;
    }
    shared_ptr<BoatStateNode> start_state_node = make_shared<BoatStateNode>(
        sx, sy, sdir, -1, false, 0, GetHValue(sx, sy, sdir, -1, d_type, d_id), 1, nullptr);

    // 开始A*找最短时间
    vector<shared_ptr<BoatStateNode>> openlist_heap;             // 开放列表，小根堆
    vector<vector<vector<shared_ptr<BoatStateNode>>>> all_nodes( // 哈希表，用来存所有的节点
        N, vector<vector<shared_ptr<BoatStateNode>>>(
               N, vector<shared_ptr<BoatStateNode>>(
                      4, nullptr)));

    all_nodes[sx][sy][start_state_node->dir] = start_state_node; // 加入哈希表
    openlist_heap.push_back(start_state_node);                   // 一个元素直接加入开放列表，不需要调整堆
    start_state_node->list_state = 1;                            // 更新节点状态

    while (!openlist_heap.empty())
    {
        // 从开放列表中取出f值最小的状态节点
        shared_ptr<BoatStateNode> cur = openlist_heap.front();
        pop_heap(openlist_heap.begin(), openlist_heap.end(), CompareBoatStateNode());
        openlist_heap.pop_back();
        search_node_num++;
        // 将该节点加入到关闭列表（实际上是修改其状态）
        all_nodes[cur->x][cur->y][cur->dir]->list_state = 2;
        // 判断是不是终点
        bool is_arrive = false;
        int to_berth_time = 0;
        if (d_type == 1 && IsInBerthRange(cur->x, cur->y) == d_id)
        { // 到了港口的靠泊区
            is_arrive = true;
            // 大约为目前位置到港口位置曼哈顿距离的两倍
            to_berth_time = 2 * (abs(cur->x - dx) + std::abs(cur->y - dy));
        }
        else if (d_type == 0 && cur->x == dx && cur->y == dy)
        { // 到了交货点
            is_arrive = true;
        }
        if (is_arrive)
        { // 到达目的地
            AStarSearchNodeNum.push_back(search_node_num);
            return cur->g_value + to_berth_time;
        }
        // 下一步动作之后的船状态(3种动作 0顺时针转 1逆时针转 2正方向前进)
        for (int move = 0; move < 3; move++)
        {
            // 获取下一步的位置状态是否是合法的
            int nx, ny, ndir;
            if (move == 0 || move == 1)
            { // 顺时针转或者逆时针转，方向变化ROT_DIR ，核心点也变化ROT_POS
                nx = cur->x + DX_BOAT[cur->dir][ROT_POS[move]];
                ny = cur->y + DY_BOAT[cur->dir][ROT_POS[move]];
                ndir = ROT_DIR[move][cur->dir];
            }
            else
            { // 正方向前进一步，方向不变，核心点变化为1号位置
                nx = cur->x + DX_BOAT[cur->dir][1];
                ny = cur->y + DY_BOAT[cur->dir][1];
                ndir = cur->dir;
            }
            // 看该位置是否合法，以及是否有某部分是主航道
            int boat_state = JudgeBoatState(nx, ny, ndir);
            if (boat_state == 0)
            { // 该位置状态不合法
                continue;
            }
            else
            { // 该位置合法，开始判断该状态节点
                // 如果下个位置有部分在主航道上，则2帧移动一步
                int new_g_value = cur->g_value + (boat_state == 1 ? 1 : 2);
                if (all_nodes[nx][ny][ndir] == nullptr)
                { // 之前没找过，则计算F值，G值，加入到开放列表并且调整小根堆(更新到节点哈希表)
                    shared_ptr<BoatStateNode> new_state_node = make_shared<BoatStateNode>(
                        nx, ny, ndir, move, bool(boat_state == 2), new_g_value, GetHValue(nx, ny, ndir, move, d_type, d_id), 1, cur);
                    all_nodes[nx][ny][ndir] = new_state_node;
                    openlist_heap.push_back(new_state_node);
                    push_heap(openlist_heap.begin(), openlist_heap.end(), CompareBoatStateNode());
                }
                else if (all_nodes[nx][ny][ndir]->list_state == 1)
                { // 节点在Open列表中，如果G值更小，则更新G值，并且重新调整小根堆
                    if (new_g_value < all_nodes[nx][ny][ndir]->g_value)
                    {
                        all_nodes[nx][ny][ndir]->g_value = new_g_value;
                        all_nodes[nx][ny][ndir]->pre_state = cur;
                        // 调整小根堆
                        PercolateUp(openlist_heap, all_nodes[nx][ny][ndir]);
                    }
                }
                else if (all_nodes[nx][ny][ndir]->list_state == 2)
                { // 之前找过并且在关闭列表中，则什么都不干
                }
            }
        }
    }
    AStarSearchNodeNum.push_back(search_node_num);
    return -1;
}

// 记录交货点到泊位之间的最短时间
void InitDeliveryToBerth()
{
    // 初始化交货点到泊位之间的最短时间 -1为不可达
    memset(DeliveryToBerthTime, -1, sizeof(DeliveryToBerthTime));
    // 初始化每个泊位的最近交货点
    memset(BerthNearestDelivery, -1, sizeof(BerthNearestDelivery));
    for (int di = 0; di < DeliveryNum; di++)
    { // 对每一个交货点
        for (int bi = 0; bi < BerthNum; bi++)
        { // 对每一个泊位,A星计算时间
            if (ToBerthEstimateTime[bi][Deliveries[di].x][Deliveries[di].y] == -1)
            { // 如果不可达
                DeliveryToBerthTime[di][bi] = -1;
                BerthNearestDelivery[bi] = -1;
            }
            else
            {
                DeliveryToBerthTime[di][bi] = PositionToPositionAStar(Deliveries[di].x, Deliveries[di].y, 1, bi);
                // 如果之前没有找到或者找到的时间更短
                if (BerthNearestDelivery[bi] == -1 || DeliveryToBerthTime[di][bi] < DeliveryToBerthTime[BerthNearestDelivery[bi]][bi])
                {
                    BerthNearestDelivery[bi] = di;
                }
            }
        }
    }
}

// 记录船舶购买点到泊位的最短时间
void InitBuyingToBerth()
{
    // 初始化交货点到泊位之间的最短时间 -1为不可达
    memset(BuyingToBerthTime, -1, sizeof(BuyingToBerthTime));
    // 初始化离每个港口最近的购船点，购船点不可达为-1
    memset(BerthNearestBuying, -1, sizeof(BerthNearestBuying));
    for (int bbi = 0; bbi < BoatBuyingNum; bbi++)
    { // 对每一个轮船购买点
        for (int bi = 0; bi < BerthNum; bi++)
        { // 对每一个泊位，A星计算时间
            if (ToBerthEstimateTime[bi][BoatBuyings[bbi].x][BoatBuyings[bbi].y] == -1)
            { // 如果不可达
                BuyingToBerthTime[bbi][bi] = -1;
                BerthNearestBuying[bi] = -1;
            }
            else
            {
                BuyingToBerthTime[bbi][bi] = PositionToPositionAStar(BoatBuyings[bbi].x, BoatBuyings[bbi].y, 1, bi);
                // 如果之前没有找到或者找到的时间更短
                if (BerthNearestBuying[bi] == -1 || BuyingToBerthTime[bbi][bi] < BuyingToBerthTime[BerthNearestBuying[bi]][bi])
                {
                    BerthNearestBuying[bi] = bbi;
                }
            }
        }
    }
}

// 初始化港口最近的购买点和交货点
void InitBerthNearest()
{
    memset(InitBuyingToBuy, -1, sizeof(InitBuyingToBuy));
    memset(InitBerthToGo, -1, sizeof(InitBerthToGo));
}

// 初始化函数
void Init()
{
    InitWorldInfo();
    InitToBerthBFS();
    InitToBerthEstimateTimeDijkstra();
    InitToDeliveryEstimateTimeDijkstra();
    InitDeliveryToBerth();
    InitBuyingToBerth();
    InitBerthNearest();
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
    RobotNum = robot_num;
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
    scanf("%d", &boat_num);
    BoatNum = boat_num;
    for (int i = 0; i < boat_num; i++)
    {
        int id, goods_num, x, y, dir, status;
        scanf("%d%d%d%d%d%d\n", &id, &goods_num, &x, &y, &dir, &status);

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
    double ri_val = (double)AllGoods[goodsi].val / (Robots[ri].goods_distance * TO_GOODS_WEIGHT + BerthPathLength[Robots[ri].berth_index][AllGoods[goodsi].x][AllGoods[goodsi].y]); // 机器人i要拿物品的性价比
    double rj_val = (double)AllGoods[goodsj].val / (Robots[rj].goods_distance * TO_GOODS_WEIGHT + BerthPathLength[Robots[rj].berth_index][AllGoods[goodsj].x][AllGoods[goodsj].y]); // 机器人j要拿物品的性价比
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
void RobotDispatchGreedy()
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
void RushPositionAvoid(int ri, int rj, bool (&is_collision_robot)[MAX_ROBOT_NUM])
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
                // 若在主干道上，就不管
                if (IsOnMainRoad(nx_ri, ny_ri))
                {
                    continue;
                }
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
                                        RushPositionAvoid(ri, rj, is_collision_robot);
                                    }
                                }
                                else
                                { // 没机器人，判断i，j抢位
                                    RushPositionAvoid(ri, rj, is_collision_robot);
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
int FindBestBerthFromDelivery(int boat_id, int delivery_id)
{
    int best_berth = -1;
    double max_value = -1;
    for (int bi = 0; bi < BerthNum; bi++)
    { // 对每个没有船要去的港口，并且该港口可达
        if (Berths[bi].boat_id == -1 && DeliveryToBerthTime[delivery_id][boat_id] != -1)
        {
            // 计算性价比（过去装完直接去交货点）
            double goods_value = 0;
            int transport_time = DeliveryToBerthTime[delivery_id][bi] + DeliveryToBerthTime[BerthNearestDelivery[bi]][bi];
            queue<int> tmp = Berths[bi].goods_queue;
            int to_load_goods = min((int)tmp.size(), BoatCapacity);
            for (int j = 0; j < to_load_goods; j++)
            {
                goods_value += AllGoods[tmp.front()].val;
                tmp.pop();
            }
            goods_value /= transport_time;
            if (goods_value > max_value)
            {
                best_berth = bi;
                max_value = goods_value;
            }
        }
    }
    return best_berth;
}

// 在港口选择其他港口或者返回交货点
int FindBestBerthOrGoFromBerth(int boat_index)
{
    return -1;
}

// 在购买点选择最好的港口
int FindBestBerthFromBuying(int boat_id, int boat_buying_id)
{
    int best_berth = -1;
    double max_value = -1;
    for (int bi = 0; bi < BerthNum; bi++)
    { // 对每个没有船要去的港口，并且该购买点可达
        if (Berths[bi].boat_id == -1 && BuyingToBerthTime[boat_buying_id][bi] != -1)
        {
            // 计算性价比（过去装完直接去交货点）
            double goods_value = 0;
            int transport_time = BuyingToBerthTime[boat_buying_id][bi] + DeliveryToBerthTime[BerthNearestDelivery[bi]][bi];
            queue<int> tmp = Berths[bi].goods_queue;
            int to_load_goods = min((int)tmp.size(), BoatCapacity);
            for (int j = 0; j < to_load_goods; j++)
            {
                goods_value += AllGoods[tmp.front()].val;
                tmp.pop();
            }
            goods_value /= transport_time;
            if (goods_value > max_value)
            {
                best_berth = bi;
                max_value = goods_value;
            }
        }
    }
    return best_berth;
}

void BoatToPositionAStar(int bi, int d_type, int d_id)
{
    int dx = d_type ? Berths[d_id].x : Deliveries[d_id].x;
    int dy = d_type ? Berths[d_id].y : Deliveries[d_id].y;
    shared_ptr<BoatStateNode> start_state_node = make_shared<BoatStateNode>(
        Boats[bi].x, Boats[bi].y, Boats[bi].dir, 3, false, 0, GetHValue(Boats[bi].x, Boats[bi].y, Boats[bi].dir, -1, d_type, d_id), 1, nullptr);

    // 开始A*找最短时间
    vector<shared_ptr<BoatStateNode>> openlist_heap;                      // 开放列表，小根堆
    unordered_map<BoatStateNodeKey, shared_ptr<BoatStateNode>> all_nodes; // 哈希表，用来存所有节点

    all_nodes[BoatStateNodeKey(Boats[bi].x * N + Boats[bi].y, Boats[bi].dir, 0)] = start_state_node; // 加入哈希表
    openlist_heap.push_back(start_state_node);                                                       // 一个元素直接加入开放列表，不需要调整堆
    start_state_node->list_state = 1;                                                                // 更新节点状态

    while (!openlist_heap.empty())
    {
        // 从开放列表中取出f值最小的状态节点
        shared_ptr<BoatStateNode> cur = openlist_heap.front();
        pop_heap(openlist_heap.begin(), openlist_heap.end(), CompareBoatStateNode());
        openlist_heap.pop_back();

        // 将该节点加入到关闭列表（实际上是修改其状态）
        all_nodes[BoatStateNodeKey(cur->x * N + cur->y, cur->dir, cur->g_value)]->list_state = 2;
        // 判断是不是终点（交货点的话判断核心点是不是到了，泊位的话判断到了核心点到了靠泊区）
        bool is_arrive = false;
        if (d_type == 1)
        { // 目的地是港口，只要核心点到靠泊区就好
            is_arrive = (IsInBerthRange(cur->x, cur->y) == d_id);
        }
        else
        {
            is_arrive = (cur->x == dx && cur->y == dy);
        }
        if (is_arrive)
        { // 到达目的地，回溯得到路径
            // 首先清空路径
            BoatRoutes[bi].clear();
            shared_ptr<BoatStateNode> p = cur;
            // 回溯得到路径
            while (p != nullptr)
            {
                if (p->is_on_main)
                {
                    BoatRoutes[bi].push_back(BoatRouteState(p->x, p->y, p->dir, 3));
                }
                BoatRoutes[bi].push_back(BoatRouteState(p->x, p->y, p->dir, p->action));
                p = p->pre_state;
            }
            reverse(BoatRoutes[bi].begin(), BoatRoutes[bi].end());
            return;
        }
        // 下一步动作之后的船状态(3种动作 0顺时针转 1逆时针转 2正方向前进 3不动)
        for (int move = 0; move < 4; move++)
        {
            // 获取下一步的位置状态是否是合法的
            int nx, ny, ndir;
            if (move == 0 || move == 1)
            { // 顺时针转或者逆时针转，方向变化ROT_DIR ，核心点也变化ROT_POS
                nx = cur->x + DX_BOAT[cur->dir][ROT_POS[move]];
                ny = cur->y + DY_BOAT[cur->dir][ROT_POS[move]];
                ndir = ROT_DIR[move][cur->dir];
            }
            else if (move == 2)
            { // 正方向前进一步，方向不变，核心点变化为1号位置
                nx = cur->x + DX_BOAT[cur->dir][1];
                ny = cur->y + DY_BOAT[cur->dir][1];
                ndir = cur->dir;
            }
            else
            {
                nx = cur->x;
                ny = cur->y;
                ndir = cur->dir;
            }
            // 看该位置是否合法，以及是否有某部分是主航道
            int boat_state = JudgeBoatState(nx, ny, ndir);
            if (boat_state == 0)
            { // 该位置状态不合法
                continue;
            }
            else
            { // 该位置合法，开始判断该状态节点
                // 如果下个位置有部分在主航道上，则2帧移动一步
                int new_g_value = cur->g_value + ((boat_state == 1 || move == 3) ? 1 : 2);
                // 碰撞检测
                bool will_collision = false;
                for (int bj = 0; bj < BoatNum; bj++)
                {
                    if (bj < bi)
                    { // 对于序号在bi前面的船，已经确定了路线
                        for (int t = cur->g_value + 1; t <= new_g_value + 1; t++)
                        {
                            if (t < BoatRoutes[bj].size() && JudgeBoatCollision(nx, ny, ndir, BoatRoutes[bj][t].x, BoatRoutes[bj][t].y, BoatRoutes[bj][t].dir))
                            {
                                will_collision = true;
                                break;
                            }
                        }
                    }
                    else if (bj > bi)
                    { // 对于序号在bi后面的船，都还没动，确保不会和j t0以及t0+1或[t0+1, t0+2] 时刻的位置撞
                        for (int t = cur->g_value; t <= new_g_value; t++)
                        {
                            if (t < BoatRoutes[bj].size() && JudgeBoatCollision(nx, ny, ndir, BoatRoutes[bj][t].x, BoatRoutes[bj][t].y, BoatRoutes[bj][t].dir))
                            {
                                will_collision = true;
                                break;
                            }
                        }
                    }
                    if (will_collision)
                        break;
                }
                if (will_collision)
                    continue;

                auto find_node = all_nodes.find(BoatStateNodeKey(nx * N + ny, ndir, new_g_value));
                if (find_node == all_nodes.end())
                { // 之前没找过，则计算F值，G值，加入到开放列表并且调整小根堆(更新到节点哈希表)
                    // 到新节点的动作，如果是第一步，则记录move，否则和cur的action一样
                    // int action = (cur->pre_state == nullptr) ? move : cur->action;
                    shared_ptr<BoatStateNode> new_state_node = make_shared<BoatStateNode>(
                        nx, ny, ndir, move, bool(boat_state == 2 && move != 3), new_g_value, GetHValue(nx, ny, ndir, move, d_type, d_id), 1, cur);
                    all_nodes[BoatStateNodeKey(nx * N + ny, ndir, new_g_value)] = new_state_node;
                    openlist_heap.push_back(new_state_node);
                    push_heap(openlist_heap.begin(), openlist_heap.end(), CompareBoatStateNode());
                }
                else if (find_node->second->list_state == 1)
                { // 节点在Open列表中，如果G值更小，则更新G值，并且重新调整小根堆
                }
                else if (find_node->second->list_state == 2)
                { // 之前找过并且在关闭列表中，则什么都不干
                }
            }
        }
    }
    // 没有找到到目的地的路径，将本时刻的位置加入航线
    BoatRoutes[bi].clear();
    BoatRoutes[bi].push_back(BoatRouteState(Boats[bi].x, Boats[bi].y, Boats[bi].dir, 3));
    return;
}

// 船的调度
void BoatDispatch()
{
    // 记录所有需要寻路的船
    vector<bool> is_need_astar(BoatNum, false);
    for (int bi = 0; bi < BoatNum; bi++)
    { // 对于每艘船
        if (Boats[bi].status == 1)
        { // 恢复状态
          // 如果没有航线，加入当前位置（其实不需要，因为一定是在主航道范围内）
          // 如果有航线，就正常不动
        }
        else if (Boats[bi].status == 2)
        { // 装载状态进行讨论
            if (Boats[bi].goods_num >= BoatCapacity)
            { // 装满或者不够时间回交货点 就滚
                Berths[Boats[bi].dest_berth].boat_id = -1; // 清除港口目的地以及港口的船
                Boats[bi].dest_berth = -1;
                Boats[bi].dest_delivery = 0;               // 加入交货点目的地
                Boats[bi].status = 0;                      // 清空状态避免后面装货
                // 清空路径并且寻路（实际上靠泊的时候路径就一定清空了）
                BoatRoutes[bi].clear();
                is_need_astar[bi] = true;
            }
            else
            { // 没有装满，有货继续装，没货就调度
                if (Berths[Boats[bi].dest_berth].goods_queue.size() == 0)
                { // 港口上没货了
                }
            }
        }
        else
        { // 行驶状态
            // 先判断是不是到了目的地
            if (Boats[bi].dest_berth != -1 && IsInBerthRange(Boats[bi].x, Boats[bi].y) == Boats[bi].dest_berth)
            { // 到港口的话，进港装货(一定要确保能进去)
                printf("berth %d\n", bi);
            }
            else if (Boats[bi].dest_delivery != -1 && Boats[bi].x == Deliveries[Boats[bi].dest_delivery].x && Boats[bi].y == Deliveries[Boats[bi].dest_delivery].y)
            { // 到交货点的话，卸货然后再确定目的港口
                // 确定目标港口
                int best_berth = FindBestBerthFromDelivery(bi, Boats[bi].dest_delivery);
                Boats[bi].dest_delivery = -1;
                Boats[bi].dest_berth = best_berth;
                Berths[best_berth].boat_id = bi;
                // 确定港口之后是要寻路的
                is_need_astar[bi] = true;
            }
            else
            {
                if (BoatRoutes[bi].size() == 1)
                { // 新买的船
                    is_need_astar[bi] = true;
                }
                else if (BoatRoutes[bi].size() == 0)
                { // 上一帧没找到航线的船需要把自己当前状态加入防止碰撞
                    BoatRoutes[bi].push_back(BoatRouteState(Boats[bi].x, Boats[bi].y, Boats[bi].dir, 3));
                    is_need_astar[bi] = true;
                }
            }

            // //初始时等装满了再走
            // if (Berths[Boats[bi].pre_dest_berth].focus == 1 and Boats[bi].goods_num < BoatCapacity and 15000-Frame > DeliveryToBerthTime[0][Boats[bi].pre_dest_berth]){
            //     //有focus(初始或最后)、货没装满、剩余帧大于去0号交货点的时间（不是最后）（TODO:暂定）,则不寻路，等着
            //     !is_need_astar[bi];
            // }

            // //若装好了货，则把港口的focus去了
            // if (Boats[bi].dest_delivery != -1 && IsInBerthRange(Boats[bi].x, Boats[bi].y) == Boats[bi].pre_dest_berth){
            //     Berths[Boats[bi].pre_dest_berth].focus = 0;
            // }
        }
        // 对于偏航的船（要清空路径，只保留当前位置，然后重新寻路）
        if (BoatRoutes[bi].size() > 0 && (BoatRoutes[bi][0].x != Boats[bi].x || BoatRoutes[bi][0].y != Boats[bi].y || BoatRoutes[bi][0].dir != Boats[bi].dir))
        {
            BoatRoutes[bi].clear();
            BoatRoutes[bi].push_back(BoatRouteState(Boats[bi].x, Boats[bi].y, Boats[bi].dir, 3));
            is_need_astar[bi] = true;
        }
    }

    // 6帧给其他船让位的时间
    for (int bi = 0; bi < BoatNum; bi++)
    { // 考虑对需要寻路的船加入5帧（实际上是6帧，因为前面修改is_need_astar[bi]的时候一定把初始状态加进去了 route_size=1）
        if (is_need_astar[bi])
        {
            for (int i = 0; i < 5; i++)
            {
                BoatRoutes[bi].push_back(BoatRouteState(Boats[bi].x, Boats[bi].y, Boats[bi].dir, 3));
            }
        }
    }

    for (int bi = 0; bi < BoatNum; bi++)
    { // 考虑对需要寻路的船进行寻路
        if (is_need_astar[bi])
        {
            if (Boats[bi].dest_berth != -1)
            { // 目的地是港口
                BoatToPositionAStar(bi, 1, Boats[bi].dest_berth);
            }
            else
            { // 目的地是交货点
                BoatToPositionAStar(bi, 0, Boats[bi].dest_delivery);
            }
            // 重置
            if (BoatRoutes[bi].size() == 1)
            { // 没有找到航线，则记录下来
                BoatCanNotFindRoute[bi]++;
                if (BoatCanNotFindRoute[bi] >= 2)
                { // 如果2帧没找到，就重置
                    printf("dept %d\n", bi);
                    BoatCanNotFindRoute[bi] = 0;
                }
            }
            else
            { // 找到航线，更新
                BoatCanNotFindRoute[bi] = 0;
            }
        }
    }

    for (int bi = 0; bi < BoatNum; bi++)
    { // 根据船的路径执行动作，并且将这一帧的路径状态（路径的第一个点移出）
        if (BoatRoutes[bi].size() > 1)
        {
            int action = BoatRoutes[bi][1].action;
            if (action == 0 || action == 1)
            {
                printf("rot %d %d\n", bi, action);
            }
            else if (action == 2)
            {
                printf("ship %d\n", bi);
            }
        }
        if (BoatRoutes[bi].size() > 0)
        {
            BoatRoutes[bi].erase(BoatRoutes[bi].begin());
        }
    }
}

// 装载货物
void LoadGoods()
{
    for (int b = 0; b < BoatNum; b++)
    {
        if (Boats[b].status == 2)
        { // 如果是装载状态就装货（如果是装在状态但是走了，status就会改）
            int berth_index = Boats[b].dest_berth;
            for (int i = 0; i < min((int)Berths[berth_index].goods_queue.size(), Berths[berth_index].velocity); i++)
            {
                Boats[b].goods_num++; // 这个加不加无所谓，后一帧输入会覆盖掉
                Boats[b].goods_value += AllGoods[Berths[berth_index].goods_queue.front()].val;
                Berths[berth_index].goods_queue.pop();
            }
        }
    }
}

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

// 买一个机器人
void BuyARobot(int robot_buying_index)
{
    printf("lbot %d %d\n", RobotBuyings[robot_buying_index].x, RobotBuyings[robot_buying_index].y); // 输出指令
    Robots[RobotNum] = Robot(RobotBuyings[robot_buying_index].x, RobotBuyings[robot_buying_index].y); // 机器人的初始化
    RobotNum++;
    OurMoney -= ROBOT_BUY_MONEY; // 花钱
}

// 买一艘船，并指派其去的地方
void BuyABoat(int boat_buying_index, int berth_index)
{
    printf("lboat %d %d\n", BoatBuyings[boat_buying_index].x, BoatBuyings[boat_buying_index].y);     // 输出指令
    Boats[BoatNum] = Boat(BoatBuyings[boat_buying_index].x, BoatBuyings[boat_buying_index].y, 0, 0); // 船的初始化
    Boats[BoatNum].dest_berth = berth_index;
    Berths[berth_index].boat_id = BoatNum;                                                         // 船的目的地
    BoatNum++;
    OurMoney -= BOAT_BUY_MONEY; // 花钱
}

// 购买机器人,返回初始船只购买港口 (cxh)
void BuyRobotsCxh()
{
    if (Frame == 1)
    {
        memset(InitBuyingToBuy, -1, sizeof(InitBuyingToBuy));
        memset(InitBerthToGo, -1, sizeof(InitBerthToGo));
        // 初始购买
        priority_queue<BestBuy> best_buy;
        // 对每个轮船购买点
        for (int bbi = 0; bbi < BoatBuyingNum; bbi++)
        {
            // 对每个港口
            for (int bi = 0; bi < BerthNum; bi++)
            {
                if (BuyingToBerthTime[bbi][bi] < 0)
                {
                    // 船不可达的港口不去
                    continue;
                }
                else
                {
                    // 船可达的港口
                    // 对每个机器人购买点
                    for (int rbi = 0; rbi < RobotBuyingNum; rbi++)
                    {
                        int x = RobotBuyings[rbi].x;
                        int y = RobotBuyings[rbi].y;
                        if (BerthPathLength[bi][x][y] < 0)
                        {
                            // 机器人不可达
                            continue;
                        }
                        else
                        {
                            best_buy.emplace(BerthPathLength[bi][x][y] + BuyingToBerthTime[bbi][bi], rbi, bbi, bi);
                        }
                    }
                }
            }
        }
        // 根据船数目分配机器人
        if (best_buy.size() < InitBuyBoatNum)
        {
            // 若可达港口小于初始船只数
            int buy_robot_num = InitBuyRobotNum / (int)best_buy.size();
            int boat_index = 0;
            while (!best_buy.empty())
            {
                BestBuy choose = best_buy.top();
                if (Berths[choose.best_buy_berth].focus == 1){
                    //若这个方案是同一港口，则舍弃
                    best_buy.pop();
                    continue;
                }
                for (int i = 0; i < buy_robot_num; i++)
                {
                    if (InitBuyRobotNum == 0)
                    {
                        break;
                    }
                    else
                    {
                        printf("lbot %d %d\n", RobotBuyings[choose.best_buy_bot].x, RobotBuyings[choose.best_buy_bot].y);
                        InitBuyRobotNum--;
                    }
                }
                InitBuyingToBuy[boat_index] = choose.best_buy_boat;
                InitBerthToGo[boat_index] = choose.best_buy_berth;
                boat_index++;
                Berths[choose.best_buy_berth].focus = 1; // 初始标记只送这几个港口
                best_buy.pop();
            }
        }
        else
        {
            int buy_robot_num = InitBuyRobotNum / InitBuyBoatNum;
            int boat_index = 0;
            // 对所有船
            for (int bi = 0; bi < InitBuyBoatNum; bi++)
            {
                BestBuy choose = best_buy.top();
                if (Berths[choose.best_buy_berth].focus == 1){
                    //若这个方案是同一港口，则舍弃
                    best_buy.pop();
                    continue;
                }
                for (int i = 0; i < buy_robot_num; i++)
                {
                    if (InitBuyRobotNum == 0)
                    {
                        break;
                    }
                    else
                    {
                        printf("lbot %d %d\n", RobotBuyings[choose.best_buy_bot].x, RobotBuyings[choose.best_buy_bot].y);
                        InitBuyRobotNum--;
                    }
                }
                InitBuyingToBuy[boat_index] = choose.best_buy_boat;
                InitBerthToGo[boat_index] = choose.best_buy_berth;
                boat_index++;
                Berths[choose.best_buy_berth].focus = 1; // 初始标记只送这几个港口
                best_buy.pop();
            }
        }
    }
    else
    {
        // 后续购买
        // 统计囤积货物量
        int num_goods = 0;
        for (int bi = 0; bi < BerthNum; bi++)
        {
            num_goods += Berths[bi].goods_queue.size();
        }
        if ((15000 - Frame) * RobotEveryFrame <= 2000 || Money <= RobotBuyMoney || num_goods >= RobotBuyGoods || RobotNum >= RobotBuyNum)
        {
            // 再买就不礼貌了 或 没钱了 或 囤货太多 或 机器人太多
            return;
        }
        else
        {
            // 要买
            int goods_stack = 50; // 货物堆积最少处的最近位置买
            int length = MAX_LENGTH;
            int robot_buy_index;
            // 对每个轮船购买点
            for (int bbi = 0; bbi < BoatBuyingNum; bbi++)
            {
                // 对每个港口
                for (int bi = 0; bi < BerthNum; bi++)
                {
                    if (BuyingToBerthTime[bbi][bi] < 0)
                    {
                        // 船不可达的港口不去
                        continue;
                    }
                    else
                    {
                        // 船可达的港口
                        length = MAX_LENGTH;
                        // 对每个机器人购买点
                        for (int rbi = 0; rbi < RobotBuyingNum; rbi++)
                        {
                            int x = RobotBuyings[rbi].x;
                            int y = RobotBuyings[rbi].y;
                            if (BerthPathLength[bi][x][y] < 0)
                            {
                                // 机器人不可达
                                continue;
                            }
                            else
                            {
                                if (goods_stack >= Berths[bi].goods_queue.size())
                                {
                                    if (length >= BerthPathLength[bi][x][y])
                                    {
                                        goods_stack = Berths[bi].goods_queue.size();
                                        length = BerthPathLength[bi][x][y];
                                        robot_buy_index = rbi;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            printf("lbot %d %d\n", RobotBuyings[robot_buy_index].x, RobotBuyings[robot_buy_index].y);
        }
    }
}

// 购买船（cxh)
void BuyBoatsCxh()
{
    if (Frame == 1)
    {
        for (int i : InitBuyingToBuy)
        {
            if (i == -1)
                break;
            else
            {
                BuyABoat(i, InitBerthToGo[i]); // 购买船舶
            }
        }
    }
    else // 其余时刻船的购买策略
    {
        // 后续购买
        // 统计囤积货物量
        int num_goods = 0;
        for (int bi = 0; bi < BerthNum; bi++)
        {
            num_goods += Berths[bi].goods_queue.size();
        }
        if ((15000 - Frame) * BoatEveryFrame <= 8000 || Money <= BoatBuyMoney || num_goods >= BoatBuyGoods || BoatNum >= MAX_BUY_BOAT_NUM)
        {
            // 再买就不礼貌了 或 没钱了 或 囤货太少 或 船太多
            return;
        }
        else
        {
            int max_goods_value_distance = -1; // 最大货物价值距离
            int berth_index_to_go = -1;       // 要去的港口
            for (int be = 0; be < BerthNum; be++)
            {
                if (Berths[be].boat_id != -1)
                { // 忽略已有船的港口和已经忙碌的购买点附近的港口
                    continue;
                }
                int total_goods_value = 0;
                queue<int> tmp = Berths[be].goods_queue;
                for (int j = 0; j < min((int)tmp.size(), BoatCapacity); j++)
                {
                    total_goods_value += AllGoods[tmp.front()].val;
                    tmp.pop();
                }
                double value_distance = (double)total_goods_value / (double)(DeliveryToBerthTime[BerthNearestDelivery[be]][be] + BuyingToBerthTime[BerthNearestBuying[be]][be]);
                if (value_distance > max_goods_value_distance)
                {
                    max_goods_value_distance = total_goods_value;
                    berth_index_to_go = be;
                }
            }
            if (berth_index_to_go < 0){
                return;
            }
            BuyABoat(BerthNearestBuying[berth_index_to_go], berth_index_to_go); // 购买船舶
        }
    }
    return;
}

// 购买机器人，用最大数量以及船的数量来限制，能买就买 (xmc)
int BuyRobotsXmc()
{
    int buy = 0;
    if (Frame == 1)
    {
        // 初始购买
        priority_queue<BestBuy> best_buy;
        // 对每个轮船购买点
        for (int bbi = 0; bbi < BoatBuyingNum; bbi++)
        {
            // 对每个港口
            for (int bi = 0; bi < BerthNum; bi++)
            {
                if (BuyingToBerthTime[bbi][bi] < 0)
                {
                    // 船不可达的港口不去
                    continue;
                }
                else
                {
                    // 船可达的港口
                    // 对每个机器人购买点
                    for (int rbi = 0; rbi < RobotBuyingNum; rbi++)
                    {
                        int x = RobotBuyings[rbi].x;
                        int y = RobotBuyings[rbi].y;
                        if (BerthPathLength[bi][x][y] < 0)
                        {
                            // 机器人不可达
                            continue;
                        }
                        else
                        {
                            best_buy.emplace(BerthPathLength[bi][x][y] + BuyingToBerthTime[bbi][bi], rbi, bbi, bi);
                        }
                    }
                }
            }
        }
        // 根据船数目分配机器人
        if (best_buy.size() < InitBuyBoatNum)
        {
            // 若可达港口小于初始船只数
            int buy_robot_num = InitBuyRobotNum / (int)best_buy.size();
            int boat_index = 0;
            while (!best_buy.empty())
            {
                BestBuy choose = best_buy.top();
                for (int i = 0; i < buy_robot_num; i++)
                {
                    if (InitBuyRobotNum == 0)
                    {
                        break;
                    }
                    else
                    {
                        BuyARobot(choose.best_buy_bot);
                        InitBuyRobotNum--;
                    }
                }
                InitBuyingToBuy[boat_index] = choose.best_buy_boat;
                InitBerthToGo[boat_index] = choose.best_buy_berth;
                boat_index++;
                Berths[choose.best_buy_berth].focus = 1; // 初始标记只送这几个港口
                best_buy.pop();
            }
        }
        else
        {
            int buy_robot_num = InitBuyRobotNum / InitBuyBoatNum;
            int boat_index = 0;
            // 对所有船
            for (int bi = 0; bi < InitBuyBoatNum; bi++)
            {
                BestBuy choose = best_buy.top();
                for (int i = 0; i < buy_robot_num; i++)
                {
                    if (InitBuyRobotNum == 0)
                    {
                        break;
                    }
                    else
                    {
                        BuyARobot(choose.best_buy_bot);
                        InitBuyRobotNum--;
                    }
                }
                InitBuyingToBuy[boat_index] = choose.best_buy_boat;
                InitBerthToGo[boat_index] = choose.best_buy_berth;
                boat_index++;
                Berths[choose.best_buy_berth].focus = 1; // 初始标记只送这几个港口
                best_buy.pop();
            }
        }
    }
    else
    {  // 后续购买
        if ( Money < ROBOT_BUY_MONEY || RobotNum >= MAX_BUY_ROBOT_NUM || RobotNum >= ROBOT_BOAT_PROPORTION * (BoatNum + 1))
        {
            return buy;
        }
        else
        {  // 要买
            int goods_stack = MAX_GOODS_NUM; // 货物堆积最少处的最近位置买
            int robot_buy_index;

            // 对每个港口
            for (int bi = 0; bi < BerthNum; bi++)
            {
                if (BerthNearestBuying[bi] < 0)
                {  // 船不可达的港口不去
                    continue;
                }
                else
                {  // 船可达的港口
                    int length = MAX_LENGTH;
                    // 对每个机器人购买点
                    for (int rbi = 0; rbi < RobotBuyingNum; rbi++)
                    {
                        int x = RobotBuyings[rbi].x;
                        int y = RobotBuyings[rbi].y;
                        if (BerthPathLength[bi][x][y] < 0)
                        {  // 机器人不可达
                            continue;
                        }
                        else
                        {
                            if (goods_stack >= Berths[bi].goods_queue.size() && length >= BerthPathLength[bi][x][y])
                            {
                                goods_stack = (int) Berths[bi].goods_queue.size();
                                length = BerthPathLength[bi][x][y];
                                robot_buy_index = rbi;
                            }
                        }
                    }
                }
            }
            BuyARobot(robot_buy_index);
            buy = 1;
        }
    }
    return buy;
}

// 购买船舶 (用最大数量来限制，能买就买，买来去当时最多货的港口)
int BuyBoatsXmc()
{
    int buy = 0;
    if (Frame == 1)
    {
        for (int i : InitBuyingToBuy)
        {
            if (i == -1)
                break;
            else
            {
                BuyABoat(i, InitBerthToGo[i]); // 购买船舶
                buy = 1;
            }
        }
    }
    else // 其余时刻船的购买策略
    {
        if (Money < BOAT_BUY_MONEY || BoatNum >= MAX_BUY_BOAT_NUM) // 金钱不够起限额或者超过最大购船数量，不买船
        {
            return buy;
        }
        else
        {
            int max_goods_value_distance = 0; // 最大货物价值距离
            int berth_index_to_go = -1;       // 要去的港口
            for (int be = 0; be < BerthNum; be++)
            {
                if (Berths[be].boat_id != -1)
                { // 忽略已有船的港口和已经忙碌的购买点附近的港口
                    continue;
                }
                int total_goods_value = 0;
                queue<int> tmp = Berths[be].goods_queue;
                for (int j = 0; j < min((int)tmp.size(), BoatCapacity); j++)
                {
                    total_goods_value += AllGoods[tmp.front()].val;
                    tmp.pop();
                }
                int value_distance;
                if ((value_distance = total_goods_value /
                                      (DeliveryToBerthTime[BerthNearestDelivery[be]][be] + BuyingToBerthTime[BerthNearestBuying[be]][be])) > max_goods_value_distance)
                {
                    max_goods_value_distance = value_distance;
                    berth_index_to_go = be;
                }
            }
            BuyABoat(BerthNearestBuying[berth_index_to_go], berth_index_to_go); // 购买船舶
            buy = 1;
        }
    }
    return buy;
}

// 购买函数
void Buy()
{
    if (BuyChoose == 1){
        while (BuyRobotsXmc() || BuyBoatsXmc())
        ;
    }
    else{
        BuyRobotsCxh();
        BuyBoatsCxh();
    }

}

// 购买船舶
void BuyBoats1()
{
    if (Frame == 1 || Frame == 10)
    {
        // 初始购买（买几艘船测试寻路用）
        for (int bbi = 0; bbi < BoatBuyingNum && Money >= 8000; bbi++)
        { // 对于每个购买点有钱就购买船
            printf("lboat %d %d\n", BoatBuyings[bbi].x, BoatBuyings[bbi].y);
            Boats[BoatNum] = Boat(BoatBuyings[bbi].x, BoatBuyings[bbi].y, 0, 0);
            // 将当前的位置加入航线路径
            BoatRoutes[BoatNum].clear();
            BoatRoutes[BoatNum].push_back(BoatRouteState(BoatBuyings[bbi].x, BoatBuyings[bbi].y, 0, 3));

            for (int bi = 0; bi < BerthNum; bi++)
            {
                if (Berths[bi].boat_id == -1)
                {
                    Boats[BoatNum].dest_berth = bi;
                    Berths[bi].boat_id = BoatNum;
                    break;
                }
            }
            BoatNum++;
            Money -= 8000;
        }
    }
}

// 输出实际金钱与我们自己算的金钱
void PrintMoney(ofstream &out_file)
{
    out_file << "Real money: " << Money << endl;
    out_file << "Our  money: " << OurMoney << endl;
}

// 输出每个港口的货物量，货物总价值以及是否聚焦，以及装载速度
void PrintBerthGoodsInfo(ofstream &out_file)
{
    int berth_goods_value[MAX_BERTH_NUM] = {0};
    int berth_goods_num[MAX_BERTH_NUM] = {0};
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
        out_file << "Berth " << i << " " << left << setw(3) << berth_goods_num[i] << " goods, value "
                 << setw(5) << berth_goods_value[i] << ", focus " << Berths[i].focus 
                 << " boat " << setw(3) << Berths[i].boat_id << " velocity " << setw(3) << Berths[i].velocity << endl;
    }
}

// 输出每艘船的信息
void PrintBoatInfo(ofstream &out_file)
{
    for (int i = 0; i < BoatNum; i++)
    {
        out_file << "Boat " << i << " status " << Boats[i].status
                 << ", at (" << setw(3) << Boats[i].x << "," << setw(3) << Boats[i].y << ") "
                 << "dir " << Boats[i].dir << ", dest berth "
                 << setw(2) << Boats[i].dest_berth << ", dest delivery "
                 << setw(2) << Boats[i].dest_delivery << ", goods num "
                 << setw(3) << Boats[i].goods_num << endl;
    }
}

// 输出机器人所获得的总金额
void PrintRobotsMoney(ofstream &out_file)
{
    out_file << "----------------------------------------Robot Money-------------------------------------------" << endl;
    out_file << "Robot Money: " << RobotMoney << endl;
}

// 输出地图信息
void PrintWorldInfo(ofstream &out_file)
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (IsInBerthRange(i, j) == -1)
            {
                out_file << '.';
            }
            else
            {
                out_file << IsInBerthRange(i, j);
            }
        }
        out_file << endl;
    }
}

// 输出船的容量
void PrintBoatCapacity(ofstream &out_file)
{
    out_file << "----------------------------------------Boat Capacity-----------------------------------------" << endl;
    out_file << "Boat Capacity: " << BoatCapacity << endl;
}

// 输出每个交货点到每个港口时间
void PrintDeliveryToBerthTime(ofstream &out_file)
{
    out_file << "-----------------------------------Delivery To Berth Time-------------------------------------" << endl;
    for (int di = 0; di < DeliveryNum; di++)
    {
        for (int bi = 0; bi < BerthNum; bi++)
        {
            out_file << "Delivery " << di << " to Berth " << bi << " Time: " << left << setw(6) << DeliveryToBerthTime[di][bi];
            out_file << " Search Node:" << AStarSearchNodeNum[di * DeliveryNum + bi] << endl;
        }
    }
}

// 输出船舶购买点到每个港口的时间
void PrintBuyingToBerthTime(ofstream &out_file)
{
    out_file << "------------------------------------Buying To Berth Time--------------------------------------" << endl;
    for (int bbi = 0; bbi < BoatBuyingNum; bbi++)
    {
        for (int bi = 0; bi < BerthNum; bi++)
        {
            out_file << "Buying   " << bbi << " to Berth " << bi << " Time: " << left << setw(6) << BuyingToBerthTime[bbi][bi];
            out_file << " Search Node:" << AStarSearchNodeNum[DeliveryNum * BerthNum + bbi * BoatBuyingNum + bi] << endl;
        }
    }
}

// 输出每艘船的航线
void PrintBoatRouts(ofstream &out_file)
{
    for (int bi = 0; bi < BoatNum; bi++)
    {
        for (int i = 0; i < BoatRoutes[bi].size(); i++)
        {
            out_file << "(" << BoatRoutes[bi][i].x << "," << BoatRoutes[bi][i].y << "," << BoatRoutes[bi][i].dir << ") ";
        }
        out_file << endl;
    }
}

// 输出初始购买信息
void PrintInitBuy(ofstream &out_file)
{
    out_file << "----------------------------------------Init Buy Info-----------------------------------------" << endl;
    out_file << "Init Buy Robot Num: " << InitBuyRobotNum << endl;
    out_file << "Init Buy Boat Num: " << InitBuyBoatNum << endl;
    for (int i = 0; i < MAX_BOAT_NUM; i++)
    {
        if (InitBuyingToBuy[i] == -1 || InitBerthToGo[i] == -1)
            break;
        out_file << "Init Berth to go: " << InitBerthToGo[i] << endl;
        out_file << "Init Boat Buying: " << InitBuyingToBuy[i] << endl;
    }
}

// 输出信息
void Print(ofstream &out_file, int interval)
{
    if (Frame == 0)
    {
        // PrintWorldInfo(out_file);
        PrintDeliveryToBerthTime(out_file);
        PrintBuyingToBerthTime(out_file);
        PrintBoatCapacity(out_file);
    }

    if (Frame == 1)
        PrintInitBuy(out_file);

    if (Frame % interval != 0)
    {
        return;
    }

    if (out_file.is_open())
    {
        out_file << "----------------------------------------Frame: " << left << setw(5) << Frame << "------------------------------------------" << endl;
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
        RobotDispatchGreedy();
        AvoidCollision();
        PrintRobotsIns();
        BoatDispatch();
        LoadGoods();
        Buy();
        puts("OK");
        fflush(stdout);
    }

#ifdef DEBUG
    out_file.close();
#endif

    return 0;
}