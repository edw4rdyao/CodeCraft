#ifdef _WIN32
#include <bits/stdc++.h>
#else
#include <cstdlib>
#include <queue>
#include <vector>
#include <cstdio>
#include <algorithm>
#include <set>
#include <random>
#include <cstring>
#include <cstdlib> // 包含 rand() 和 srand()
#include <ctime>   // 包含 time()
#endif

using namespace std;

const int ROBOT_NUM = 10;
const int BERTH_NUM = 10;
const int BOAT_NUM = 5;
const int N = 200;
const int MAX_GOODS_NUM = 150000;

const int MAX_ROAD_NUM = 5;
const int MAX_ROAD_LEN = 50;

const int DX[4] = {-1, 1, 0, 0};     // 每个方向x轴的偏移
const int DY[4] = {0, 0, -1, 1};     // 每个方向y轴的偏移
const int REV_DIR[4] = {1, 0, 3, 2}; // 上下左右的反方向（用于BFS记录路径）：下上右左

int Money, BoatCapacity, Frame;      // 金钱，船的容量，当前帧数
int World[N][N];                     // 地图
int BerthPath[BERTH_NUM][N][N];      // 泊位到每个点的最短路径(0: 上，1: 下，2: 左，3: 右)
int BerthPathLenth[BERTH_NUM][N][N]; // 泊位到每个点的最短路径长度

struct Goods
{
    int x, y;          // 货物的坐标
    int val;           // 货物的价值
    int refresh_frame; // 货物的刷新帧数
    // int lock;          // 货物是否被机器人锁定(0:未锁定)

    Goods() {}
    Goods(int x, int y, int val, int refresh_frame)
    {
        this->x = x;
        this->y = y;
        this->val = val;
        this->refresh_frame = refresh_frame;
    }
} AllGoods[MAX_GOODS_NUM];
int NextGoodsIndex = 0; // 下一个货物的编号

struct Robot
{
    int x, y;           // 机器人的坐标
    int is_goods;       // 机器是否携带货物 (0：没有，1：有)
    int status;         // 机器人的状态（0：恢复，1：运行）
    int dir;            // 机器人下一步运动的方向
    int goods_index;    // 机器人携带/想携带的货物编号
    int goods_distance; // 机器人和货物的距离
    int berth_index;    // 机器人要去的泊位编号
    int is_dead;        // 机器人是否被困死（0：没有，1：有）
    int action;         // 0:get()拿货物， 1:pull()放物品, -1没有动作

    Robot() {}
    Robot(int x, int y)
    {
        this->x = x;
        this->y = y;
        this->is_goods = 0;
        this->status = 1;
        this->dir = -1;
        this->goods_index = -1;
        this->goods_distance = 40001;
        this->berth_index = -1;
        this->is_dead = 0;
    }

} Robots[ROBOT_NUM];

struct Berth
{
    int x, y;               // 泊位的坐标
    int transport_time;     // 货物运输时间
    int loading_speed;      // 货物装卸速度
    queue<int> goods_queue; // 泊位上的货物队列

    Berth() {}
    Berth(int x, int y, int transport_time, int loading_speed)
    {
        this->x = x;
        this->y = y;
        this->transport_time = transport_time;
        this->loading_speed = loading_speed;
    }
    int sum_goods_value()
    {
        int sum = 0;
        queue<int> temp = this->goods_queue;
        for (int i = 0; i < temp.size(); i++)
        {
            sum += AllGoods[temp.front()].val;
            temp.pop();
        }
        return sum;
    }
} Berths[BERTH_NUM];

struct Boat
{
    int pos;       // 船的位置（港口id）（-1: 在虚拟点）
    int status;    // 船的状态（0：移动或运输中，1：装货或运输完成，2:泊位外等待）
    int goods_num; // 船上货物的数量

    Boat() {}
    Boat(int pos, int status)
    {
        this->pos = pos;
        this->status = status;
        this->goods_num = 0;
    }
    int find_best_berth(set<int> busy_berth) // 返回最佳泊位编号
    {
        double value_time = 0;
        int berth_tmp = -1;
        for (int i = 0; i < BERTH_NUM; i++)
        {
            if (Berths[i].goods_queue.empty()) // 泊位上没有货物
            {
                return -1;
            }
            else if (busy_berth.find(i) == busy_berth.end()) // 泊位是空闲的
            {
                int load_time = Berths[i].goods_queue.size() / Berths[i].loading_speed;
                double value_time_tmp = (double)Berths[i].sum_goods_value() / load_time;
                if (value_time_tmp > value_time)
                {
                    value_time = value_time_tmp;
                    berth_tmp = i;
                }
            }
        }
        return berth_tmp;
    }

    void load_goods() // 装货
    {
        if (this->status == 1 && this->pos != -1) // 船只在装货或装货完成
        {
            if (this->goods_num < BoatCapacity && Berths[this->pos].goods_queue.size() > 0)
            {
                int add_num;
                add_num = min(BoatCapacity - this->goods_num, int(Berths[this->pos].goods_queue.size()));
                add_num = min(add_num, Berths[this->pos].loading_speed);
                for (int i = 0; i < add_num; i++)
                {
                    this->goods_num += 1;
                    Berths[this->pos].goods_queue.pop();
                }
            }
            else
            {
                return;
            }
        }
    }
} Boats[BOAT_NUM];

// 返回泊位上有货物的泊位编号
set<int> BusyBerth()
{
    set<int> busy_berth;
    for (int i = 0; i < BOAT_NUM; i++)
    {
        if (Boats[i].pos != -1)
        {
            busy_berth.insert(Boats[i].pos);
        }
    }
    return busy_berth;
}

// 检查坐标(x, y)是否在地图范围内并且不是障碍物或者海洋
bool IsValid(int x, int y)
{
    return x >= 0 && x < N && y >= 0 && y < N && World[x][y] >= 0;
}

// 判断是否在港口内
bool IsInBerth(int robot_index)
{
    if (World[Robots[robot_index].x][Robots[robot_index].y] == 3)
    {
        return true;
    }
    return false;
}

// 判断是否在物品上，如果在则返回物品ID，否则返回-2
int IsOnGoods(int x, int y)
{
    int goods_id = World[x][y] - 100;
    if (goods_id >= 0)
    {
        return goods_id;
    }
    return -100;
}

// 计算物品性价比
double CalculateGoodsValue(int goods_index, int step_num, int &to_berth_index)
{
    int goods_value;          // 物品价值
    int goods_x, goods_y;     // 物品坐标
    int to_berth_len = 40001; // 物品到港口距离
    goods_value = AllGoods[goods_index].val;
    goods_x = AllGoods[goods_index].x;
    goods_y = AllGoods[goods_index].y;

    // 确定最近港口
    for (int i = 0; i < BERTH_NUM; i++)
    {
        int length = BerthPathLenth[i][goods_x][goods_y];
        if (length >= 0 && length <= to_berth_len)
        {
            to_berth_len = length;
            to_berth_index = i;
        }
    }
    double cost_value = (double)goods_value / (step_num + to_berth_len);
    return cost_value;
}

void BoatDispatch()
{
    for (int i = 0; i < BOAT_NUM; i++)
    {
        if (Boats[i].status == 0) // 船只在移动中
        {
            continue;
        }
        else if (Boats[i].status == 1) // 船只在装货或运输完成
        {
            if (Boats[i].pos == -1) // 船只在虚拟点
            {
                int best_berth = Boats[i].find_best_berth(BusyBerth());
                if (best_berth != -1)
                {
                    printf("ship %d %d\n", i, best_berth);
                }
                else
                    return;
            }
            else // 船只在港口
            {
                if (Boats[i].goods_num == BoatCapacity) // 船只装满货物
                {
                    printf("go %d\n", i);
                }
                else
                {
                    Boats[i].load_goods();
                    int min_distance = 40001;
                    for (int j = 0; j < ROBOT_NUM; j++)
                        if (Robots[j].berth_index == Boats[i].pos && Robots[j].is_goods == 1) // 机器人锁定港口且带货
                        {
                            int distance = BerthPathLenth[Robots[j].berth_index][Robots[j].x][Robots[j].y];
                            if (distance < min_distance)
                            {
                                min_distance = distance;
                            }
                        }
                    if (min_distance > Berths[Boats[i].pos].transport_time) // 没有机器人锁定港口且带货
                    {
                        printf("go %d\n", i);
                    }
                }
            }
        }
        else if (Boats[i].status == 2) // 船只在等待
        {
            int best_berth = Boats[i].find_best_berth(BusyBerth());
            if (best_berth != -1)
            {
                printf("ship %d %d\n", i, best_berth);
            }
            else
                return;
        }
    }
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

void ToBerthBFS()
{
    // 初始化所有泊位到每个点的最短路径及长度，均为-1
    memset(BerthPath, -1, sizeof(BerthPath));
    memset(BerthPathLenth, -1, sizeof(BerthPathLenth));
    // 对每个泊位（左上角->中间点）开始做BFS
    for (int b = 0; b < BERTH_NUM; b++)
    {
        int berth_x = Berths[b].x + 1;
        int berth_y = Berths[b].y + 1;
        queue<pair<int, int>> q;
        q.push({berth_x, berth_y});
        // 泊位本身的路径长度为0
        BerthPathLenth[b][berth_x][berth_y] = 0;
        while (!q.empty())
        {
            // 从队列中取出一个点
            pair<int, int> cur_pos = q.front();
            q.pop();
            // 四个方向随机遍历
            vector<int> random_dir = GetRandomDirection();
            for (int i = 0; i < 4; i++)
            { // 遍历到下一个点
                int dir = random_dir[i];
                int nx = cur_pos.first + DX[dir];
                int ny = cur_pos.second + DY[dir];
                if (IsValid(nx, ny) && BerthPathLenth[b][nx][ny] < 0)
                { // 判断该点是否可以到达(没有越界&&为空地或者泊位&&之前没有到达过)
                    // 路径长度+1
                    BerthPathLenth[b][nx][ny] = BerthPathLenth[b][cur_pos.first][cur_pos.second] + 1;
                    // 记录路径的方向
                    BerthPath[b][nx][ny] = REV_DIR[dir];
                    // 将该点加入队列
                    q.push({nx, ny});
                }
            }
        }
    }
}

void JudgeRobotsLife()
{
    // 对每个机器人进行检查
    for (int r = 0; r < ROBOT_NUM; r++)
    {
        int can_get = 0;
        // 对每个港口进行检查
        for (int b = 0; b < BERTH_NUM; b++)
        {
            if (BerthPathLenth[b][Robots[r].x][Robots[r].y] >= 0)
            {
                can_get++;
            }
        }
        // 如果对每个港口都不可达，则被困死
        if (can_get == 0)
        {
            Robots[r].is_dead = 1;
        }
    }
}

void Init()
{
    // 读取地图
    int r_num = 0;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            char ch;
            scanf("%c", &ch);
            if (ch == '.')
            {
                World[i][j] = 0; // 空地
            }
            else if (ch == '*')
            {
                World[i][j] = -1; // 海洋
            }
            else if (ch == '#')
            {
                World[i][j] = -2; // 障碍
            }
            else if (ch == 'B')
            {
                World[i][j] = 3; // 港口
            }
            else if (ch == 'A')
            {
                // 有机器人的空地
                World[i][j] = 0;
                // 机器人的初始位置(i,j)，用robots数组存下来
                Robots[r_num++] = Robot(i, j);
            }
            else
            {
                World[i][j] = -3; // 其他
            }
        }
        char ch;
        scanf("%c", &ch);
    }

    // 初始化泊位的信息
    for (int i = 0; i < BERTH_NUM; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &Berths[id].x, &Berths[id].y, &Berths[id].transport_time, &Berths[id].loading_speed);
    }
    // 读入船的容量
    scanf("%d", &BoatCapacity);
    // ok
    char ok[100];
    scanf("%s", ok);

    for (int i = 0; i < BOAT_NUM; i++)
    {
        Boats[i] = Boat(-1, 1);
    }

    // BFS存储每个港口到地图每个点最短路径和距离
    ToBerthBFS();
    // 判断每个机器人和所有港口的可达性（如果都不可达则被困死）
    JudgeRobotsLife();

    // ok
    printf("OK\n");
    fflush(stdout);
}

void Input()
{
    // 更新当前帧数和分数
    scanf("%d%d", &Frame, &Money);
    // 添加新增的物品
    int goods_num;
    scanf("%d", &goods_num);
    for (int i = 0; i < goods_num; i++)
    {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        AllGoods[NextGoodsIndex] = Goods(x, y, val, Frame);
        World[x][y] = 100 + NextGoodsIndex;
        NextGoodsIndex++;
    }
    // 同步机器人的位置
    for (int i = 0; i < ROBOT_NUM; i++)
    {
        int is_goods, x, y, status;
        scanf("%d%d%d%d", &is_goods, &x, &y, &status);
        // 更新机器人的状态
        Robots[i].x = x;
        Robots[i].y = y;
        Robots[i].is_goods = is_goods;
        Robots[i].status = status;
        // 重置动作，商品id以及方向
        Robots[i].action = -1;
        Robots[i].dir = -1;
    }
    // 更新船的状态和位置
    for (int i = 0; i < BOAT_NUM; i++)
    {
        int status, pos;
        scanf("%d%d\n", &status, &pos);
        // 当帧数不为1时，轮船的状态和目标点才有效（轮船的初始位置均认为在虚拟点）
        if (Frame != 1)
        {
            Boats[i].status = status;
            Boats[i].pos = pos;
        }
    }

    // ok
    char ok[100];
    scanf("%s", ok);
}

// 没拿物品的机器人之间比较要拿物品的性价比
bool NoGoodsRobotsCompair(int ri, int rj)
{
    int goodsi = Robots[ri].goods_index;                                                                                                                         // 机器人i要拿的物品
    int goodsj = Robots[rj].goods_index;                                                                                                                         // 机器人j要拿的物品
    double ri_val = (double)AllGoods[goodsi].val / (Robots[ri].goods_distance + BerthPathLenth[Robots[ri].berth_index][AllGoods[goodsi].x][AllGoods[goodsi].y]); // 机器人i要拿物品的性价比
    double rj_val = (double)AllGoods[goodsj].val / (Robots[rj].goods_distance + BerthPathLenth[Robots[rj].berth_index][AllGoods[goodsj].x][AllGoods[goodsj].y]); // 机器人j要拿物品的性价比
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
    int goodsi = Robots[ri].goods_index;                                                                               // 机器人i拿的物品
    int goodsj = Robots[rj].goods_index;                                                                               // 机器人j拿的物品
    double ri_val = (double)AllGoods[goodsi].val / BerthPathLenth[Robots[ri].berth_index][Robots[ri].x][Robots[ri].y]; // 机器人i物品的性价比
    double rj_val = (double)AllGoods[goodsj].val / BerthPathLenth[Robots[rj].berth_index][Robots[rj].x][Robots[rj].y]; // 机器人j物品的性价比
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
    int robot_index; // 路径对应的机器人id
    int goods_index; // 路径对应的物品id
    int goods_distance;
    int berth_index; // 最近港口id
    int next_dir;    // 机器人下一步方向
    double value;    // 性价比

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

// struct Road // 物品id、最近港口、机器人下一步方向
// {
//     int goods_index;
//     int berth_index;
//     int next_dir;

//     Road() {}
//     Road(int goods_index, int berth_index, int next_dir)
//     {
//         this->goods_index = goods_index;
//         this->berth_index = berth_index;
//         this->next_dir = next_dir;
//     }
// };

// // 使用BFS算法找到最好的3个物品,返回物品index、最近港口和下一步方向
// vector<Road> ToGoodsBFS(int robot_index, int total_step)
// {
//     int robot_x, robot_y; // 机器人的坐标
//     robot_x = Robots[robot_index].x;
//     robot_y = Robots[robot_index].y;

//     int visited[N][N] = {0}; // 访问标记数组
//     queue<pair<int, int>> q; // BFS队列,坐标值
//     queue<int> direction;    // 用于记录到达每个位置的移动方向
//     vector<Road> best_goods; // 保存返回的物品

//     q.push({robot_x, robot_y});    // 将机器人初始位置入队
//     visited[robot_x][robot_y] = 1; // 标记为已访问
//     direction.push(-1);            // -1表示初始位置，没有实际方向意义

//     int step = 0; // 当前步数
//     while (!q.empty() && step < total_step)
//     {
//         int min_val = 201; // 当前最小价值
//         int mid_val;       // 第二小价值
//         int max_val = 0;   // 当前最大价值

//         int size = q.size();
//         step++;
//         for (int i = 0; i < size; ++i)
//         {
//             auto [x, y] = q.front();
//             q.pop();
//             int dir = direction.front();
//             direction.pop();

//             if (World[x][y] - 100 >= 0 && step <= AllGoods[World[x][y] - 100].refresh_frame)
//             { // 找到物品
//                 int berth_index;
//                 double current_val = CalculateGoodsValue(World[x][y] - 100, step, berth_index); // 计算物品性价比
//                 if (best_goods.size() < 3)
//                 { // 已找到小于三个
//                     if (min_val > current_val)
//                     { // 最小价值总是插在末尾
//                         if (min_val != 201)
//                         { // 不是第一个插入的
//                             mid_val = min_val;
//                         }
//                         min_val = current_val;
//                         best_goods.push_back(Road(World[x][y] - 100, berth_index, dir));
//                     }
//                     else
//                     {
//                         if (max_val < current_val)
//                         { // 最大的插在开头
//                             max_val = current_val;
//                             best_goods.insert(best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
//                         }
//                         else
//                         {
//                             mid_val = max_val;
//                             best_goods.insert(++best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
//                         }
//                     }
//                 }
//                 else
//                 { // 保留最大的3个
//                     if (min_val <= current_val)
//                     { // 最小价值总是插在末尾
//                         if (max_val < current_val)
//                         { // 当前最大
//                             min_val = mid_val;
//                             mid_val = max_val;
//                             max_val = current_val;
//                             best_goods.insert(best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
//                             best_goods.pop_back(); // 弹出最小的
//                         }
//                         else if (mid_val <= current_val)
//                         { // 当前为第二小
//                             min_val = mid_val;
//                             mid_val = current_val;
//                             best_goods.pop_back(); // 弹出最小的
//                             best_goods.insert(++best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
//                         }
//                         else
//                         { // 当前最小
//                             min_val = current_val;
//                             best_goods.pop_back();
//                             best_goods.push_back(Road(World[x][y] - 100, berth_index, dir));
//                         }
//                     }
//                 }
//             }

//             for (int d = 0; d < 4; ++d)
//             { // 检查四个方向
//                 int nx = x + DX[d];
//                 int ny = y + DY[d];

//                 if (IsValid(nx, ny) && !visited[nx][ny])
//                 {                        // 检查下一步是否可走
//                     visited[nx][ny] = 1; // 标记已走过
//                     q.push({nx, ny});    // 存储下一步位置
//                     direction.push(d);   // 存储对应的方向
//                 }
//             }
//         }
//     }
//     return best_goods;
// }

// // 每个机器人本帧动作
// void RobotDispatch()
// {
//     vector<vector<Road>> roads; // 每个机器人各自最好的几个商品和对应的下一步
//     for (int i = 0; i < ROBOT_NUM; i++)
//     {
//         vector<Road> current_road; // 当前机器人BFS结果
//         if (Robots[i].is_dead == 1 or Robots[i].status == 0)
//         { // 若机器人不能到港口就不考虑
//             roads.push_back(current_road);
//             Robots[i].dir = -1;
//             continue;
//         }

//         if (Robots[i].is_goods == 1)
//         { // 若拿着物品
//             roads.push_back(current_road);
//             if (IsInBerth(i))
//             { // 到达港口
//                 Robots[i].action = 1;
//                 Robots[i].dir = -1;
//             }
//             else
//             { // 没到港口，沿着图走
//                 Robots[i].dir = BerthPath[Robots[i].berth_index][Robots[i].x][Robots[i].y];
//             }
//             continue;
//         }

//         current_road = ToGoodsBFS(i, 80);
//         roads.push_back(current_road); // 最多找80步
//     }

//     for (int i = 0; i < ROBOT_NUM; i++)
//     {
//         if (roads[i].empty())
//         { // 若没有bfs到物品或机器人不能到港口或已经拿了物品
//             if (Robots[i].is_dead != 1 && Robots[i].is_goods != 1)
//             { // 没拿物品也没死
//                 Robots[i].dir = -1;
//             }
//             continue;
//         }

//         if (IsInGoods(i))
//         {                                             // 在物品上
//             AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
//             int j = 0;
//             for (j = 0; j < roads[i].size(); j++)
//             {
//                 int goods_id = roads[i][j].goods_index;
//                 if (AllGoods[goods_id].lock == 0 && AllGoods[goods_id].x == Robots[i].x && AllGoods[goods_id].y == Robots[i].y)
//                 { // 确认过眼神，是我要拿的人
//                     // 拿物品
//                     AllGoods[goods_id].lock = 1;                                                // 物品上锁
//                     Robots[i].goods_index = goods_id;                                           // 机器人拿的物品编号
//                     Robots[i].berth_index = roads[i][j].berth_index;                            // 要去的港口编号
//                     Robots[i].action = 0;                                                       // 拿物品的动作
//                     Robots[i].dir = BerthPath[Robots[i].berth_index][Robots[i].x][Robots[i].x]; // 按图找方向
//                     World[Robots[i].x][Robots[i].x] = 0;                                        // 地图上去除物品
//                     break;
//                 }
//             }
//             if (j == roads[i].size())
//             { // 不是我要拿的
//                 // 按road走
//                 int k = 0;
//                 for (k = 0; k < roads[i].size(); k++)
//                 {
//                     int goods_id = roads[i][k].goods_index;
//                     if (AllGoods[goods_id].lock == 0)
//                     {
//                         if (Robots[i].goods_index != goods_id)
//                         {
//                             AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
//                         }
//                         Robots[i].goods_index = goods_id;     // 机器人要拿的物品编号
//                         AllGoods[goods_id].lock = 1;          // 物品上锁
//                         Robots[i].dir = roads[i][k].next_dir; // 机器人方向
//                         Robots[i].berth_index = roads[i][k].berth_index; //物品最近港口
//                         break;
//                     }
//                 }
//                 if (k == roads[i].size())
//                 {                                             // 没东西拿
//                     AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
//                     Robots[i].dir = -1;                       // 罚站
//                 }
//             }
//         }
//         else
//         { // 按road走
//             int j = 0;
//             for (j = 0; j < roads[i].size(); j++)
//             {
//                 int goods_id = roads[i][j].goods_index;
//                 if (AllGoods[goods_id].lock == 0)
//                 {
//                     if (Robots[i].goods_index != goods_id)
//                     {
//                         AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
//                     }
//                     Robots[i].goods_index = goods_id;     // 机器人要拿的物品编号
//                     AllGoods[goods_id].lock = 1;          // 物品上锁
//                     Robots[i].dir = roads[i][j].next_dir; // 机器人方向
//                     Robots[i].berth_index = roads[i][j].berth_index; //物品最近港口
//                     break;
//                 }
//             }
//             if (j == roads[i].size())
//             {                                             // 没东西拿
//                 AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
//                 Robots[i].dir = -1;                       // 罚站
//             }
//         }
//     }
// }

void RobotDsipatchGreedy()
{
    // 维护一个Road优先队列，每次找一条最短的路径匹配
    priority_queue<Road, std::vector<Road>, Road::Comparator> roads_pq;
    vector<int> robots_match(ROBOT_NUM, 0); // 机器人是否匹配
    set<int> goods_match;                   // 被匹配的商品
    int robot_match_num = 0;                // 被匹配的机器人数

    pair<int, int> berth_robot_num[BERTH_NUM]; //每个港口作为目标匹配的机器人数
    // 初始化，所有机器人数目设置为0
    for (int i = 0; i < BERTH_NUM; i++) {
        berth_robot_num[i] = {i, 0}; 
    }

    for (int ri = 0; ri < ROBOT_NUM; ri++)
    { // 对于每个机器人
        if (Robots[ri].is_dead == 1 || Robots[ri].status == 0)
        { // 如果机器人不能动（被困或者处于恢复状态），就不考虑
            robot_match_num++;
            robots_match[ri] = 1;
            Robots[ri].dir = -1;
            continue;
        }
        if (Robots[ri].is_goods == 1)
        { // 如果机器人拿着物品就直接找港口
            if (IsInBerth(ri))
            { // 如果到达港口，放下货物，继续找其他货物（?是否需要判断是不是自己要去的港口呢）
                Robots[ri].action = 1;
                Robots[ri].goods_index = -1;
                Robots[ri].goods_distance = 40000;
                //Robots[ri].berth_index = -1;
                Robots[ri].is_goods = 0;
            }
            else
            { // 没到港口，沿着图走
                robot_match_num++;
                robots_match[ri] = 1;
                Robots[ri].dir = BerthPath[Robots[ri].berth_index][Robots[ri].x][Robots[ri].y];
                continue;
            }
        }

        // 以上情况之外，其他机器人都要BFS找货物
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
            if (goods_id >= 0)
            { // 如果现在的位置有物品
                if (Frame - AllGoods[goods_id].refresh_frame >= 1000)
                { // 该物品已消失
                    World[cur_pos.first][cur_pos.second] = 0;
                }
                else if (cur_path_length < (1000 - (Frame - AllGoods[goods_id].refresh_frame)))
                { // 机器人到物品处时商品未消失, 计算最近港口和性价比，并将路径加入
                    int to_berth_index = -1;
                    double value = CalculateGoodsValue(goods_id, cur_path_length, to_berth_index);
                    roads_pq.push(Road(ri, goods_id, cur_path_length, to_berth_index, goods_path[cur_pos.first][cur_pos.second], value));
                    ri_road_num ++;
                }
            }
            // 限制搜索的范围以控制时间
            if (ri_road_num > MAX_ROAD_NUM || cur_path_length > MAX_ROAD_LEN)
            {
                break;
            }
            // 四个方向随机遍历
            vector<int> random_dir = GetRandomDirection();
            for (int i = 0; i < 4; i++)
            { // 遍历到下一个点
                int dir = random_dir[i];
                int nx = cur_pos.first + DX[dir];
                int ny = cur_pos.second + DY[dir];
                if (IsValid(nx, ny) && goods_path_length[nx][ny] < 0)
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
    // 开始机器人和物品的匹配
    while (!roads_pq.empty())
    { // 价值最大的路径
        if (robot_match_num >= ROBOT_NUM)
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
                Robots[ri].dir = BerthPath[Robots[ri].berth_index][Robots[ri].x][Robots[ri].y];
                World[Robots[ri].x][Robots[ri].y] = 0;
            }
            robot_match_num++;
            robots_match[ri] = 1;
            goods_match.insert(road.goods_index);
        }
        roads_pq.pop();
    }

    //若有人没匹配上
    for (int i = 0; i < ROBOT_NUM; i++){
        //先统计每个港口作为目标匹配的机器人数
        if (Robots[i].berth_index != -1){ 
            berth_robot_num[Robots[i].berth_index].second++;
        }
    }
    //按港口机器人数排序
    sort(berth_robot_num, berth_robot_num + BERTH_NUM, 
        [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
            return a.second < b.second;
        });
    for (int i = 0; i < ROBOT_NUM; i++){
        // 没匹配上的向人最少的港口移动
        if (robots_match[i] == 0){ //没匹配上
            int j = 0;
            for (j = 0; j < BERTH_NUM; j++){
                if (BerthPathLenth[berth_robot_num[j].first][Robots[i].x][Robots[i].y] >= 0){
                    Robots[i].dir = BerthPath[berth_robot_num[j].first][Robots[i].x][Robots[i].y]; //向人最少的港口移动一步
                    break;
                }
            }
            if (j == BERTH_NUM){
                Robots[i].dir = -1; //到不了港口罚站
            }
        }
    }

}

// 对冲让位函数
bool CrashAvoid(int ri)
{
    // ri优先让两边
    int new_dir = Robots[ri].dir; // 新方向

    srand(time(nullptr)); // 使用当前时间作为随机数生成器的种子

    if (new_dir == 0 || new_dir == 1)
    {
        // 当前方向是上或下，所以选择左或右
        int nx1 = Robots[ri].x + DX[2]; // 向左的坐标
        int ny1 = Robots[ri].y + DY[2];
        int nx2 = Robots[ri].x + DX[3]; // 向右的坐标
        int ny2 = Robots[ri].y + DY[3];
        int nx3 = Robots[ri].x + DX[(new_dir + 1) % 2]; // 向后的坐标
        int ny3 = Robots[ri].y + DY[(new_dir + 1) % 2];
        if (World[nx1][ny1] >= 0 && World[nx2][ny2] >= 0)
        {                             // 两个方向都可行,随机选一个
            new_dir = 2 + rand() % 2; // 生成 2 或 3
        }
        else if (World[nx1][ny1] >= 0)
        {
            new_dir = 2; // 左
        }
        else if (World[nx2][ny2] >= 0)
        {
            new_dir = 3; // 右
        }
        else if (World[nx3][ny3] >= 0)
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
        if (World[nx1][ny1] >= 0 && World[nx2][ny2] >= 0)
        {                         // 两个方向都可行,随机选一个
            new_dir = rand() % 2; // 生成 0 或 1
        }
        else if (World[nx1][ny1] >= 0)
        {
            new_dir = 0; // 上
        }
        else if (World[nx2][ny2] >= 0)
        {
            new_dir = 1; // 下
        }
        else if (World[nx3][ny3] >= 0)
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

// 碰撞检测与规避
void AvoidCollision()
{
    bool is_collision = true; // 本次是否有碰撞
    int detect_num = 0;       // 检测的次数
    while (is_collision)
    {
        detect_num++;
        is_collision = false;
        // 对每个机器人进行碰撞检测
        for (int ri = 0; ri < ROBOT_NUM; ri++)
        {
            // 如果这个机器人没有被困死并且下一步会移动（对于每个机器人只考虑主动碰撞，如果是被碰，会被其他机器人主动碰）
            if (!Robots[ri].is_dead && Robots[ri].dir >= 0)
            {
                // 机器人i下一步的位置
                int nx_ri = Robots[ri].x + DX[Robots[ri].dir];
                int ny_ri = Robots[ri].y + DY[Robots[ri].dir];
                // 检测除自己之外其他机器人会不会在这个位置上
                // （不移动的机器人看当前位置，会移动的机器人分析是对冲还是抢位置）
                for (int rj = 0; rj < ROBOT_NUM; rj++)
                {
                    if (ri != rj)
                    {
                        // 碰上不移动的机器人j
                        if (Robots[rj].dir < 0 && Robots[rj].x == nx_ri && Robots[rj].y == ny_ri)
                        {
                            is_collision = true;
                            //？若不动的处于恢复状态，我垂直移动，不行我就罚站
                            if (Robots[rj].status == 0){
                                if (CrashAvoid(ri)){
                                    Robots[ri].dir = -1;
                                };
                            }
                            //若不动的只是在罚站，他垂直移动
                            else{
                                CrashAvoid(rj);
                            }
                            break;
                        }
                        // 碰上移动的机器人j
                        if (Robots[rj].dir >= 0)
                        {
                            int nx_rj = Robots[rj].x + DX[Robots[rj].dir];
                            int ny_rj = Robots[rj].y + DY[Robots[rj].dir];
                            // 对冲
                            if (Robots[rj].x == nx_ri && Robots[rj].y == ny_ri && (Robots[ri].dir / 2 == Robots[rj].dir / 2))
                            {
                                is_collision = true;
                                // 性价比低的机器人避让（优先让两边，实在不行往后面退，再不行就让性价比高的让）
                                if (Robots[ri].is_goods && Robots[rj].is_goods)
                                { // 都拿物品
                                    if (GetGoodsRobotsCompair(ri, rj))
                                    { // 我拿的好
                                        if (!CrashAvoid(rj))
                                        {
                                            CrashAvoid(ri);
                                        }
                                    }
                                    else
                                    { // 别人拿的好
                                        if (!CrashAvoid(ri))
                                        {
                                            CrashAvoid(rj);
                                        }
                                    }
                                }
                                else if (!Robots[ri].is_goods && !Robots[rj].is_goods)
                                { // 都没拿物品
                                    if (NoGoodsRobotsCompair(ri, rj))
                                    { // 我拿的好
                                        if (!CrashAvoid(rj))
                                        {
                                            CrashAvoid(ri);
                                        }
                                    }
                                    else
                                    { // 别人拿的好
                                        if (!CrashAvoid(ri))
                                        {
                                            CrashAvoid(rj);
                                        }
                                    }
                                }
                                else if (!Robots[ri].is_goods && Robots[rj].is_goods)
                                { // 我没拿别人拿了，我让，ri优先让两边
                                    if (!CrashAvoid(ri))
                                    {
                                        CrashAvoid(rj);
                                    }
                                }
                                else
                                { // 别人没拿我拿了，别人让
                                    if (!CrashAvoid(rj))
                                    {
                                        CrashAvoid(ri);
                                    }
                                }
                                break;
                            }
                            // 抢位
                            if (nx_ri == nx_rj && ny_ri == ny_rj)
                            {
                                is_collision = true;
                                // 有货物的优先走，另一个停下
                                if (Robots[ri].is_goods && !Robots[rj].is_goods)
                                {
                                    Robots[rj].dir = -1;
                                }
                                else if (Robots[rj].is_goods && !Robots[ri].is_goods)
                                {
                                    Robots[ri].dir = -1;
                                }
                                // 都有货物或者都没有货物则比较性价比
                                else if (Robots[ri].is_goods && Robots[rj].is_goods)
                                {
                                    if (GetGoodsRobotsCompair(ri, rj))
                                    { // 我拿的好，别人停
                                        Robots[rj].dir = -1;
                                    }
                                    else
                                    { // 别人拿的好，我停
                                        Robots[ri].dir = -1;
                                    }
                                }
                                else
                                {
                                    if (NoGoodsRobotsCompair(ri, rj))
                                    { // 我要拿的好，别人停
                                        Robots[rj].dir = -1;
                                    }
                                    else
                                    { // 别人要拿的好，我停
                                        Robots[ri].dir = -1;
                                    }
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
    for (int i = 0; i < ROBOT_NUM; i++)
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

int main()
{
    Init();
    for (int frame = 1; frame <= 15000; frame++)
    {
        Input();
        RobotDsipatchGreedy();
        AvoidCollision();
        PrintRobotsIns();
        BoatDispatch();
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
