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
#endif

using namespace std;

const int ROBOT_NUM = 10;
const int BERTH_NUM = 10;
const int BOAT_NUM = 5;
const int N = 200;
const int MAX_GOODS_NUM = 150000;

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
    int lock;          // 货物是否被机器人锁定(0:未锁定)

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
    int x, y;        // 机器人的坐标
    int is_goods;    // 机器是否携带货物 (0：没有，1：有)
    int status;      // 机器人的状态（0：恢复，1：运行）
    int dir;         // 机器人下一步运动的方向
    int goods_index; // 机器人携带的货物编号
    int berth_index; // 机器人要去的泊位编号
    int is_dead;     // 机器人是否被困死（0：没有，1：有）
    int action;      // 0:get()拿货物， 1:pull()放物品

    Robot() {}
    Robot(int x, int y)
    {
        this->x = x;
        this->y = y;
        this->is_goods = 0;
        this->status = 1;
        this->dir = -1;
        this->goods_index = -1;
        this->berth_index = -1;
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
        int value_time = 0;
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
                int value_time_tmp = Berths[i].sum_goods_value() / load_time;
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

// 判断是否在物品上
bool IsInGoods(int robot_index)
{
    if (World[Robots[robot_index].x][Robots[robot_index].y] >= 100)
    {
        return true;
    }
    return false;
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
        if (BerthPathLenth[i][goods_x][goods_y] <= to_berth_len)
        {
            to_berth_len = BerthPathLenth[i][goods_x][goods_y];
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
            {
                // 遍历到下一个点
                int dir = random_dir[i];
                int nx = cur_pos.first + DX[dir];
                int ny = cur_pos.second + DY[dir];
                // 判断该点是否可以到达(没有越界&&为空地或者泊位&&之前没有到达过)
                if (IsValid(nx, ny) && BerthPathLenth[b][nx][ny] < 0)
                {
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

    // TODO: 初始化船的信息

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

struct Road // 物品id、最近港口、机器人下一步方向
{
    int goods_index;
    int berth_index;
    int next_dir;

    Road() {}
    Road(int goods_index, int berth_index, int next_dir)
    {
        this->goods_index = goods_index;
        this->berth_index = berth_index;
        this->next_dir = next_dir;
    }
};

// 使用BFS算法找到最好的3个物品,返回物品index、最近港口和下一步方向
vector<Road> ToGoodsBFS(int robot_index, int total_step)
{
    int robot_x, robot_y; // 机器人的坐标
    robot_x = Robots[robot_index].x;
    robot_y = Robots[robot_index].y;

    int visited[N][N] = {0}; // 访问标记数组
    queue<pair<int, int>> q; // BFS队列,坐标值
    queue<int> direction;    // 用于记录到达每个位置的移动方向
    vector<Road> best_goods; // 保存返回的物品

    q.push({robot_x, robot_y});    // 将机器人初始位置入队
    visited[robot_x][robot_y] = 1; // 标记为已访问
    direction.push(-1);            // -1表示初始位置，没有实际方向意义

    int step = 0; // 当前步数
    while (!q.empty() && step < total_step)
    {
        int min_val = 201; // 当前最小价值
        int mid_val;       // 第二小价值
        int max_val = 0;   // 当前最大价值

        int size = q.size();
        step++;
        for (int i = 0; i < size; ++i)
        {
            auto [x, y] = q.front();
            q.pop();
            int dir = direction.front();
            direction.pop();

            if (World[x][y] - 100 >= 0 && step <= AllGoods[World[x][y] - 100].refresh_frame)
            { // 找到物品
                int berth_index;
                double current_val = CalculateGoodsValue(World[x][y] - 100, step, berth_index); // 计算物品性价比
                if (best_goods.size() < 3)
                { // 已找到小于三个
                    if (min_val > current_val)
                    { // 最小价值总是插在末尾
                        if (min_val != 201)
                        { // 不是第一个插入的
                            mid_val = min_val;
                        }
                        min_val = current_val;
                        best_goods.push_back(Road(World[x][y] - 100, berth_index, dir));
                    }
                    else
                    {
                        if (max_val < current_val)
                        { // 最大的插在开头
                            max_val = current_val;
                            best_goods.insert(best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
                        }
                        else
                        {
                            mid_val = max_val;
                            best_goods.insert(++best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
                        }
                    }
                }
                else
                { // 保留最大的3个
                    if (min_val <= current_val)
                    { // 最小价值总是插在末尾
                        if (max_val < current_val)
                        { // 当前最大
                            min_val = mid_val;
                            mid_val = max_val;
                            max_val = current_val;
                            best_goods.insert(best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
                            best_goods.pop_back(); // 弹出最小的
                        }
                        else if (mid_val <= current_val)
                        { // 当前为第二小
                            min_val = mid_val;
                            mid_val = current_val;
                            best_goods.pop_back(); // 弹出最小的
                            best_goods.insert(++best_goods.begin(), Road(World[x][y] - 100, berth_index, dir));
                        }
                        else
                        { // 当前最小
                            min_val = current_val;
                            best_goods.pop_back();
                            best_goods.push_back(Road(World[x][y] - 100, berth_index, dir));
                        }
                    }
                }
            }

            for (int d = 0; d < 4; ++d)
            { // 检查四个方向
                int nx = x + DX[d];
                int ny = y + DY[d];

                if (IsValid(nx, ny) && !visited[nx][ny])
                {                        // 检查下一步是否可走
                    visited[nx][ny] = 1; // 标记已走过
                    q.push({nx, ny});    // 存储下一步位置
                    direction.push(d);   // 存储对应的方向
                }
            }
        }
    }
    return best_goods;
}

// 每个机器人本帧动作
void RobotDispatch()
{
    vector<vector<Road>> roads; // 每个机器人各自最好的几个商品和对应的下一步
    for (int i = 0; i < ROBOT_NUM; i++)
    {
        vector<Road> current_road; // 当前机器人BFS结果
        if (Robots[i].is_dead == 1 or Robots[i].status == 0)
        { // 若机器人不能到港口就不考虑
            roads.push_back(current_road);
            Robots[i].dir = -1;
            continue;
        }

        if (Robots[i].is_goods == 1)
        { // 若拿着物品
            roads.push_back(current_road);
            if (IsInBerth(i))
            { // 到达港口
                Robots[i].action = 1;
                Robots[i].dir = -1;
            }
            else
            { // 没到港口，沿着图走
                Robots[i].dir = BerthPath[Robots[i].berth_index][Robots[i].x][Robots[i].y];
            }
            continue;
        }

        current_road = ToGoodsBFS(i, 80);
        roads.push_back(current_road); // 最多找80步
    }

    for (int i = 0; i < ROBOT_NUM; i++)
    {
        if (roads[i].empty())
        { // 若没有bfs到物品或机器人不能到港口或已经拿了物品
            if (Robots[i].is_dead != 1 && Robots[i].is_goods != 1)
            { // 没拿物品也没死
                Robots[i].dir = -1;
            }
            continue;
        }

        if (IsInGoods(i))
        {                                             // 在物品上
            AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
            int j = 0;
            for (j = 0; j < roads[i].size(); j++)
            {
                int goods_id = roads[i][j].goods_index;
                if (AllGoods[goods_id].lock == 0 && AllGoods[goods_id].x == Robots[i].x && AllGoods[goods_id].y == Robots[i].y)
                { // 确认过眼神，是我要拿的人
                    // 拿物品
                    AllGoods[goods_id].lock = 1;                                                // 物品上锁
                    Robots[i].goods_index = goods_id;                                           // 机器人拿的物品编号
                    Robots[i].berth_index = roads[i][j].berth_index;                            // 要去的港口编号
                    Robots[i].action = 0;                                                       // 拿物品的动作
                    Robots[i].dir = BerthPath[Robots[i].berth_index][Robots[i].x][Robots[i].x]; // 按图找方向
                    World[Robots[i].x][Robots[i].x] = 0;                                        // 地图上去除物品
                    break;
                }
            }
            if (j == roads[i].size())
            { // 不是我要拿的
                // 按road走
                int k = 0;
                for (k = 0; k < roads[i].size(); k++)
                {
                    int goods_id = roads[i][k].goods_index;
                    if (AllGoods[goods_id].lock == 0)
                    {
                        if (Robots[i].goods_index != goods_id)
                        {
                            AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
                        }
                        Robots[i].goods_index = goods_id;     // 机器人要拿的物品编号
                        AllGoods[goods_id].lock = 1;          // 物品上锁
                        Robots[i].dir = roads[i][k].next_dir; // 机器人方向
                        break;
                    }
                }
                if (k == roads[i].size())
                {                                             // 没东西拿
                    AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
                    Robots[i].dir = -1;                       // 罚站
                }
            }
        }
        else
        { // 按road走
            int j = 0;
            for (j = 0; j < roads[i].size(); j++)
            {
                int goods_id = roads[i][j].goods_index;
                if (AllGoods[goods_id].lock == 0)
                {
                    if (Robots[i].goods_index != goods_id)
                    {
                        AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
                    }
                    Robots[i].goods_index = goods_id;     // 机器人要拿的物品编号
                    AllGoods[goods_id].lock = 1;          // 物品上锁
                    Robots[i].dir = roads[i][j].next_dir; // 机器人方向
                    break;
                }
            }
            if (j == roads[i].size())
            {                                             // 没东西拿
                AllGoods[Robots[i].goods_index].lock = 0; // 物品解锁
                Robots[i].dir = -1;                       // 罚站
            }
        }
    }
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
                        // ？碰上不移动的机器人j，则机器人i自己也不动（停一帧）
                        if (Robots[rj].dir < 0 && Robots[rj].x == nx_ri && Robots[rj].y == ny_ri)
                        {
                            is_collision = true;
                            Robots[ri].dir = -1;
                            break;
                        }
                        // 碰上移动的机器人j
                        if (Robots[rj].dir > 0)
                        {
                            int nx_rj = Robots[rj].x + DX[Robots[rj].dir];
                            int ny_rj = Robots[rj].y + DY[Robots[rj].dir];
                            // 对冲
                            if (Robots[rj].x == nx_ri && Robots[rj].y == ny_ri && (Robots[rj].dir / 2 == Robots[rj].dir / 2))
                            {
                                is_collision = true;
                                // 性价比低的机器人避让（优先让两边，实在不行往后面退，再不行就让性价比高的让）
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
                                    //
                                }
                                else
                                {
                                    //
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
    // 
}

int main()
{
    Init();
    for (int frame = 1; frame <= 15000; frame++)
    {
        Input();
        // RobotDispatch();
        for (int i = 0; i < ROBOT_NUM; i++)
            printf("move %d %d\n", i, rand() % 4);
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
