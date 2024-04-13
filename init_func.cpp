#include "head/function.h"

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
    MAX_BUY_BOAT_NUM = MAX_BUY_BOAT_NUM_ARRAY[BerthNum];
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

// 记录从交货点到泊位的最短时间
void InitDeliveryToBerth()
{
    // 初始化从交货点到泊位的最短时间 -1为不可达
    memset(DeliveryToBerthTime, -1, sizeof(DeliveryToBerthTime));
    for (int di = 0; di < DeliveryNum; di++)
    { // 对每一个交货点
        for (int bi = 0; bi < BerthNum; bi++)
        { // 对每一个泊位
            for (int dir = 0; dir < 4; dir++)
            { // 对每个方向
                if (ToBerthEstimateTime[bi][Deliveries[di].x][Deliveries[di].y] == -1)
                { // 如果单点都不可达
                    DeliveryToBerthTime[di][bi][dir] = -1;
                }
                else
                { // 单点可达，A*寻路
                    DeliveryToBerthTime[di][bi][dir] = PositionToPositionAStar(Deliveries[di].x, Deliveries[di].y, dir, 1, bi);
                }
            }
        }
    }
}

// 记录从泊位到交货点的最短时间
void InitBerthToDelivery()
{
    // 初始化从泊位到交货点的最短时间 -1为不可达
    memset(BerthToDeliveryTime, -1, sizeof(BerthToDeliveryTime));
    // 初始化从泊位到交货点的每个方向最短时间的最大时间
    memset(BerthToDeliveryTimeMax, -1, sizeof(BerthToDeliveryTimeMax));
    for (int bi = 0; bi < BerthNum; bi++)
    { // 对每一个泊位
        for (int di = 0; di < DeliveryNum; di++)
        { // 对每一个交货点
            for (int dir = 0; dir < 4; dir++)
            { // 对每个方向
                if (ToBerthEstimateTime[bi][Deliveries[di].x][Deliveries[di].y] == -1)
                { // 如果单点都不可达
                    BerthToDeliveryTime[bi][di][dir] = -1;
                }
                else
                { // 单点可达，A*寻路
                    BerthToDeliveryTime[bi][di][dir] = PositionToPositionAStar(Berths[bi].x, Berths[bi].y, dir, 0, di);
                    if (BerthToDeliveryTime[bi][di][dir] > BerthToDeliveryTimeMax[bi][di])
                    { // 记录从该泊位到交货点最短时间的最大值
                        BerthToDeliveryTimeMax[bi][di] = BerthToDeliveryTime[bi][di][dir];
                    }
                }
            }
        }
    }
}

// 记录每个泊位对应的最近交货点
void InitBerthNearestDelivery()
{
    // 初始化每个泊位的最近交货点
    memset(BerthNearestDelivery, -1, sizeof(BerthNearestDelivery));
    for (int bi = 0; bi < BerthNum; bi++)
    { // 对每一个泊位
        for (int di = 0; di < DeliveryNum; di++)
        { // 对每一个交货点
            if (BerthNearestDelivery[bi] == -1 || BerthToDeliveryTimeMax[bi][di] < BerthToDeliveryTimeMax[bi][BerthNearestDelivery[bi]])
            {
                BerthNearestDelivery[bi] = di;
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
        { // 对每一个泊位
            if (ToBerthEstimateTime[bi][BoatBuyings[bbi].x][BoatBuyings[bbi].y] == -1)
            { // 如果单点都不可达
                BuyingToBerthTime[bbi][bi] = -1;
                BerthNearestBuying[bi] = -1;
            }
            else
            { // 方向只考虑向右正方向
                BuyingToBerthTime[bbi][bi] = PositionToPositionAStar(BoatBuyings[bbi].x, BoatBuyings[bbi].y, 0, 1, bi);
                // 如果之前没有找到或者找到的时间更短
                if (BerthNearestBuying[bi] == -1 || BuyingToBerthTime[bbi][bi] < BuyingToBerthTime[BerthNearestBuying[bi]][bi])
                {
                    BerthNearestBuying[bi] = bbi;
                }
            }
        }
    }
}

void InitBerthToBerth()
{
    // 初始化泊位到泊位之间的最短时间 -1为不可达
    memset(BerthToBerthTime, -1, sizeof(BerthToBerthTime));
    for (int bi = 0; bi < BerthNum; bi++)
    { // 对每一个港口
        for (int bj = 0; bj < BerthNum; bj++)
        { // 对另一个港口作为目的地
            if (bi != bj)
            {
                for (int dir = 0; dir < 4; dir++)
                { // 对每一个方向
                    if (ToBerthEstimateTime[bj][Berths[bi].x][Berths[bi].y] == -1)
                    { // 如果单点都不可达
                        BerthToBerthTime[bi][bj][dir] = -1;
                    }
                    else
                    {
                        BerthToBerthTime[bi][bj][dir] = PositionToPositionAStar(Berths[bi].x, Berths[bi].y, dir, 1, bj);
                    }
                }
            }
        }
    }
}

// 初始化港口最近的购买点和交货点
void InitRobotAndBoatBuy()
{
    memset(InitRobotType0ToBuy, 0, sizeof(InitRobotType0ToBuy));
    memset(InitRobotType1ToBuy, 0, sizeof(InitRobotType1ToBuy));
    memset(InitBuyingToBuy, -1, sizeof(InitBuyingToBuy));
    memset(InitBerthToGo, -1, sizeof(InitBerthToGo));

    // 机器人
    int robot_buy_num_average = InitBuyType0RobotNum / (int)RobotBuyingNum;
    int robot_buy_num_remain = InitBuyType0RobotNum % (int)RobotBuyingNum;
    for (int rbi = 0; rbi < RobotBuyingNum; rbi++)
    {
        InitRobotType0ToBuy[rbi] = robot_buy_num_average;
        if (robot_buy_num_remain > 0)
        {
            InitRobotType0ToBuy[rbi]++;
            robot_buy_num_remain--;
        }
    }
    int gold_robot_num = InitBuyType1RobotNum;
    for (int rbi = 0; rbi < RobotBuyingNum; rbi++)
    {
        if (gold_robot_num == 0)
            break;
        else
        {
            InitRobotType1ToBuy[rbi]++;
            gold_robot_num--;
        }
    }

    struct BestBuy
    {
        int best_buy_cost = MAX_LENGTH;
        int best_buy_buying;
        int best_buy_berth;

        // 重载 < 运算符，根据 best_buy_cost 比较
        bool operator<(const BestBuy &other) const
        { // 小的cost优先级高
            return this->best_buy_cost > other.best_buy_cost;
        }
        bool operator==(const BestBuy &other) const
        {
            return this->best_buy_buying == other.best_buy_buying ||
                   this->best_buy_berth == other.best_buy_berth;
        }
        BestBuy(int cost, int buying, int berth)
        {
            this->best_buy_cost = cost;
            this->best_buy_buying = buying;
            this->best_buy_berth = berth;
        }
    };

    // 船
    int init_buy_boat_num = 0;
    priority_queue<BestBuy> best_buy_queue;

    for (int bb = 0; bb < BoatBuyingNum; bb++)
    {
        for (int be = 0; be < BerthNum; be++)
        {
            if (BuyingToBerthTime[bb][be] != -1)
            {
                best_buy_queue.emplace(BuyingToBerthTime[bb][be], bb, be);
            }
        }
    }

    set<BestBuy> best_buy_set;
    while (init_buy_boat_num < InitBuyBoatNum && !best_buy_queue.empty())
    {
        BestBuy best_buy = best_buy_queue.top();
        best_buy_queue.pop();
        if (best_buy_set.find(best_buy) != best_buy_set.end())
        {
            continue;
        }
        InitBuyingToBuy[init_buy_boat_num] = best_buy.best_buy_buying;
        InitBerthToGo[init_buy_boat_num] = best_buy.best_buy_berth;
        init_buy_boat_num++;
    }
}

// 初始化函数
void Init()
{
    InitWorldInfo();
    InitToBerthBFS();
    InitToBerthEstimateTimeDijkstra();
    InitToDeliveryEstimateTimeDijkstra();
    InitDeliveryToBerth();
    InitBerthToDelivery();
    InitBerthToBerth();
    InitBerthNearestDelivery();
    InitBuyingToBerth();
    Map = WhichMap();
    InitRobotAndBoatBuy();
    // 输出OK
    printf("OK\n");
    fflush(stdout);
}
