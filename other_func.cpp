#include "head/function.h"

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
        return BerthPath[berth_id][x][y][0];
    }
    return -1;
}

// 计算物品性价比，并确定最接近的港口
double CalculateGoodsValue(int ri, int goods_index, int step_num, int &to_berth_index, int consider_rest_time)
{
    int goods_value;                   // 物品价值
    int goods_x, goods_y;              // 物品坐标
    double goods_rest_time_weight = 1; // 物品剩余时间
    int to_berth_len = MAX_LENGTH;     // 物品到港口距离
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

    // 根据机器人类型计算
    if(Robots[ri].type == 1 && Robots[ri].is_goods == 1)
    { // 已经装了一个货的金牌机器人，加上自己的货
        goods_value += AllGoods[Robots[ri].goods_stack.top()].val;
    }

    double cost_value = (double)goods_value * goods_rest_time_weight / (step_num * TO_GOODS_WEIGHT + to_berth_len);
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

// 计算机器人购买点占据地图大小
int CalculateArea(int buy_index)
{
    int visted[N][N]; // 判断是否bfs过
    memset(visted, -1, sizeof(visted));
    int area = 0;
    int buy_x = RobotBuyings[buy_index].x;
    int buy_y = RobotBuyings[buy_index].y;
    queue<pair<int, int>> q;
    // 将本位置加入
    q.push({buy_x, buy_y});
    area++;
    visted[buy_x][buy_y] = 0;
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
            { // 判断陆上是否可以到达
                int next_path_length = visted[cur_pos.first][cur_pos.second] + 1;
                if (visted[nx][ny] < 0)
                { // 这个点之前没有遍历过
                    visted[nx][ny] = next_path_length;
                    // 将该点加入队列
                    q.push({nx, ny});
                    area++;
                }
            }
        }
    }
    return area;
}

// 根据海域连通性确定船只最少购买数
void linkMaxBuyBoat()
{
    int buy_index = 0;
    for (buy_index = 0; buy_index < BoatBuyingNum; buy_index++)
    {
        if (buy_index == 0)
        {
            // 0号购买点直接算一个海域
            AllocateBoatNum[buy_index] = buy_index;
            LinkMaxBoatBuying++;
        }
        else
        {
            // 对其他的要看之前的购买点是否连通
            bool is_link = false; // 是否连通
            for (int pre_buy_index = 0; pre_buy_index < buy_index; pre_buy_index++)
            {
                // 之前的港口
                int bei = 0;
                for (bei = 0; bei < BerthNum; bei++)
                {
                    // 根据跟港口的连通性判断
                    if (BuyingToBerthTime[pre_buy_index][bei] > 0)
                    {
                        // 之前连通
                        if (BuyingToBerthTime[buy_index][bei] > 0)
                        {
                            // 我也连通
                            is_link = true;
                            break;
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
                if (is_link)
                {
                    // 有连通
                    AllocateBoatNum[buy_index] = pre_buy_index;
                    break;
                }
            }
            if (!is_link)
            {
                // 无连通,新的海域
                AllocateRobotNum[buy_index] = buy_index;
                LinkMaxBoatBuying++;
            }
        }
    }
    MAX_BUY_BOAT_NUM = max(MAX_BUY_BOAT_NUM, LinkMaxBoatBuying);
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

    return h_value;
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

// 返回一个位置到另一个位置最短时间，-1表示不可达
int PositionToPositionAStar(int sx, int sy, int sdir, int d_type, int d_id)
{
    int search_node_num = 0;
    // 如果状态不合法则返回-1
    if (!JudgeBoatState(sx, sy, sdir))
    {
        AStarSearchNodeNum.push_back(search_node_num);
        return -1;
    }
    int dx = d_type ? Berths[d_id].x : Deliveries[d_id].x;
    int dy = d_type ? Berths[d_id].y : Deliveries[d_id].y;
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
            to_berth_time = 2 * (abs(cur->x - dx) + abs(cur->y - dy));
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

// 是哪张地图
int WhichMap()
{
    if (DeliveryToBerthTime[0][0][0] == -1 && DeliveryToBerthTime[0][0][1] == 487 && DeliveryToBerthTime[0][0][1] == 485)
        return 1;
    else if (DeliveryToBerthTime[0][0][0] == 308 && DeliveryToBerthTime[0][0][1] == 307 && DeliveryToBerthTime[0][0][1] == 312)
        return 2;
    else
        return 3;
}
