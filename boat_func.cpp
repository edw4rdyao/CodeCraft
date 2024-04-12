#include "head/function.h"

// 在交货点找一个最好的港口出发(但是要确保去了可以回来)
int FindBestBerthFromDelivery(int boat_id, int delivery_id)
{
    int best_berth = -1;
    double max_value = -1;
    for (int bi = 0; bi < BerthNum; bi++)
    { // 对每个没有船要去的港口，并且该港口可达
        if (Berths[bi].boat_id == -1 && DeliveryToBerthTime[delivery_id][boat_id][Boats[boat_id].dir] != -1)
        {
            // 计算性价比（过去装完直接去交货点）
            double goods_value = 0;
            int transport_time = DeliveryToBerthTime[delivery_id][bi][Boats[boat_id].dir] + BerthToDeliveryTimeMax[bi][BerthNearestDelivery[bi]];
            queue<int> tmp = Berths[bi].goods_queue;
            int to_load_goods = min((int)tmp.size(), BoatCapacity);
            for (int j = 0; j < to_load_goods; j++)
            {
                goods_value += AllGoods[tmp.front()].val;
                tmp.pop();
            }
            transport_time += (to_load_goods / Berths[bi].velocity);
            if (MAX_FRAME - Frame < transport_time)
                continue;
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

// 在港口选择其他港口或者返回交货点 -2表示回交货点最优
int FindBestBerthOrGoFromBerth(int boat_id)
{
    int best_berth = -1;
    double max_value = -1;
    for (int bi = 0; bi < BerthNum; bi++)
    { // 对每个没有船要去的其他港口，并且该港口可达
        if (bi != Boats[boat_id].dest_berth && Berths[bi].boat_id == -1 && BerthToBerthTime[Boats[boat_id].dest_berth][bi][Boats[boat_id].dir] != -1)
        {
            // 计算性价比
            double goods_value = Boats[boat_id].goods_value;
            int transport_time = BerthToBerthTime[Boats[boat_id].dest_berth][bi][Boats[boat_id].dir] + BerthToDeliveryTimeMax[bi][BerthNearestDelivery[bi]];
            queue<int> tmp = Berths[bi].goods_queue;
            int to_load_goods = min((int)tmp.size(), BoatCapacity - Boats[boat_id].goods_num); // 还能装的货（囤的货和还能装的货取最小值）
            for (int j = 0; j < to_load_goods; j++)
            {
                goods_value += AllGoods[tmp.front()].val;
                tmp.pop();
            }
            transport_time += (to_load_goods / Berths[bi].velocity);
            if (MAX_FRAME - Frame < transport_time)
                continue;
            goods_value /= transport_time;
            if (goods_value > max_value)
            {
                best_berth = bi;
                max_value = goods_value;
            }
        }
    }
    // 再对比一下直接回交货点的性价比
    if ((double)Boats[boat_id].goods_value / BerthToDeliveryTime[Boats[boat_id].dest_berth][BerthNearestDelivery[Boats[boat_id].dest_berth]][Boats[boat_id].dir] > max_value)
    {
        return -2;
    }
    return best_berth;
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
            int transport_time = BuyingToBerthTime[boat_buying_id][bi] + BerthToDeliveryTimeMax[bi][BerthNearestDelivery[bi]];
            queue<int> tmp = Berths[bi].goods_queue;
            int to_load_goods = min((int)tmp.size(), BoatCapacity);
            for (int j = 0; j < to_load_goods; j++)
            {
                goods_value += AllGoods[tmp.front()].val;
                tmp.pop();
            }
            transport_time += (to_load_goods / Berths[bi].velocity);
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

int BoatToPositionAStar(int bi, int d_type, int d_id)
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
        { // 目的地是交货点，核心点在交货点
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
            return BoatRoutes[bi].size();
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
    return MAX_LENGTH;
}

// 船滚去交货点
void RunToDeliveryGun(int bi)
{ // 船
    // 根据当前状态找一个近的交货点
    int best_delivery = -1;
    int min_to_delivery_time = -1;
    for (int di = 0; di < DeliveryNum; di++)
    {
        if (BerthToDeliveryTime[Boats[bi].dest_berth][di][Boats[bi].dir] == -1)
            continue;
        if (min_to_delivery_time == -1 || BerthToDeliveryTime[Boats[bi].dest_berth][di][Boats[bi].dir] < min_to_delivery_time)
        { // 该交货点可达
            min_to_delivery_time = BerthToDeliveryTime[Boats[bi].dest_berth][di][Boats[bi].dir];
            best_delivery = di;
        }
    }
    Berths[Boats[bi].dest_berth].focus = 0;    // 清除港口聚焦
    Berths[Boats[bi].dest_berth].boat_id = -1; // 清除港口的船
    Boats[bi].dest_delivery = best_delivery;   // 加入交货点目的地
    Boats[bi].dest_berth = -1;                 // 清除港口目的地
    Boats[bi].status = 0;                      // 清空状态避免后面装货
    // 清空路径并且寻路（实际上靠泊的时候路径就一定清空了）
    BoatRoutes[bi].clear();
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
            // 找最好的目的地
            // int to_dilivery[DeliveryNum];
            // int min_len = MAX_LENGTH;
            // int best_dilivery = -1;
            // 对所有交货点
            // for (int di = 0; di < DeliveryNum; di++)
            // {
            //     to_dilivery[di] = BoatToPositionAStar(bi, 0, di);
            //     if (min_len >= to_dilivery[di])
            //     {
            //         min_len = to_dilivery[di];
            //         best_dilivery = di;
            //     }
            // }
            if (Boats[bi].goods_num >= BoatCapacity || MAX_FRAME - Frame <= (BerthToDeliveryTime[Boats[bi].dest_berth][BerthNearestDelivery[Boats[bi].dest_berth]][Boats[bi].dir] + 4))
            { // 装满或者不够时间回交货点 就滚
                RunToDeliveryGun(bi);
                // 清空路径并且寻路（实际上靠泊的时候路径就一定清空了）
                is_need_astar[bi] = true;
            }
            else
            { // 没有装满，有货继续装，没货就调度
                if (Berths[Boats[bi].dest_berth].goods_queue.size() == 0)
                { // 港口上没货了，选一个最好的港口或者调度
                    int best_berth = FindBestBerthOrGoFromBerth(bi);
                    if (best_berth == -2)
                    { // 直接滚
                        RunToDeliveryGun(bi);
                        is_need_astar[bi] = true;
                    }
                    else if (best_berth != -1)
                    { // 去其他港口
                        Berths[Boats[bi].dest_berth].focus = 0;
                        Berths[Boats[bi].dest_berth].boat_id = -1; // 清除港口的船
                        Boats[bi].dest_berth = best_berth;         // 加入港口目的地
                        Boats[bi].dest_delivery = -1;
                        Berths[best_berth].boat_id = bi; // 目的港口的船清空
                        Boats[bi].status = 0;            // 清空状态避免后面装货
                        // 清空路径并且寻路（实际上靠泊的时候路径就一定清空了）
                        BoatRoutes[bi].clear();
                        // 去其他港口也要寻路的
                        is_need_astar[bi] = true;
                    }
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
                // 卸货
                OurMoney += Boats[bi].goods_value;
                Boats[bi].goods_num = 0;
                Boats[bi].goods_value = 0;
                // 确定目标港口
                int best_berth = FindBestBerthFromDelivery(bi, Boats[bi].dest_delivery);
                if (best_berth != -1)
                {
                    Boats[bi].dest_delivery = -1;
                    Boats[bi].dest_berth = best_berth;
                    Berths[best_berth].boat_id = bi;
                    // 确定港口之后是要寻路的
                    is_need_astar[bi] = true;
                }
                else
                { // 找不到可以去的港口了，后面重置
                    Boats[bi].dest_delivery = -1;
                    Boats[bi].dest_berth = -1;
                }
            }
            else
            {
                if (BoatRoutes[bi].size() == 1)
                { // 新买的船
                    is_need_astar[bi] = true;
                }
                else if (BoatRoutes[bi].empty())
                { // 上一帧没找到航线的船需要把自己当前状态加入防止碰撞
                    BoatRoutes[bi].push_back(BoatRouteState(Boats[bi].x, Boats[bi].y, Boats[bi].dir, 3));
                    is_need_astar[bi] = true;
                }
            }
        }
        // 对于偏航的船（要清空路径，只保留当前位置，然后重新寻路）
        if (!BoatRoutes[bi].empty() && (BoatRoutes[bi][0].x != Boats[bi].x || BoatRoutes[bi][0].y != Boats[bi].y || BoatRoutes[bi][0].dir != Boats[bi].dir))
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
            else if (Boats[bi].dest_delivery != -1)
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
        else if (BoatRoutes[bi].size() == 1 && Boats[bi].dest_berth == -1 && Boats[bi].dest_delivery == -1)
        { // 没有目的地的，重置
            printf("dept %d\n", bi);
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
        if (!BoatRoutes[bi].empty())
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
        if (Boats[b].status == 2 && Boats[b].goods_num < BoatCapacity)
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

// 买一艘船，并指派其去的地方
void BuyABoat(int boat_buying_index, int berth_index)
{
    printf("lboat %d %d\n", BoatBuyings[boat_buying_index].x, BoatBuyings[boat_buying_index].y);     // 输出指令
    Boats[BoatNum] = Boat(BoatBuyings[boat_buying_index].x, BoatBuyings[boat_buying_index].y, 0, 0); // 船的初始化
    Boats[BoatNum].dest_berth = berth_index;
    Berths[berth_index].boat_id = (int)BoatNum; // 船的目的地
    // if (RobotNum < MAX_BUY_ROBOT_NUM)
    // {
    //     Berths[berth_index].focus = 1;
    // }
    BoatNum++;
    Money -= BOAT_BUY_MONEY;    // 花钱
    OurMoney -= BOAT_BUY_MONEY; // 花钱
}

// 购买船舶 (用最大数量来限制，能买就买，买来去当时最多货的港口)
int BuyBoatsXmc()
{
    int buy = 0;
    if (Frame == 1)
    {
        for (int i = 0; i < MAX_BOAT_BUYING_NUM; i++)
        {
            if (InitBuyingToBuy[i] == -1)
                break;
            else
            {
                BuyABoat(InitBuyingToBuy[i], InitBerthToGo[i]); // 购买船舶
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
            double max_goods_value_distance = -1; // 最大货物价值距离
            int berth_index_to_go = -1;           // 要去的港口
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
                double value_distance;
                if ((value_distance = (double)total_goods_value /
                                      (BuyingToBerthTime[BerthNearestBuying[be]][be] + BerthToDeliveryTimeMax[be][BerthNearestDelivery[be]])) > max_goods_value_distance)
                {
                    max_goods_value_distance = value_distance;
                    berth_index_to_go = be;
                }
            }
            if (berth_index_to_go == -1)
            {
                return buy;
            }
            if (JudgeBoatBuyingValid(BerthNearestBuying[berth_index_to_go]))
            {
                BuyABoat(BerthNearestBuying[berth_index_to_go], berth_index_to_go); // 购买船舶
                buy = 1;
            }
        }
    }
    return buy;
}
