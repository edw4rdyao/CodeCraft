#include "head/function.h"

// 没拿物品的机器人之间比较要拿物品的性价比
bool NoGoodsRobotsCompare(int ri, int rj)
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
    double ri_val = 0;
    double rj_val = 0;
    // if (RobotNum < MAX_BUY_ROBOT_NUM)
    // {
    // ri_val = (double)AllGoods[goodsi].val / (Robots[ri].goods_distance * TO_GOODS_WEIGHT + BerthPathLength[Robots[ri].berth_index][AllGoods[goodsi].x][AllGoods[goodsi].y]); // 机器人i要拿物品的性价比
    // rj_val = (double)AllGoods[goodsj].val / (Robots[rj].goods_distance * TO_GOODS_WEIGHT + BerthPathLength[Robots[rj].berth_index][AllGoods[goodsj].x][AllGoods[goodsj].y]); // 机器人j要拿物品的性价比
    ri_val = (double)AllGoods[goodsi].val; // 机器人i要拿物品的性价比
    rj_val = (double)AllGoods[goodsj].val; // 机器人j要拿物品的性价比
    // }
    // else
    // {
    //     double ri_val = (double)AllGoods[goodsi].val * ri_rest_time_weight / (Robots[ri].goods_distance * TO_GOODS_WEIGHT + BerthPathLength[Robots[ri].berth_index][AllGoods[goodsi].x][AllGoods[goodsi].y]); // 机器人i要拿物品的性价比
    //     double rj_val = (double)AllGoods[goodsj].val * rj_rest_time_weight / (Robots[rj].goods_distance * TO_GOODS_WEIGHT + BerthPathLength[Robots[rj].berth_index][AllGoods[goodsj].x][AllGoods[goodsj].y]); // 机器人j要拿物品的性价比
    // }
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
bool GetGoodsRobotsCompare(int ri, int rj)
{
    int goodsi = Robots[ri].goods_index; // 机器人i拿的物品
    int goodsj = Robots[rj].goods_index; // 机器人j拿的物品
    // double ri_val = (double)AllGoods[goodsi].val / BerthPathLength[Robots[ri].berth_index][Robots[ri].x][Robots[ri].y]; // 机器人i物品的性价比
    // double rj_val = (double)AllGoods[goodsj].val / BerthPathLength[Robots[rj].berth_index][Robots[rj].x][Robots[rj].y]; // 机器人j物品的性价比
    double ri_val = (double)AllGoods[goodsi].val;
    double rj_val = (double)AllGoods[goodsj].val;
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
        { // 如果现在的位置有物品，
            // 先看到了之后它是否消失
            if (cur_path_length < (1000 - (Frame - AllGoods[goods_id].fresh)))
            {
                // 计算最近港口和性价比，并将路径加入
                if ((Robots[ri].goods_index == -1 && AllGoods[goods_id].robot_id == -1) || Robots[ri].goods_index == goods_id)
                { // 如果自己没东西要拿，并且现在找到一个没人要拿的 或者 找到了自己要拿的
                    int to_berth_index = -1;
                    double value = 0;
                    value = CalculateGoodsValue(ri, goods_id, cur_path_length, to_berth_index, 0);
                    {
                        lock_guard<mutex> lock(roads_pq_mutex); // 锁住互斥锁
                        roads_pq.push(Road(ri, goods_id, cur_path_length, to_berth_index, goods_path[cur_pos.first][cur_pos.second], value));
                    }
                    ri_road_num++;
                    if (Robots[ri].goods_index == goods_id)
                    { // 如果这个机器人找到了自己要拿的 就不找了
                        break;
                    }
                }
            }
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
        if ((Robots[ri].type == 0 && Robots[ri].is_goods == 1) || (Robots[ri].type == 1 && Robots[ri].is_goods == 2))
        { // 如果机器人已经装满拿着物品就直接找港口
            int now_berth_id = IsOnBerth(Robots[ri].x, Robots[ri].y);
            if (now_berth_id >= 0 && now_berth_id == Robots[ri].berth_index)
            { // TODO 如果到达港口，放下货物，继续找其他货物（要判断是不是自己要去的港口）
                while (!Robots[ri].goods_stack.empty())
                {
                    int goods_id = Robots[ri].goods_stack.top();
                    RobotMoney += AllGoods[goods_id].val;
                    Robots[ri].action = 1;
                    Berths[Robots[ri].berth_index].goods_queue.push(goods_id); // 港口放入货物
                    Berths[Robots[ri].berth_index].total_goods_num++;          // 港口放过的总货物数量
                    Robots[ri].goods_index = -1;
                    Robots[ri].goods_distance = MAX_LENGTH;
                    Robots[ri].is_goods = 0;
                }
            }
            else
            { // 没到港口（或者不是自己要去的港口），沿着图走（先看有没有找到被标记的优先港口）
                int berth_id = LastMinBerth(Robots[ri].x, Robots[ri].y);
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
            // if (Robots[ri].type == 1 && Robots[ri].is_goods == 1)
            // { // 已经拿了一个货的不定港口
            //     road.berth_index = -1;
            // }
            Robots[ri].dir = road.next_dir;
            Robots[ri].goods_distance = road.goods_distance;
            // 如果该机器人在自己想要拿的物品上，就拿起并且朝港口走
            int goods_id = IsOnGoods(Robots[ri].x, Robots[ri].y);
            if (goods_id == Robots[ri].goods_index)
            {
                if ((Robots[ri].type == 1 && Robots[ri].is_goods == 1) || Robots[ri].type == 0)
                { // 如果拿满了，朝着港口走
                    Robots[ri].dir = PsbDirToBerth(Robots[ri].berth_index, Robots[ri].x, Robots[ri].y);
                    WorldGoods[Robots[ri].x][Robots[ri].y] = 0;
                }
                else
                { // 金牌机器人没拿满
                    // 已经拿了要拿的了，还没确定下次要拿的，只能停一帧再抉择
                    Robots[ri].goods_index = -1;
                    Robots[ri].dir = -1;
                }
                Robots[ri].action = 0;
                Robots[ri].goods_stack.push(goods_id);
                Robots[ri].is_goods++;
            }
            robot_match_num++;
            robots_match[ri] = 1;
            AllGoods[road.goods_index].robot_id = ri;
            goods_match.insert(road.goods_index);
        }
        roads_pq.pop();
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
            if (GetGoodsRobotsCompare(ri, rj))
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
            if (NoGoodsRobotsCompare(ri, rj))
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
            if (GetGoodsRobotsCompare(ri, rj))
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
            if (NoGoodsRobotsCompare(ri, rj))
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
                            // 若不动的在主干道
                            if (IsOnMainRoad(Robots[rj].x, Robots[rj].y))
                            {
                                continue;
                            }
                            is_collision = true;
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
                                // 若他下一步要进我主干道
                                if (IsOnMainRoad(nx_rj, ny_rj))
                                {
                                    Robots[ri].dir = -1;
                                    break;
                                }
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

// 买一个机器人
void BuyARobot(int robot_buying_index, int type)
{
    printf("lbot %d %d %d\n", RobotBuyings[robot_buying_index].x, RobotBuyings[robot_buying_index].y, type); // 输出指令
    Robots[RobotNum] = Robot(RobotBuyings[robot_buying_index].x, RobotBuyings[robot_buying_index].y, type);  // 机器人的初始化
    RobotNum++;
    Money -= ROBOT_BUY_MONEY[type];    // 花钱
    OurMoney -= ROBOT_BUY_MONEY[type]; // 花钱
    if (RobotNum >= MAX_BUY_ROBOT_NUM)
    {
        // 清除所有港口的聚焦
        for (int i = 0; i < BerthNum; i++)
        {
            // Berths[i].focus = 0;
        }
    }
}

// 购买机器人，用最大数量以及船的数量来限制，能买就买 (xmc)
int BuyRobotsXmc()
{
    int type = 0;
    int buy = 0;
    if (Frame == 1)
    {
        for (int i = 0; i < RobotBuyingNum; i++)
        {
            for (int j = 0; j < InitRobotToBuy[i]; j++)
            {
                if (InitBuyingToBuy[j] == -1)
                    break;
                BuyARobot(i, type);
                buy = 1;
            }
        }
    }
    else
    { // 后续购买
        if (Money < ROBOT_BUY_MONEY[type] || RobotNum >= MAX_BUY_ROBOT_NUM)
        {
            return buy;
        }
        else
        {                                    // 要买
            int goods_stack = MAX_GOODS_NUM; // 货物堆积最少处的最近位置买
            int robot_buy_index;
            // 对每个港口
            for (int bi = 0; bi < BerthNum; bi++)
            {
                if (BerthNearestBuying[bi] < 0)
                { // 船不可达的港口不去
                    continue;
                }
                else
                { // 船可达的港口
                    int length = MAX_LENGTH;
                    // 对每个机器人购买点
                    for (int rbi = 0; rbi < RobotBuyingNum; rbi++)
                    {
                        int x = RobotBuyings[rbi].x;
                        int y = RobotBuyings[rbi].y;
                        if (BerthPathLength[bi][x][y] < 0)
                        { // 机器人不可达
                            continue;
                        }
                        else
                        {
                            if (AllocateRobotNum[rbi] <= 0)
                            {
                                // 不该在该港口买了
                                continue;
                            }
                            if (goods_stack >= Berths[bi].goods_queue.size() && length >= BerthPathLength[bi][x][y])
                            {
                                goods_stack = (int)Berths[bi].goods_queue.size();
                                length = BerthPathLength[bi][x][y];
                                robot_buy_index = rbi;
                            }
                        }
                    }
                }
            }
            AllocateRobotNum[robot_buy_index]--;
            BuyARobot(robot_buy_index, type);
            buy = 1;
        }
    }
    return buy;
}
