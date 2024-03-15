#ifdef _WIN32
#include <bits/stdc++.h>
#else
#include <iostream>
#include <cstdlib>
#include <queue>
#include <vector>
#endif

using namespace std;

const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 200;

struct Goods
{
    int x, y;  // 货物的坐标
    int val;   // 货物的价值
    int refresh_frame;  // 货物的刷新帧数
    int lock; // 货物是否被机器人锁定(0:未锁定)

    Goods() {}
    Goods(int x, int y, int val, int refresh_frame) {
        this -> x = x;
        this -> y = y;
        this -> val = val;
        this -> refresh_frame = refresh_frame;
    }
}goods[150000];
int next_goods = 0;

struct Robot
{
    int x, y;  // 机器人的坐标
    int is_goods;  // 机器是否携带货物 (0：没有，1：有)
    int status; // 机器人的状态（0：恢复，1：运行）
    int dir;
    int goods_index;  // 机器人携带的货物编号
    int berth_index;  // 机器人要去的泊位编号
    int is_dead; // 机器人是否被困死（0：没有，1：有）
    int action; //0:get()拿货物， 1:pull()放物品

    Robot() {}
    Robot(int x, int y) {
        this -> x = x;
        this -> y = y;
        this -> is_goods = 0;
        this -> status = 1;
        this -> dir = -1;
        this -> goods_index = -1;
        this -> berth_index = -1;
    }
}robots[robot_num];

struct Berth
{
    int x, y; // 泊位的坐标
    int transport_time; // 货物运输时间
    int loading_speed; // 货物装卸速度
    queue<int> goods_queue;  // 泊位上的货物队列

    Berth(){}
    Berth(int x, int y, int transport_time, int loading_speed) {
        this -> x = x;
        this -> y = y;
        this -> transport_time = transport_time;
        this -> loading_speed = loading_speed;
    }
}berths[berth_num];

struct Boat
{
    int pos; // 船的位置（港口id）（-1: 在虚拟点）
    int status; // 船的状态（0：移动中，1：装货或运输，2:等待）
    int goods_num; // 船上货物的数量

    Boat(){}
    Boat(int pos, int status) {
        this -> pos = pos;
        this -> status = status;
        this -> goods_num = 0;
    }
}boat[boat_num];

int money, boat_capacity, frame; // 金钱，船的容量，当前帧数
int world[N][N]; // 地图
int berth_path[berth_num][N][N] = {0}; // 泊位到每个点的最短路径(0: 上，1: 下，2: 左，3: 右)
int berth_path_len[berth_num][N][N] = {0}; // 泊位到每个点的最短路径长度

const int dx[4] = {-1, 1, 0, 0}; // 每个方向x轴的偏移
const int dy[4] = {0, 0, -1, 1}; // 每个方向y轴的偏移
const int reverse_direction[4] = {1, 0, 3, 2}; // 上下左右的反方向：下上右左

vector<int> GetRandomDirection()
{
    vector<int> random_dir = {0, 1, 2, 3};
    random_device rd;
    mt19937 g(rd());
    shuffle(random_dir.begin(), random_dir.end(), g);
}

void ToBerthBFS()
{
    // 初始化所有泊位到每个点的最短路径及长度，均为-1
    memset(berth_path, -1, sizeof(berth_path));
    memset(berth_path_len, -1, sizeof(berth_path_len));
    // 对每个泊位（左上角->中间点）开始做BFS
    for(int b = 0; b < berth_num; b++)
    {
        int berth_x = berths[b].x + 1;
        int berth_y = berths[b].y + 1;
        queue<pair<int, int>> q;
        q.push({berth_x, berth_y});
        // 泊位本身的路径长度为0
        berth_path_len[b][berth_x][berth_y] = 0;
        while(!q.empty())
        {
            // 从队列中取出一个点
            pair<int, int> cur_pos = q.front();
            q.pop();
            // 四个方向随机遍历
            vector<int> random_dir = GetRandomDirection();
            for(int i = 0; i < 4; i ++)
            {
                // 遍历到下一个点
                int dir = random_dir[i];
                int nx = cur_pos.first + dx[dir];
                int ny = cur_pos.second + dy[dir];
                // 判断该点是否可以到达(没有越界&&为空地或者泊位&&之前没有到达过)
                if(nx >= 0 && ny < N && ny >= 0 && ny < N && (world[nx][ny] == 0 || world[nx][ny] == 3)  && berth_path_len[b][nx][ny] < 0)
                {
                    // 路径长度+1
                    berth_path_len[b][nx][ny] = berth_path_len[b][cur_pos.first][cur_pos.second];
                    // 记录路径的方向
                    berth_path[b][nx][ny] = reverse_direction[dir];
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
    for(int r = 0; r < robot_num; r ++)
    {
        int can_get = 0;
        // 对每个港口进行检查
        for(int b = 0; b < berth_num; b++)
        {
            if(berth_path_len[b][robots[r].x][robots[r].y] >= 0)
            {
                can_get ++;
            }
        }
        // 如果对每个港口都不可达，则被困死
        if(can_get == 0)
        {
            robots[r].is_dead = 1;
        }
    }
}

void Init()
{
    int r_num = 0;
    for(int i = 0; i < N; i++)
    {
        for(int j = 0; j < N; j++)
        {
            char ch;
            scanf("%c", &ch);
            if(ch == '.' || ch == 'A')
            {
                world[i][j] = 0; // 空地
            }
            else if(ch == '*')
            {
                world[i][j] = 1; // 海洋
            }
            else if(ch == '#')
            {
                world[i][j] = 2; // 障碍
            }
            else if(ch == 'B')
            {
                world[i][j] = 3; // 港口
            }
            else if(ch == 'A')
            {
                world[i][j] = 0; // 有机器人的空地
                // 机器人的初始位置(i,j)，用robots数组存下来
                robots[r_num ++] = Robot(i , j);
            }
            else
            {
                world[i][j] = -1; // 其他
            }
        }
        char ch;
        scanf("%c", &ch);
    }

    for(int i = 0; i < berth_num; i++)
    {
        int id;
        scanf("%d", &id);
        scanf("%d%d%d%d", &berths[id].x, &berths[id].y, &berths[id].transport_time, &berths[id].loading_speed);
    }

    scanf("%d", &boat_capacity);
    char ok[100];
    scanf("%s", ok);

    // BFS存储每个港口到地图每个点最短路径和距离
    ToBerthBFS();
    // 判断每个机器人和所有港口的可达性（如果都不可达则被困死）
    JudgeRobotsLife();

    printf("OK\n");
    fflush(stdout);
}

int Input() {
    scanf("%d%d", &frame, &money);
    int goods_num;
    scanf("%d", &goods_num);
    for (int i = 1; i <= goods_num; i++) {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        goods[next_goods] = Goods(x, y, val, frame);
        world[x][y] = 100+next_goods;
        next_goods++;
    }
    for (int i = 0; i < robot_num; i++) {
        int is_goods, x, y, status;
        scanf("%d%d%d%d", &is_goods, &x, &y, &status);
        // robots[i] = Robot(x, y, is_goods, status);
        robots[i].x = x;
        robots[i].y = y;
        robots[i].is_goods = is_goods;
        robots[i].status = status;


    }
    for (int i = 0; i < boat_num; i++) {
        int status, pos;
        scanf("%d%d\n", &status, &pos);
        boat[i] = Boat(pos, status);
    }

    char ok[100];
    scanf("%s", ok);

    return frame;
}


// 计算物品性价比
int CalculateGoodsValue(int goods_index, int step_num, int &to_berth_index){
    int goods_value; // 物品价值
    int goods_x, goods_y; // 物品坐标
    int to_berth_len = 40001; // 物品到港口距离
    goods_value = goods[goods_index].val;
    goods_x = goods[goods_index].x;
    goods_y = goods[goods_index].y;

    //确定最近港口
    for (int i = 0; i < berth_num; i++){
        if (berth_path_len[i][goods_x][goods_y] <= to_berth_len){
            to_berth_len = berth_path_len[i][goods_x][goods_y];
            to_berth_index = i;
        }
    }
    
    return goods_value/(step_num + to_berth_len);
}

// 方向数组，表示0:上、1:下、3:左、4:右移动
const int dx[] = {-1, 1, 0, 0}; //x方向
const int dy[] = {0, 0, -1, 1}; //y方向

// 检查坐标(x, y)是否在地图范围内并且不是障碍物
bool isValid(int x, int y) {
    return x >= 0 && x < N && y >= 0 && y < N && world[x][y] != 1 && world[x][y] != 2;
}

struct Road //物品id、最近港口、机器人下一步方向
{
    int goods_index;
    int berth_index;
    int next_dir;

    Road() {}
    Road(int goods_index, int berth_index, int next_dir) {
        this -> goods_index = goods_index;
        this -> berth_index = berth_index;
        this -> next_dir = next_dir;
    }
};

// 使用BFS算法找到最好的3个物品,返回物品index、最近港口和下一步方向
vector<Road> ToGoodsBFS(int robot_index, int total_step){
    int robot_x, robot_y; // 机器人的坐标
    robot_x = robots[robot_index].x;
    robot_y = robots[robot_index].y;

    int visited[N][N] = {0}; // 访问标记数组
    queue<pair<int, int>> q; // BFS队列,坐标值
    queue<int> direction; // 用于记录到达每个位置的移动方向
    vector<Road> best_goods; //保存返回的物品

    q.push({robot_x, robot_y}); // 将机器人初始位置入队
    visited[robot_x][robot_y] = 1; // 标记为已访问
    direction.push(-1); // -1表示初始位置，没有实际方向意义

    int step = 0; // 当前步数
    while (!q.empty() && step < total_step){
        int min_val = 201; //当前最小价值
        int mid_val; //第二小价值
        int max_val = 0; //当前最大价值

        int size = q.size();
        step++;
        for (int i = 0; i < size; ++i) {
            auto [x, y] = q.front(); 
            q.pop();
            int dir = direction.front(); 
            direction.pop();

            if (world[x][y]-100 >= 0 && step <= goods[world[x][y]-100].refresh_frame) { // 找到物品
                int berth_index;
                int current_val = CalculateGoodsValue(world[x][y]-100, step, berth_index); //计算物品性价比
                if (best_goods.size() < 3){ //已找到小于三个
                    if (min_val > current_val){ //最小价值总是插在末尾
                        if (min_val != 201){ //不是第一个插入的
                            mid_val = min_val;
                        }
                        min_val = current_val;
                        best_goods.push_back(Road(world[x][y]-100, berth_index, dir));
                    }
                    else{
                        if (max_val < current_val){ //最大的插在开头
                            max_val = current_val;
                            best_goods.insert(best_goods.begin(),Road(world[x][y]-100, berth_index, dir));
                        }
                        else{
                            mid_val = max_val;
                            best_goods.insert(++best_goods.begin(),Road(world[x][y]-100, berth_index, dir));
                        }
                    }
                }
                else{ //保留最大的3个
                    if (min_val <= current_val){ //最小价值总是插在末尾
                        if (max_val < current_val){ //当前最大
                            min_val = mid_val;
                            mid_val = max_val; 
                            max_val = current_val;
                            best_goods.insert(best_goods.begin(),Road(world[x][y]-100, berth_index, dir));
                            best_goods.pop_back(); //弹出最小的
                        }
                        else if (mid_val <= current_val){ //当前为第二小
                            min_val = mid_val;
                            mid_val = current_val;
                            best_goods.pop_back(); //弹出最小的
                            best_goods.insert(++best_goods.begin(),Road(world[x][y]-100, berth_index, dir));
                        }
                        else{ //当前最小
                            min_val = current_val;
                            best_goods.pop_back();
                            best_goods.push_back(Road(world[x][y]-100, berth_index, dir));
                        }
                    }
                }
            }

            for (int d = 0; d < 4; ++d) { // 检查四个方向
                int nx = x + dx[d];
                int ny = y + dy[d];

                if (isValid(nx, ny) && !visited[nx][ny]) { //检查下一步是否可走
                    visited[nx][ny] = 1; //标记已走过
                    q.push({nx, ny}); //存储下一步位置
                    direction.push(d);  //存储对应的方向
                }
            }
        }
    }
    return best_goods;
}

//判断是否在港口内
bool IsInBerth(int robot_index){
    if (world[robots[robot_index].x][robots[robot_index].y] == 3){
        return true;
    }
    return false;
}

//判断是否在物品上
bool IsInGoods(int robot_index){
    if (world[robots[robot_index].x][robots[robot_index].y] >= 100){
        return true;
    }
    return false;
}

// 每个机器人本帧动作
int RobotFindGoods(){
    vector<vector<Road>> roads; //每个机器人各自最好的几个商品和对应的下一步
    for (int i = 0; i < robot_num; i++){
        vector<Road> current_road; //当前机器人BFS结果
        if (robots[i].is_dead == 1 or robots[i].status == 0){ //若机器人不能到港口就不考虑
            roads.push_back(current_road);
            robots[i].dir = -1;
            continue;
        }

        if (robots[i].is_goods == 1){ //若拿着物品
            roads.push_back(current_road);
            if (IsInBerth(i)){ //到达港口
                robots[i].action = 1;
                robots[i].dir = -1;
            }
            else{ //没到港口，沿着图走
                robots[i].dir = berth_path[robots[i].berth_index][robots[i].x][robots[i].y];
            }
            continue;
        }

        current_road = ToGoodsBFS(i, 80);
        roads.push_back(current_road); //最多找80步
    }

    for (int i = 0; i < robot_num; i++)
    {
        if (roads[i].empty()){ //若没有bfs到物品或机器人不能到港口或已经拿了物品
            if (robots[i].is_dead != 1 && robots[i].is_goods != 1){ //没拿物品也没死
                robots[i].dir = -1;
            } 
            continue;
        }

        if (IsInGoods(i)){ //在物品上
            int j = 0;
            for (j = 0; j < roads[i].size(); j++){
                int goods_id = roads[i][j].goods_index;
                if (goods[goods_id].lock == 0 && goods[goods_id].x == robots[i].x && goods[goods_id].y == robots[i].y){ //确认过眼神，是我要拿的人
                    //拿物品
                    goods[goods_id].lock = 1; //物品上锁
                    robots[i].goods_index = goods_id; //机器人拿的物品编号
                    robots[i].berth_index = roads[i][j].berth_index; //要去的港口编号
                    robots[i].action = 0; //拿物品的动作
                    robots[i].dir = berth_path[robots[i].berth_index][robots[i].x][robots[i].x]; //按图找方向
                    world[robots[i].x][robots[i].x] = 0; //地图上去除物品
                    break;
                }
            }
            if (j == roads[i].size()){ //不是我要拿的
                //按road走
                int k = 0;
                for (k = 0; k < roads[i].size(); k++){
                    int goods_id = roads[i][k].goods_index;
                    if (goods[goods_id].lock == 0){
                        if (robots[i].goods_index != goods_id){
                            goods[robots[i].goods_index].lock = 0; //物品解锁
                        }
                        robots[i].goods_index = goods_id; //机器人要拿的物品编号
                        goods[goods_id].lock = 1; //物品上锁
                        robots[i].dir = roads[i][k].next_dir; //机器人方向
                        break;
                    }
                }
                if (j == roads[i].size()){ //没东西拿
                    goods[robots[i].goods_index].lock = 0; //物品解锁
                    robots[i].dir = -1; //罚站
                }
            }
        }
        else{ //按road走
            int j = 0;
            for (j = 0; j < roads[i].size(); j++){
                int goods_id = roads[i][j].goods_index;
                if (goods[goods_id].lock == 0){
                    if (robots[i].goods_index != goods_id){
                        goods[robots[i].goods_index].lock = 0; //物品解锁
                    }
                    robots[i].goods_index = goods_id; //机器人要拿的物品编号
                    goods[goods_id].lock = 1; //物品上锁
                    robots[i].dir = roads[i][j].next_dir; //机器人方向
                    break;
                }
            }
            if (j == roads[i].size()){ //没东西拿
                goods[robots[i].goods_index].lock = 0; //物品解锁
                robots[i].dir = -1; //罚站
            }
        }
    }
}




int main()
{
    Init();
    for(int zhen = 1; zhen <= 15000; zhen++)
    {
        int id = Input();
        for(int i = 0; i < robot_num; i++)
            printf("move %d %d\n", i, rand() % 4);
        puts("OK");
        fflush(stdout);
    }

    return 0;
}
