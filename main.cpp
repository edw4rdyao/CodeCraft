#ifdef _WIN32
#include <bits/stdc++.h>
#else
#include <iostream>
#include <cstdlib>
#include <queue>
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
    int lock; // 货物是否被机器人锁定

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
    int next_x, next_y;
    int goods_index;  // 机器人携带的货物编号
    int berth_index;  // 机器人要去的泊位编号
    int is_dead; // 机器人是否被困死（0：没有，1：有）

    Robot() {}
    Robot(int x, int y) {
        this -> x = x;
        this -> y = y;
        this -> is_goods = 0;
        this -> status = 1;
        this -> next_x = -1;
        this -> next_y = -1;
        this -> goods_index = -1;
        this -> berth_index = -1;
    }
    Robot(int x, int y, int is_goods, int status) {
        this -> x = x;
        this -> y = y;
        this -> is_goods = 0;
        this -> status = 1;
        this -> next_x = -1;
        this -> next_y = -1;
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
int berth_path[berth_num][N][N] = {0}; // 泊位到每个点的最短路径(1: 上，2: 下，3: 左，4: 右)
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
        goods[next_goods++] = Goods(x, y, val, frame);
    }
    for (int i = 0; i < robot_num; i++) {
        int is_goods, x, y, status;
        scanf("%d%d%d%d", &is_goods, &x, &y, &status);
        robots[i] = Robot(x, y, is_goods, status);
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
