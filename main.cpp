#ifdef _WIN32
#include <bits/stdc++.h>
#else
#include <cstdlib>
#include <queue>
#include <cstdio>
#include <algorithm>
#include <set>
#endif

using namespace std;

const int robot_num = 10;
const int berth_num = 10;
const int boat_num = 5;
const int N = 200;

int money, boat_capacity, frame; // 金钱，船的容量，当前帧数
int world[N][N]; // 地图
int berth_path[berth_num][N][N] = {0}; // 泊位到每个点的最短路径(1: 上，2: 下，3: 左，4: 右)
int berth_path_len[berth_num][N][N] = {0}; // 泊位到每个点的最短路径长度

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
int next_goods = 0; // 下一个货物的编号

struct Robot
{
    int x, y;  // 机器人的坐标
    int is_goods;  // 机器是否携带货物 (0：没有，1：有)
    int status; // 机器人的状态（0：恢复，1：运行）
    int next_x, next_y;
    int goods_index;  // 机器人携带的货物编号
    int berth_index;  // 机器人要去的泊位编号

    Robot() {}
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
    int SumGoodsValue()
    {
        int sum = 0;
        queue<int> temp = this->goods_queue;
        for(int i = 0; i < temp.size(); i++)
        {
            sum += goods[temp.front()].val;
            temp.pop();
        }
        return sum;
    }
}berth[berth_num];


struct Boat
{
    int pos; // 船的位置（港口id）（-1: 在虚拟点）
    int status; // 船的状态（0：移动或运输中，1：装货或运输完成，2:泊位外等待）
    int goods_num; // 船上货物的数量

    Boat(){}
    Boat(int pos, int status) {
        this -> pos = pos;
        this -> status = status;
        this -> goods_num = 0;
    }
    int FindBestBerth(set <int> busy_berth) // 返回最佳泊位编号
    {
        int value_time = 0;
        int berth_tmp = -1;
        for(int i = 0; i < berth_num; i++)
        {
            if(berth[i].goods_queue.empty()) // 泊位上没有货物
            {
                return -1;
            }
            else if(busy_berth.find(i) == busy_berth.end()) // 泊位是空闲的
            {
                int load_time = berth[i].goods_queue.size() / berth[i].loading_speed;
                int value_time_tmp = berth[i].SumGoodsValue() / load_time;
                if(value_time_tmp > value_time)
                {
                    value_time = value_time_tmp;
                    berth_tmp = i;
                }
            }
        }
        return berth_tmp;
    }

    void LoadGoods() // 装货
    {
        if(this->status == 1 && this->pos != -1) // 船只在装货或装货完成
        {
            if(this->goods_num < boat_capacity && berth[this->pos].goods_queue.size() > 0)
            {
                int add_num;
                add_num = min(boat_capacity - this->goods_num, int(berth[this->pos].goods_queue.size()));
                add_num = min(add_num, berth[this->pos].loading_speed);
                for (int i = 0; i < add_num; i++) {
                    this->goods_num += 1;
                    berth[this->pos].goods_queue.pop();
                }
            }
            else
            {
                return;
            }
        }
    }
}boat[boat_num];

set <int> BusyBerth() // 返回泊位上有货物的泊位编号
{
    set <int> busy_berth;
    for(int i = 0; i < boat_num; i++)
    {
        if(boat[i].pos != -1)
        {
            busy_berth.insert(boat[i].pos);
        }
    }
    return busy_berth;
}

/* 船只调度函数 */
void BoatDispatch()
{
    for(int i = 0; i < boat_num; i++)
    {
        if(boat[i].status == 0) // 船只在移动中
        {
            continue;
        }
        else if(boat[i].status == 1) // 船只在装货或运输完成
        {
            if(boat[i].pos == -1) // 船只在虚拟点
            {
                int best_berth = boat[i].FindBestBerth(BusyBerth());
                if (best_berth != -1) {
                    printf("ship %d %d\n", i, best_berth);
                }
                else
                    return;
            }
            else // 船只在港口
            {
                if(boat[i].goods_num == boat_capacity) // 船只装满货物
                {
                    printf("go %d\n", i);
                }
                else
                {
                    boat[i].LoadGoods();
                    int min_distance = 40001;
                    for(int j = 0; j < robot_num; j++)
                        if(robots[j].berth_index == boat[i].pos && robots[j].is_goods == 1) // 机器人锁定港口且带货
                        {
                            int distance = berth_path_len[robots[j].berth_index][robots[j].x][robots[j].y];
                            if(distance < min_distance)
                            {
                                min_distance = distance;
                            }
                        }
                    if(min_distance > berth[boat[i].pos].transport_time) // 没有机器人锁定港口且带货
                    {
                        printf("go %d\n", i);
                    }
                }
            }
        }
        else if(boat[i].status == 2)// 船只在等待
        {
            int best_berth = boat[i].FindBestBerth(BusyBerth());
            if (best_berth != -1) {
                printf("ship %d %d\n", i, best_berth);
            }
            else
                return;
        }
    }
}



void Init()
{
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
        scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time, &berth[id].loading_speed);
    }

    scanf("%d", &boat_capacity);
    char ok[100];
    scanf("%s", ok);
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
