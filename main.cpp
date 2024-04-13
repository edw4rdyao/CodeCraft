# include "head/function.h"

#define DEBUG

int Map;

//extern const int MAX_FRAME = 15000;

//extern const unsigned MAX_ROBOT_NUM = 100;
unsigned int RobotNum = 0;

//extern const unsigned int MAX_BERTH_NUM = 10;
unsigned int BerthNum = 0;

//extern const int MAX_BOAT_NUM = 100;
unsigned int BoatNum = 0;

//extern const int MAX_DELIVERY_NUM = 10;
unsigned int DeliveryNum = 0;

//extern const int MAX_ROBOT_BUYING_NUM = 10;
unsigned int RobotBuyingNum = 0; // 机器人购买点数量

//extern const int MAX_BOAT_BUYING_NUM = 10;
unsigned int BoatBuyingNum = 0;

//extern const int N = 200;
//extern const int MAX_GOODS_NUM = 150010;
//extern const int MAX_LENGTH = 40000;

int GoodsValue = 0;

// 可以修改的参数 每个机器人BFS找的最多路径与长度 离货物距离的比重
//extern const int MAX_ROAD_NUM = 10;
//extern const int MAX_ROAD_LEN = 350;
//extern const double TO_GOODS_WEIGHT = 3.0;
//extern const double H_VALUE_WEIGHT = 2.0;

// 最大购买机器人和船的数量，购买机器人和船的价格
int MAX_BUY_ROBOT_NUM = 15;
int MAX_BUY_BOAT_NUM = 1;

//extern const int MAX_BUY_BOAT_NUM_ARRAY[10] = {0, 1, 1, 1, 1, 2, 2, 3, 3, 3};
//extern const int ROBOT_BUY_MONEY = 2000;
//extern const int BOAT_BUY_MONEY = 8000;

//extern const int DX[4] = {-1, 1, 0, 0};     // 每个方向x轴的偏移
//extern const int DY[4] = {0, 0, -1, 1};     // 每个方向y轴的偏移
//extern const int REV_DIR[4] = {1, 0, 3, 2}; // 上下左右的反方向（用于BFS记录路径）：下上右左

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
//extern const int DX_BOAT[4][6] = {{0, 0, 0, 1, 1, 1},     // 右
//                                  {0, 0, 0, -1, -1, -1},  // 左
//                                  {0, -1, -2, 0, -1, -2}, // 上
//                                  {0, 1, 2, 0, 1, 2}};    // 下

//extern const int DY_BOAT[4][6] = {{0, 1, 2, 0, 1, 2},     // 右
//                                  {0, -1, -2, 0, -1, -2}, // 左
//                                  {0, 0, 0, 1, 1, 1},     // 上
//                                  {0, 0, 0, -1, -1, -1}}; // 下

// 船：0右 1左 2上 3下，顺逆时针后各个方向对应的方向
//extern const int ROT_DIR[2][4] = {{3, 2, 0, 1},
//                                  {2, 3, 1, 0}};
// 船：0顺时针 1逆时针 转之后核心点移动到的位置 2号位置 ：核心点变到4号位置
//extern const int ROT_POS[2] = {2, 4};

// 每个方向左上角/右下角对应几号位置 0右 1左 2上 3下
//extern const int BOAT_LEFT_TOP[4] = {0, 5, 2, 3};
//extern const int BOAT_RIGHT_DOWN[4] = {5, 0, 3, 2};

int robot_buy_link_to_berth[MAX_ROBOT_BUYING_NUM] = {0}; // 统计有多少港口跟购买点连通

int OurMoney = 25000;           // 自己算出来的金钱
int RobotMoney = 0;             // 机器人拿的的钱
int Money, BoatCapacity, Frame; // 金钱，船的容量，当前帧数
unsigned int World[N][N];       // 地图
int WorldGoods[N][N];           // 地图上每个位置的商品

vector<int> BerthPath[MAX_BERTH_NUM][N][N]; // 机器人泊位到每个点的最短路径第一步的可能方向(0: 上，1: 下，2: 左，3: 右)
int BerthPathLength[MAX_BERTH_NUM][N][N];   // 机器人泊位到每个点的最短路径长度

int DeliveryToBerthTime[MAX_DELIVERY_NUM][MAX_BERTH_NUM][4]; // 交货点到港口的最短时间(每个方向)
int BerthToDeliveryTime[MAX_BERTH_NUM][MAX_DELIVERY_NUM][4]; // 港口到交货点最短时间(每个方向)
int BerthToDeliveryTimeMax[MAX_BERTH_NUM][MAX_DELIVERY_NUM]; // 港口到交货点最短时间(不知道方向，所以这个存的是四个方向的最大值)
int BuyingToBerthTime[MAX_BOAT_BUYING_NUM][MAX_BERTH_NUM];   // 船舶购买点到港口的最短时间(只有正方向)
int BerthToBerthTime[MAX_BERTH_NUM][MAX_BERTH_NUM][4];       // 港口到港口的最短时间(每个方向)
int BerthNearestBuying[MAX_BERTH_NUM];                       // 港口最近的购买点
int BerthNearestDelivery[MAX_BERTH_NUM];                     // 港口最近的交货点

// 估算的到交货点或者港口的最短时间（用于启发函数）
int ToBerthEstimateTime[MAX_BERTH_NUM][N][N];
int ToDeliveryEstimateTime[MAX_DELIVERY_NUM][N][N];

// 记录初始购买机器人数目和船的数目，用于调参
int InitBuyRobotNum = 8;
int InitBuyBoatNum = (25000 - 2000 * InitBuyRobotNum) / 8000;

// 初始机器人购买点购买的机器人数量
int InitRobotToBuy[MAX_ROBOT_BUYING_NUM];

// 初始要去的港口、购买船的位置
int InitBerthToGo[MAX_BOAT_BUYING_NUM];
int InitBuyingToBuy[MAX_BOAT_BUYING_NUM];

int AllocateRobotNum[MAX_ROBOT_NUM] = {0}; // 每个购买点分配的机器人数
int AreaBuying[MAX_ROBOT_NUM] = {0};       // 每个购买点占据面积大小
int Area = 0;                              // 总面积大小

int test;
int LinkMaxBoatBuying = 0;
int AllocateBoatNum[MAX_BERTH_NUM] = {0};

vector<int> AStarSearchNodeNum; // 每次Astar搜索的节点数

// 保存每艘船的固定航线
vector<BoatRouteState> BoatRoutes[MAX_BOAT_NUM];
// 保存每艘船找不到航道的次数
int BoatCanNotFindRoute[MAX_BOAT_NUM];

Goods AllGoods[MAX_GOODS_NUM];
int NextGoodsIndex = 1; // 下一个货物的编号

Robot Robots[MAX_ROBOT_NUM];

Berth Berths[MAX_BERTH_NUM];

Boat Boats[MAX_BOAT_NUM];

RobotBuying RobotBuyings[MAX_ROBOT_BUYING_NUM];

BoatBuying BoatBuyings[MAX_BOAT_BUYING_NUM];

Delivery Deliveries[MAX_DELIVERY_NUM];

double length = 0; // 统计港口到虚拟点的平均


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
        GoodsValue += val;
        if (val == 0)
        { // 消失或者被拿走的物品
            WorldGoods[x][y] = 0;
        }
        else
        { // 新增的物品
            AllGoods[NextGoodsIndex] = Goods(x, y, val, Frame);
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

// 购买函数
void Buy()
{
    if (Frame == 1)
    {
        BuyRobotsXmc();
        BuyBoatsXmc();
        return;
    }
    int buy_r, buy_b;
    while (true)
    {
        buy_r = BuyRobotsXmc();
        if (buy_r == 0)
            break;
    }
    while (true)
    {
        buy_b = BuyBoatsXmc();
        if (buy_b == 0)
            break;
    }
}


// main函数
int main()
{
    Init();
    AllocateRobot();
    linkMaxBuyBoat();
    AllocateBoat();

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