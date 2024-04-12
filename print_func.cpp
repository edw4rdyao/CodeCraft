#include "head/function.h"

// 输出实际金钱与我们自己算的金钱
void PrintMoney(ofstream &out_file)
{
    out_file << setw(10) << "Our Money: " << OurMoney << endl;
    out_file << setw(10) << "Real Money: " << Money << endl;
    out_file << setw(10) << "Robot Money: " << RobotMoney << endl;
    out_file << setw(10) << "Goods Money: " << GoodsValue << endl;
}

// 输出机器人的数量
void PrintRobotNum(ofstream &out_file)
{
    out_file << "Robot Num: " << RobotNum << endl;
}

// 输出每个港口的货物量，货物总价值以及是否聚焦，以及装载速度
void PrintBerthGoodsInfo(ofstream &out_file)
{
    int berth_goods_value[MAX_BERTH_NUM] = {0};
    int berth_goods_num[MAX_BERTH_NUM] = {0};
    for (int i = 0; i < BerthNum; i++)
    {
        queue<int> tmp = Berths[i].goods_queue;
        for (int j = 0; j < Berths[i].goods_queue.size(); j++)
        {
            int goods_index = tmp.front();
            tmp.pop();
            berth_goods_value[i] += AllGoods[goods_index].val;
            berth_goods_num[i]++;
        }
        out_file << "Berth " << i << " " << left << setw(3) << berth_goods_num[i] << " goods, value "
                 << setw(5) << berth_goods_value[i] << ", focus " << Berths[i].focus
                 << " boat " << setw(3) << Berths[i].boat_id << " velocity " << setw(3) << Berths[i].velocity << endl;
    }
}

// 输出每艘船的信息
void PrintBoatInfo(ofstream &out_file)
{
    out_file << "Boat Num: " << BoatNum << endl;
    for (int i = 0; i < BoatNum; i++)
    {
        out_file << "Boat " << i << " status " << Boats[i].status
                 << ", at (" << setw(3) << Boats[i].x << "," << setw(3) << Boats[i].y << ") "
                 << "dir " << Boats[i].dir << ", dest berth "
                 << setw(2) << Boats[i].dest_berth << ", dest delivery "
                 << setw(2) << Boats[i].dest_delivery << ", goods num "
                 << setw(3) << Boats[i].goods_num << ", goods value "
                 << setw(3) << Boats[i].goods_value << endl;
    }
}

// 输出地图信息
void PrintWorldInfo(ofstream &out_file)
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (IsInBerthRange(i, j) == -1)
            {
                out_file << '.';
            }
            else
            {
                out_file << IsInBerthRange(i, j);
            }
        }
        out_file << endl;
    }
}

// 输出船的容量
void PrintBoatCapacity(ofstream &out_file)
{
    out_file << "----------------------------------------Boat Capacity-----------------------------------------" << endl;
    out_file << "Boat Capacity: " << BoatCapacity << endl;
}

// 输出每个交货点到每个港口时间
void PrintDeliveryToBerthTime(ofstream &out_file)
{
    out_file << "-----------------------------------Delivery To Berth Time-------------------------------------" << endl;
    for (int di = 0; di < DeliveryNum; di++)
    {
        for (int bi = 0; bi < BerthNum; bi++)
        {
            for (int dir = 0; dir < 4; dir++)
            {
                out_file << "Delivery " << di << " to Berth " << bi << " Dir: " << dir << " Time: " << left << setw(6) << DeliveryToBerthTime[di][bi][dir] << endl;
                // out_file << " Search Node:" << AStarSearchNodeNum[di * DeliveryNum + bi] << endl;
            }
        }
    }
}

// 输出港口每个方向到每个交货点时间
void PrintBerthToDeliveryTime(ofstream &out_file)
{
    out_file << "-----------------------------------Berth To Delivery Time-------------------------------------" << endl;
    for (int di = 0; di < DeliveryNum; di++)
    {
        for (int bi = 0; bi < BerthNum; bi++)
        {
            for (int dir = 0; dir < 4; dir++)
            {
                out_file << "Berth " << bi << " to Delivery " << di << " Dir: " << dir << " Time: " << left << setw(6) << BerthToDeliveryTime[bi][di][dir] << endl;
                // out_file << " Search Node:" << AStarSearchNodeNum[di * DeliveryNum + bi] << endl;
            }
            out_file << "Berth " << bi << " to Delivery " << di << " Max Time: " << left << setw(6) << BerthToDeliveryTimeMax[bi][di] << endl;
        }
    }
}

// 输出港口每个方向到每个交货点时间
void PrintBerthToBerthTime(ofstream &out_file)
{
    out_file << "-----------------------------------Berth To Berth Time----------------------------------------" << endl;
    for (int bi = 0; bi < BerthNum; bi++)
    {
        for (int bj = 0; bj < BerthNum; bj++)
        {
            for (int dir = 0; dir < 4; dir++)
            {
                out_file << "Berth " << bi << " to Berth " << bj << " Dir: " << dir << " Time: " << left << setw(6) << BerthToBerthTime[bi][bj][dir] << endl;
            }
        }
    }
}

// 输出船舶购买点到每个港口的时间
void PrintBuyingToBerthTime(ofstream &out_file)
{
    out_file << "------------------------------------Buying To Berth Time--------------------------------------" << endl;
    for (int bbi = 0; bbi < BoatBuyingNum; bbi++)
    {
        for (int bi = 0; bi < BerthNum; bi++)
        {
            out_file << "Buying   " << bbi << " to Berth " << bi << " Time: " << left << setw(6) << BuyingToBerthTime[bbi][bi] << endl;
            // out_file << " Search Node:" << AStarSearchNodeNum[DeliveryNum * BerthNum + bbi * BoatBuyingNum + bi] << endl;
        }
    }
}

// 输出离每个港口最近的购买点和交货点的index
void PrintNearestBuyingAndDelivery(ofstream &out_file)
{
    out_file << "--------------------------------Nearest Buying And Delivery----------------------------------" << endl;
    for (int bi = 0; bi < BerthNum; bi++)
    {
        out_file << "Berth " << bi << " Nearest Buying: " << BerthNearestBuying[bi] << " Nearest Delivery: " << BerthNearestDelivery[bi] << endl;
    }
}

// 输出每艘船的航线
void PrintBoatRouts(ofstream &out_file)
{
    for (int bi = 0; bi < BoatNum; bi++)
    {
        for (int i = 0; i < BoatRoutes[bi].size(); i++)
        {
            out_file << "(" << BoatRoutes[bi][i].x << "," << BoatRoutes[bi][i].y << "," << BoatRoutes[bi][i].dir << ") ";
        }
        out_file << endl;
    }
}

// 输出初始购买信息
void PrintInitBuy(ofstream &out_file)
{
    out_file << "----------------------------------------Init Buy Info-----------------------------------------" << endl;
    out_file << "Init Buy Robot Num: " << InitBuyRobotNum << endl;
    out_file << "Init Buy Boat Num: " << InitBuyBoatNum << endl;
    for (int i = 0; i < MAX_BOAT_BUYING_NUM; i++)
    {
        if (InitBuyingToBuy[i] == -1 || InitBerthToGo[i] == -1)
            break;
        out_file << "Init Berth to go: " << InitBerthToGo[i] << endl;
        out_file << "Init Boat Buying: " << InitBuyingToBuy[i] << endl;
    }

    out_file << endl;
    out_file << "AREA: " << Area << endl;
    for (int i = 0; i < RobotBuyingNum; i++)
    {
        out_file << "RobotBuying area: " << i << ' ' << AreaBuying[i] << endl;
        out_file << "RobotBuying robot: " << i << ' ' << AllocateRobotNum[i] << endl;
    }

    out_file <<  endl;
    out_file << "LinkSea: " << LinkMaxBoatBuying << endl;
    out_file << "MaxBoatBuy: " << MAX_BUY_BOAT_NUM << endl;
}

// 输出初始购买船在哪买，去哪的信息
void PrintInitBuyBoatInfo(ofstream &out_file)
{
    out_file << "--------------------------------------Init Buy Boat Info--------------------------------------" << endl;
    for (int i = 0; i < MAX_BOAT_NUM; i++)
    {
        if (InitBuyingToBuy[i] == -1 || InitBerthToGo[i] == -1)
            break;
        out_file << "Boat " << i << "：Init Berth to go: " << InitBerthToGo[i] << ", Which Boat Buying to buy: " << InitBuyingToBuy[i] << endl;
    }
}

// 输出信息
void Print(ofstream &out_file, int interval)
{
    if (Frame == 0)
    {
        // PrintWorldInfo(out_file);
        PrintDeliveryToBerthTime(out_file);
        PrintBerthToDeliveryTime(out_file);
        PrintBerthToBerthTime(out_file);
        PrintBuyingToBerthTime(out_file);
        PrintNearestBuyingAndDelivery(out_file);
        PrintBoatCapacity(out_file);
        PrintInitBuy(out_file);
    }

    if (Frame == 1)
        PrintInitBuyBoatInfo(out_file);

    if (Frame % interval != 0)
    {
        return;
    }

    if (out_file.is_open())
    {
        out_file << "----------------------------------------Frame: " << left << setw(5) << Frame << "------------------------------------------" << endl;
        PrintMoney(out_file);
        PrintRobotNum(out_file);
        PrintBoatInfo(out_file);
        PrintBerthGoodsInfo(out_file);
    }
}

string GetTimeString()
{
    // 获得现在的时间
    time_t currentTime = time(nullptr);
    struct tm *localTime = localtime(&currentTime);
    // 将时间转换为字符串形式
    char time_string[100];                                                      // 用于存储时间的字符数组
    strftime(time_string, sizeof(time_string), "%Y-%m-%d-%H-%M-%S", localTime); // 格式化时间字符串time

    return {time_string};
}

ofstream CreateFile()
{
    string time_string = GetTimeString();
    ofstream out_file(string("./output/output") + time_string + ".txt", ios::app); // 打开文件 output.txt，如果不存在则创建

    return out_file;
}