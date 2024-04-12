#include "head/function.h"

// 检查坐标(x, y)是否为机器人能走的合法块 xxxxxx01
bool IsLandValid(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00000011) == 0b00000001);
}

// 检查坐标(x, y)是否为船舶的合法块 1xxxxxxx
bool IsOceanValid(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b10000000) == 0b10000000);
}

// 判断机器人位置是否在泊位内 xxx1xxxx，如果在返回港口编号，否则返回-1
int IsOnBerth(int x, int y)
{
    if ((x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00010000) == 0b00010000))
    {
        return int(World[x][y] >> 8);
    }
    else
    {
        return -1;
    }
}

// 判断船位置是否在泊位停靠范围内 xxx1xxxx xx1xxxxx，如果在返回港口编号，否则返回-1
int IsInBerthRange(int x, int y)
{
    if ((x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00110000) != 0))
    {
        return World[x][y] >> 8;
    }
    else
    {
        return -1;
    }
}

// 判断机器人位置是否在主干道上 xxxxx1xx
bool IsOnMainRoad(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b00000100) == 0b00000100);
}

// 判断船是否在主航道上 x1xxxxxx
bool IsOnMainChannel(int x, int y)
{
    return (x >= 0) && (x < N) && (y >= 0) && (y < N) && ((World[x][y] & 0b01000000) == 0b01000000);
}

// 判断机器人是否在商品处,如果在则返回物品ID，否则返回0
int IsOnGoods(int x, int y)
{
    return WorldGoods[x][y];
}

// 判断船的当前状态每个位置是否合法以及是否在主航道上(0表示不合法，1表示合法，2表示合法并且有部分在主航道上)
int JudgeBoatState(int x, int y, int dir)
{
    bool is_on_main = false;
    for (int i = 0; i < 6; i++)
    { // 对0-5号位置检查
        int nx = x + DX_BOAT[dir][i];
        int ny = y + DY_BOAT[dir][i];
        if (IsOceanValid(nx, ny))
        {
            if (IsOnMainChannel(nx, ny))
            {
                is_on_main = true;
            }
        }
        else
        {
            return 0;
        }
    }
    return is_on_main ? 2 : 1;
}

// 判断两艘船状态是否会碰 会返回true
bool JudgeBoatCollision(int xa, int ya, int dira, int xb, int yb, int dirb)
{
    // 两艘船的左上角坐标
    int lt_xa = xa + DX_BOAT[dira][BOAT_LEFT_TOP[dira]];
    int lt_ya = ya + DY_BOAT[dira][BOAT_LEFT_TOP[dira]];
    int lt_xb = xb + DX_BOAT[dirb][BOAT_LEFT_TOP[dirb]];
    int lt_yb = yb + DY_BOAT[dirb][BOAT_LEFT_TOP[dirb]];
    // 两艘船的右下角坐标
    int rd_xa = xa + DX_BOAT[dira][BOAT_RIGHT_DOWN[dira]];
    int rd_ya = ya + DY_BOAT[dira][BOAT_RIGHT_DOWN[dira]];
    int rd_xb = xb + DX_BOAT[dirb][BOAT_RIGHT_DOWN[dirb]];
    int rd_yb = yb + DY_BOAT[dirb][BOAT_RIGHT_DOWN[dirb]];
    // 得到两艘船重叠位置的边界坐标
    int start_x = max(lt_xa, lt_xb);
    int start_y = max(lt_ya, lt_yb);
    int end_x = min(rd_xa, rd_xb);
    int end_y = min(rd_ya, rd_yb);
    // 遍历重叠部分
    if (start_x <= end_x && start_y <= end_y)
    {
        for (int x = start_x; x <= end_x; x++)
        {
            for (int y = start_y; y <= end_y; y++)
            { // 如果该位置不在主航道上
                if (!IsOnMainChannel(x, y))
                    return true;
            }
        }
    }
    return false;
}

// 判断船购买点是否不会和其他船碰（购买的船要加入其位置）
bool JudgeBoatBuyingValid(int buying_id)
{
    for (int bi = 0; bi < BoatNum; bi++)
    {
        if (JudgeBoatCollision(BoatBuyings[buying_id].x, BoatBuyings[buying_id].y, 0, Boats[bi].x, Boats[bi].y, Boats[bi].dir))
        {
            return false;
        }
    }
    return true;
}
