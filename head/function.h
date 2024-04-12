#ifndef CODECRAFTSDK_FUNCTION_H
#define CODECRAFTSDK_FUNCTION_H

#include "global.h"

// 海洋部分 10000000
// 主航道   11000000
// 靠泊区   11100000
// 泊位	    11010101
// 陆地部分 00000001
// 陆地障碍 00000011
// 主干道   00000101

// 00000001  '.' ： 空地
// 00000101  '>' ： 陆地主干道
// 10000000  '*' ： 海洋
// 11000000  '~' ： 海洋主航道
// 00000011  '#' ： 障碍
// 00000101  'R' ： 机器人购买地块，同时该地块也是主干道
// 11000000  'S' ： 船舶购买地块，同时该地块也是主航道
// 11010101  'B' ： 泊位
// 11100000  'K' ： 靠泊区
// 10000001  'C' ： 海陆立体交通地块
// 11000101  'c' ： 海陆立体交通地块，同时为主干道和主航道
// 11000000  'T' ： 交货点

bool IsLandValid(int x, int y);

bool IsOceanValid(int x, int y);

int IsOnBerth(int x, int y);

int IsInBerthRange(int x, int y);

bool IsOnMainRoad(int x, int y);

bool IsOnMainChannel(int x, int y);

int IsOnGoods(int x, int y);

int JudgeBoatState(int x, int y, int dir);

bool JudgeBoatCollision(int xa, int ya, int dira, int xb, int yb, int dirb);

bool JudgeBoatBuyingValid(int buying_id);

int LastMinBerth(int x, int y);

int PsbDirToBerth(int berth_id, int x, int y);

double CalculateGoodsValue(int goods_index, int step_num, int &to_berth_index, double rest_time_weight);

vector<int> GetRandomDirection();

void InitBerthInfo(int id);

void InitWorldInfo();

void InitToBerthBFS();

int CalculateArea(int buy_index);

void AllocateRobot();

void linkMaxBuyBoat();

void InitToDeliveryEstimateTimeDijkstra();

void InitToBerthEstimateTimeDijkstra();

double GetHValue(int x, int y, int dir, int action, int d_type, int d_id);

void PercolateUp(vector<shared_ptr<BoatStateNode>> &heap, shared_ptr<BoatStateNode> state_node);

int PositionToPositionAStar(int sx, int sy, int sdir, int d_type, int d_id);

void InitDeliveryToBerth();

void InitBuyingToBerth();

void InitBerthToBerth();

void InitRobotAndBoatBuy();

void Init();

void Input();

bool NoGoodsRobotsCompair(int ri, int rj);

bool GetGoodsRobotsCompair(int ri, int rj);

void RobotBFSToGoods(int ri, priority_queue<Road, vector<Road>, Road::Comparator> &roads_pq, mutex &roads_pq_mutex);

void RobotDispatchGreedy();

bool CrashAvoid(int ri);

void HedgeAvoid(int ri, int rj, bool (&is_collision_robot)[MAX_ROBOT_NUM]);

void RushPositionAvoid(int ri, int rj, bool (&is_collision_robot)[MAX_ROBOT_NUM]);

void AvoidCollision();

void PrintRobotsIns();

int FindBestBerthFromDelivery(int boat_id, int delivery_id);

int FindBestBerthOrGoFromBerth(int boat_id);

int FindBestBerthFromBuying(int boat_id, int boat_buying_id);

int BoatToPositionAStar(int bi, int d_type, int d_id);

void RunToDeliveryGun(int bi, int best_delivery);

void BoatDispatch();

void LoadGoods();

void BuyARobot(int robot_buying_index);

void BuyABoat(int boat_buying_index, int berth_index);

void BuyRobotsCxh();

void BuyBoatsCxh();

int BuyRobotsXmc();

int BuyBoatsXmc();

void Buy();

void BuyBoatsYzh();

void PrintMoney(ofstream &out_file);

void PrintRobotNum(ofstream &out_file);

void PrintBerthGoodsInfo(ofstream &out_file);

void PrintBoatInfo(ofstream &out_file);

void PrintWorldInfo(ofstream &out_file);

void PrintBoatCapacity(ofstream &out_file);

void PrintDeliveryToBerthTime(ofstream &out_file);

void PrintBuyingToBerthTime(ofstream &out_file);

void PrintNearestBuyingAndDelivery(ofstream &out_file);

void PrintBoatRouts(ofstream &out_file);

void PrintInitBuy(ofstream &out_file);

void PrintInitBuyBoatInfo(ofstream &out_file);

void Print(ofstream &out_file, int interval);

string GetTimeString();

ofstream CreateFile();


#endif //CODECRAFTSDK_FUNCTION_H
