/**
 * @file collisiondetection.cpp
 * @brief 碰撞检测函数集
 * @date 2019-12-12
 * 
 * 
 */
#include "collisiondetection.h"

using namespace HybridAStar;
//这个就是配置空间configurationspace的构造函数。
CollisionDetection::CollisionDetection() 
{
  this->grid = nullptr;
  //初始化——当车辆在一个位置，以某一个姿态时，需要检查哪些grid的碰撞索引
  int start_x = 0-Constants::width/2*100;//-30
  int end_x = Constants::width/2*100;//30
  int start_y = 0-Constants::length/2*100;//-40
  int end_y = Constants::length/2*100;//40
  //单位是cm
  collision_map_check.clear();
  for(int dyaw = 0;dyaw<=355;dyaw = dyaw+5)
  {//72种离散角度
      std::vector<std::pair<double,double>> thismask;
      thismask.clear(); 
      for(int dx = start_x;dx<=end_x;dx = dx+5)
      for(int dy = start_y;dy<=end_y;dy = dy+5)    
      {
          double w_x = std::cos(double(dyaw))*double(dx)+std::sin(double(dyaw))*double(dy);
          double w_y = 0-std::sin(double(dyaw))*double(dx)+std::cos(double(dyaw))*double(dy);
          thismask.push_back(std::pair<double,double>(w_x,w_y));
      }
      collision_map_check.push_back(thismask);
  }
}

//传入的单位是grid,判断的单位也都是grid——最简单的判断
bool CollisionDetection::configurationMe(float x, float y, float t)
{
  int grid_x = x;
  int grid_y = y;
  //传入的单位是grid
  for(int grid_xx = grid_x-car_dig/2;grid_xx <= grid_x+car_dig/2;grid_xx++)
  for(int grid_yy = grid_y-car_dig/2;grid_yy <= grid_y+car_dig/2;grid_yy++)
  {
  if (grid_xx >= 0 && grid_xx <int(grid->info.width)  && grid_yy >= 0 && grid_yy < int(grid->info.height)) 
  {//如果在地图范围内
  if (grid->data[grid_yy * grid->info.width + grid_xx]) 
    return false;   //有障碍物，则不可通行
  else continue;//无障碍物，检查下一个
  }
  else    return false;//如果在地图范围外，不可通行
  }
  return true;
}
//传入的点单位是grid，在世界坐标系下——更精细的判断
bool CollisionDetection::configurationMe1(float x, float y, float t)
{
    double now_x=x*Constants::cellSize*100; 
    double now_y=y*Constants::cellSize*100;//单位从grid转换为m再转换到cm
    int id_yaw = int(t/5.0);
    //世界坐标下的车原点——厘米
    std::vector<std::pair<double,double>> thismask;
    thismask = collision_map_check[id_yaw];//存放的是当车辆与世界坐标系原点重合但是旋转t角度的时候，需要检查的坐标在世界坐标系下的表示，单位是厘米
    for(int i =0;i<int(thismask.size());i++)
    {
        double shift_x =  now_x+thismask[i].first; //车辆覆盖的离散点，单位是厘米，在世界坐标系下。
        double shift_y =  now_y+thismask[i].second; 
        int grid_x = int(shift_x/100.0/Constants::cellSize);//cm -> m  ->grid
        int grid_y = int(shift_y/100.0/Constants::cellSize);
        if (grid_x >= 0 && grid_x <int(grid->info.width)  && grid_y >= 0 && grid_y < int(grid->info.height)) 
        {//如果在地图范围内
            if (grid->data[grid_y * grid->info.width + grid_x]) 
              return false;   //有障碍物，则不可通行
            else continue;//无障碍物，检查下一个
        }
        else return false;//地图范围外不可通行
    }
    return true;//每一个都通过检查，可以通行。
}
