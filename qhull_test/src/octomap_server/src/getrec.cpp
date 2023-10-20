#include<ros/ros.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/impl/pcl_base.hpp>
#include<iostream>
#include<ctime>
#include<cmath>
#include<ros/time.h>
#include <thread>
#include <Eigen/Core>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <eigen3/Eigen/Dense>
#include<sensor_msgs/Image.h>
#include<gazebo_msgs/GetModelState.h>
#include<tf/tf.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include<utility>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::chrono_literals;
float q1_x=  6.2682;
float q1_y=  4.9527;
float q3_x=  0.6516;
float q3_y=  0.6915;

float q2_x=  6.2682;
float q2_y=  -4.9527;
float q4_x=  0.6516;
float q4_y=  -0.6915;

bool checky(float outx,float outy)
{
float line1y = ((q3_y-q1_y)*outx-(q3_y-q1_y)*q1_x+(q3_x-q1_x)*q1_y)/(q3_x-q1_x);
float line2y = ((q4_y-q2_y)*outx-(q4_y-q2_y)*q2_x+(q4_x-q2_x)*q2_y)/(q4_x-q2_x);
if((line1y>=outy)  &&  (line2y<=outy))return true;
return false;
}
void GetrecFronQ()
{
  std::vector<pair<float,float>> ss ={{1,2},{1,2}};
  std::vector<pair<float,float>> rec;
  float bbx=0.25;
  for(float x=0.6500;x<=6.2700;)
  {
    for(float y=-4.9500;y<=4.9600;)
    {
      if(checky(x,y)&&checky(x+bbx,y)&&checky(x,y+bbx)&&checky(x+bbx,y+bbx))
      {
        rec.push_back(std::make_pair(x,y));
        y=y+bbx;
      }
        y=y+0.01;
    }
        x=x+bbx;
  }
  std::cout<<rec.size()<<std::endl;
}


//这个文件用于测试pcl中的滤波器函数
int main (int argc, char** argv)
{
    // Eigen::Matrix3d K;    
    // Eigen::Matrix3d K_inv1;
    // K << 695.995117, 0.0, 640.0, 0.0, 695.995117, 360.0, 0.0, 0.0, 1.0;
    // K_inv1 = K.inverse();
    // std::cout<<K_inv1<<std::endl;
  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  GetrecFronQ();
  ROS_ASSERT(0);
}

 