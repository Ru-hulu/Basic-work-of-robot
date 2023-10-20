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
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::chrono_literals;
//这个文件用于测试pcl中的滤波器函数
int main (int argc, char** argv)
{
  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orgf(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ THIP;
  std::cout<<"T"<<std::endl;
  int i =0;
  for(float x_s=-0.5;x_s<=0.5; x_s = x_s+0.01)
  {
    for(float y_s=-0.5;y_s<=0.5; y_s = y_s+0.01)
    {
        THIP.x=x_s;    THIP.y=y_s;    THIP.z=0;
        cloud_org->points.push_back(THIP);
        THIP.z=1;
        cloud_org1->points.push_back(THIP);
        i++;
        std::cout<<x_s<<" "<<y_s<<" "<<0<<" "<<"T"<<std::endl;
    }
  }
  pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_org);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.1, 0.1, 0.1);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
  // sor.setMinimumPointsNumberPerVoxel(10);//设置每个体素中的点的个数
	sor.filter(*cloud_filter);
  *cloud_filter += *cloud_org1;
  
      // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
      // sor1.setInputCloud(inputCloud);
      // sor1.setMeanK(int_param);
      // sor1.setStddevMulThresh(double_param);
      // sor1.filter(*filteredCloud);

  std::cout<<"T"<<std::endl;
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handsur(cloud_org,0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_filter, handsur, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();



  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);
}

 