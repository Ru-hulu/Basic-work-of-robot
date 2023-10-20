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


#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::chrono_literals;

//小结:文件测试了点云平面拟合的速度,将一帧的深度图像进行拟合所需要的时间大概是50ms.大体可行.
  bool fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,const double dist_thes)
  {
    if (cloud->points.size() < 3) {
      coefficients->values.push_back(0.0);
      coefficients->values.push_back(0.0);
      coefficients->values.push_back(0.0);
      coefficients->values.push_back(0.0);
      return false;
    }
    try {
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;//平面拟合
      // Optional
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(dist_thes);
      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);
    } catch (...) {
      coefficients->values.push_back(0.0);
      coefficients->values.push_back(0.0);
      coefficients->values.push_back(0.0);
      coefficients->values.push_back(0.0);
      return false;
    }
    return true;
  }
 

//这个文件用于测试pcl中的滤波器函数
int main (int argc, char** argv)
{

  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<sensor_msgs::PointCloud2>("linepoint",2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_surcenter(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  int i =0;
  if (pcl::io::loadPCDFile("/home/r/Mysoftware/qhull_test/img_pcd.pcd", *cloud_org1) == -1) 
  {
      PCL_ERROR("Couldn't read file  \n");
      return (-1);
  }
  std::cout<<"The size "<<cloud_org1->size()<<std::endl;
  //平面拟合策略：
  //高度720，从20-720进行拟合
  //宽度1280，降采样因子2或者3，即427个点，这样可以保证在25cm*25cm的范围内，有40个点。
  for(int i=0;i<=20000;i=i+720)
  {
    std::cout<<"point data"<<i<<" "<< cloud_org1->points[i].x<<" "<< cloud_org1->points[i].y<<" "<< cloud_org1->points[i].z<<std::endl;
    cloud_filter->points.push_back(cloud_org1->points[i]);
  }


  //   sensor_msgs::PointCloud2 img_cloud;
  //   pcl::toROSMsg(*cloud_filter,img_cloud);
  //   img_cloud.header.stamp = ros::Time().now();
  //   img_cloud.header.frame_id = "map";
  //   while(ros::ok())
  //   {
  //     p.publish(img_cloud);
  //     ros::spinOnce();
  //   }
  // ROS_ASSERT(0);
  //这个点云先列（上下）后行
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//法向量计算结果
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr center_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::Normal nt;
  int idx =0;
        double st_time = ros::Time().now().toSec();
  //一个点找一个oct,数据结构：一个oct对应多个点。
  for(int u=0;u<=719;u=u+20)
  {
    for(int v=0;v<=1279;v=v+100)
    {
      if(u+20>719||v+100>1279) continue;
      cloud_plane->points.clear();
      for(int uu=u;uu<=u+19;uu++)
      {
        for(int vv=v;vv<=v+99;vv++)
        {
          idx = 720*vv+uu;
          cloud_plane->points.push_back(cloud_org1->points[idx]);
        }
      }
        fit_plane_to_cloud(plane_model,cloud_plane,0.2);
        nt.normal_x = plane_model->values[0];
        nt.normal_y = plane_model->values[1]; 
        nt.normal_z = plane_model->values[2]; 
        normals->points.push_back(nt);
        center_plane->points.push_back(cloud_org1->points[idx]);
    }
  }
  //将深度图像转换为点云图像，时间在70ms左右
  //2000个点的平面拟合432次的时间在50ms左右
  double st_time1 = ros::Time().now().toSec();
  std::cout<<"Size"<<cloud_plane->points.size()<<"Time "<<st_time1-st_time<<std::endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); //创建视窗对象，定义标题栏名称“3D Viewer”
	viewer->addPointCloud<pcl::PointXYZRGB>(center_plane, "original_cloud");	//将点云添加到视窗对象中，并定义一个唯一的ID“original_cloud”
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "original_cloud"); //点云附色，三个字段，每个字段范围0-1
  std::cout<<"Size"<<cloud_plane->points.size()<<"Time "<<st_time1-st_time<<std::endl;
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(center_plane, normals, 10, 0.6, "normals");	//每十个点显示一个法线，长度为0.05
  std::cout<<"Size"<<cloud_plane->points.size()<<"Time "<<st_time1-st_time<<std::endl;
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);

}

 