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
#include <map>
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

//小结:文件测试了点云平面拟合的速度,将一帧的深度图像点云每一个点都计算出octomap的格子,然后利用每个格子中所有点计算出法向量.
//没有降采样的时间大概是200ms,降采样以后大约120ms.

bool fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes)
{
  try 
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;//平面拟合
    seg.setOptimizeCoefficients(true);
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
 
struct CompareKey
{
	bool operator()(octomap::OcTreeKey* a, octomap::OcTreeKey* b)const //作为key得告诉map如何比较
	{
    if(a->k[0]<b->k[0])return true;
    else if((a->k[0]==b->k[0])&&(a->k[1]<b->k[1]))
      return true;
    else if((a->k[0]==b->k[0])&&(a->k[1]==b->k[1])&&(a->k[2]<b->k[2]))
      return true;
    return false;
	}
};

//这个文件用于测试pcl中的滤波器函数
//计算深度时间70ms————考虑到地形因素，不能在这一步进行降采样。
//滤波时间47ms
//存放到oct时间32ms
//平面拟合时间10ms受前面影响，不做优化
int main (int argc, char** argv)
{

  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<sensor_msgs::PointCloud2>("linepoint",2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org1(new pcl::PointCloud<pcl::PointXYZ>);  
  int i =0;
  if (pcl::io::loadPCDFile("/home/r/Mysoftware/qhull_test/img_pcd.pcd", *cloud_org1) == -1) 
  {
      PCL_ERROR("Couldn't read file  \n");
      return (-1);
  }
  std::cout<<"The size "<<cloud_org1->size()<<std::endl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  //一个点找一个oct,数据结构：一个oct对应多个点。
  octomap::OcTree* m_octree;
  m_octree = new octomap::OcTree(0.25);//?
  m_octree->setProbHit(0.7);
  m_octree->setProbMiss(0.4);
  m_octree->setClampingThresMin(0.12);
  m_octree->setClampingThresMax(0.97);  
  std::vector<octomap::OcTreeKey*> tvk;
  std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr,CompareKey> thismap;
  int ct=0;

  double st_time = ros::Time().now().toSec();
  pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_org1);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.01, 0.01, 0.01);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	sor.filter(*cloud_org1);//提速非常明显。
  double st_timel = ros::Time().now().toSec();

  for(int i =0;i<cloud_org1->size();i++)
  {
    octomap::OcTreeKey* thisk(new octomap::OcTreeKey);
    *thisk=m_octree->coordToKey(cloud_org1->points[i].x,cloud_org1->points[i].y,cloud_org1->points[i].z);    
    std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator thisit = thismap.find(thisk);
    if(thisit!=thismap.end())
    {
      thisit->second->points.push_back(cloud_org1->points[i]);
    }
    else
    {
      tvk.push_back(thisk);
      pcl::PointCloud<PointXYZ>::Ptr pvector(new pcl::PointCloud<PointXYZ>);
      pvector->points.push_back(cloud_org1->points[i]);
      thismap[thisk]=pvector;      
      std::cout<<"new"<<thisk->k[0]<<" "<<thisk->k[1]<<" "<<thisk->k[2]<<std::endl;
    }
  }
  // return 0;
  //有if else执行90ms,没有执行30ms   
  double st_time1 = ros::Time().now().toSec();
  pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//法向量计算结果
  pcl::PointCloud<pcl::PointXYZ>::Ptr center_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::Normal nt;
  pcl::PointXYZ pppp;octomap::point3d tsssp; 
  for(map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  {        
       if(it->second->points.size()<120)continue;
        fit_plane_to_cloud(plane_model,it->second,0.2);
        nt.normal_x = plane_model->values[0];
        nt.normal_y = plane_model->values[1]; 
        nt.normal_z = plane_model->values[2]; 
        normals->points.push_back(nt);
        tsssp = m_octree->keyToCoord(*(it->first));
        pppp.x=tsssp.x();        pppp.y=tsssp.y();        pppp.z=tsssp.z();
        // pppp=it->second->points[40];
        center_plane->points.push_back(pppp);
        std::cout<<"This cloud size is "<<it->second->points.size()<<std::endl;
        ct++;
  }  
  std::cout<<"we have plane"<<ct<<std::endl;
  //60ms,完整一帧深度图像
  double st_time2 = ros::Time().now().toSec();
  std::cout<<"Total plane "<<center_plane->points.size()<<std::endl;
  std::cout<<"Size"<<cloud_plane->points.size()<<"滤波时间"<<st_timel-st_time<<"存放到oct时间"<<st_time1-st_timel<<"平面拟合时间"<<st_time2-st_time1 <<std::endl;
  //总共时间150ms
  // std::map<>
  // for(int u=0;u<=719;u=u++)
  // {
  //   for(int v=0;v<=1279;v=v++)
  //   {
  //     pcl::PointXYZ TP;TP.x=1;TP.y=1;TP.z=1;
  //     pvector.push_back(TP);
  //   }
  // }
  //将深度图像转换为点云图像，时间在70ms左右
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); //创建视窗对象，定义标题栏名称“3D Viewer”
	viewer->addPointCloud<pcl::PointXYZ>(center_plane, "original_cloud");	//将点云添加到视窗对象中，并定义一个唯一的ID“original_cloud”
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "original_cloud"); //点云附色，三个字段，每个字段范围0-1
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_plane, normals, 1, 0.6, "normals");	//每十个点显示一个法线，长度为0.05
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);

}

 