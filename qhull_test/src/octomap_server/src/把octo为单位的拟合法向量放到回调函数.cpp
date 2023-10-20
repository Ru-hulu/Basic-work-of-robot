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
#include "surfeltree.hpp"
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::chrono_literals;

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

typedef union USHORT_UNION {
  ushort num;
  char c[2];
} ushort_union;

  inline ushort Littlesw(char c0, char c1) 
  {
    ushort_union tmp;
    tmp.c[0] = c0;
    tmp.c[1] = c1;
    return tmp.num;
  }

class Teclas
{
  public:
  ros::NodeHandle nh;
  ros::Subscriber ims;
  ros::ServiceClient states_client;

  octomap::OcTree* m_octree;
  Teclas(const ros::NodeHandle &nh_);
  bool fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes);  
  float K_inv[9] = {0.00143679,0,-0.919547,0,0.00143679,-0.517245,0,0,1};  
  float camera_depth=8;  
  void insertCloudCallback(const sensor_msgs::ImageConstPtr & msg);  
  //点云转换的时间大概是70ms
};

Teclas::Teclas(const ros::NodeHandle &nh_):nh(nh_)
{
ims=nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw",5,&Teclas::insertCloudCallback,this);
states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  m_octree = new octomap::OcTree(0.25);//?
  m_octree->setProbHit(0.7);
  m_octree->setProbMiss(0.4);
  m_octree->setClampingThresMin(0.12);
  m_octree->setClampingThresMax(0.97);  
};
void Teclas::insertCloudCallback(const sensor_msgs::ImageConstPtr & msg)
{
  double t0=ros::Time::now().toSec();
  std::cout<<"IO"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclimg(new(pcl::PointCloud<pcl::PointXYZ>));
  double st_time = ros::Time().now().toSec();
  int u =0; int v=0;
  int wi = int(msg->width);
  int he = int(msg->height);
  int d = wi*he;
  float z_c = 0;
  pcl::PointXYZ thisp;
  float px=0;  float py=0;  float pz=0;
  float horizonAngle = 0; float now_dis=0;
  Eigen::Matrix4f Twi;  Eigen::Matrix4f Tic;
  Eigen::Matrix4f sensorToWorld;//这里拿到的就是map->cloud-head变换 
  gazebo_msgs::GetModelState rq;
  rq.request.model_name="scout";
  rq.request.relative_entity_name="map";
  while(!states_client.call(rq))
  std::cout<<"Waiting for gzbo";
  float posx=rq.response.pose.position.x;
  float posy=rq.response.pose.position.y;
  float posz=rq.response.pose.position.z;
  float yaw_h = tf::getYaw(rq.response.pose.orientation);//这里是车的yaw角
  double xita =0.6 ;
  Twi<<std::cos(yaw_h),-std::sin(yaw_h),0,posx,
              std::sin(yaw_h),std::cos(yaw_h),0,posy,
              0,0,1,0.45,
              0,0,0,1;
  Tic<<0,-sin(xita),cos(xita),0.25,
      -1,         0,        0,  0,
       0,-cos(xita),-sin(xita),0.5,
       0,         0,         0,  1;

  double t1=ros::Time::now().toSec();
  for(v=50;v<he;v++)
  for(u=0;u<wi;u=u+1)
  {
    int idx = u+v*msg->width;
    ushort tmp = Littlesw(msg->data[idx * 2], msg->data[idx * 2 + 1]);
    z_c = float(tmp)/1000;    
    if(((z_c>0.3)&&(z_c<=camera_depth)))
    {
      thisp.x =  z_c*(K_inv[0]*u+K_inv[1]*v+K_inv[2]*1);   
      thisp.y =  z_c*(K_inv[3]*u+K_inv[4]*v+K_inv[5]*1);   
      thisp.z =  z_c;   
      pclimg->points.push_back(thisp);
    }
  }
  double t2=ros::Time::now().toSec();
  std::cout<<"深度转换时间"<<t2-t1<<std::endl;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(pclimg);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.01, 0.01, 0.01);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	sor.filter(*pclimg);//提速非常明显。
  pcl::transformPointCloud(*pclimg, *pclimg, Twi*Tic);//转到世界坐标系下
  double t3=ros::Time::now().toSec();
  std::cout<<"滤波时间"<<t3-t2<<std::endl;

  std::vector<octomap::OcTreeKey*> tvk;
  std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr,CompareKey> thismap;
  for(int i =0;i<pclimg->size();i++)
  {
    octomap::OcTreeKey* thisk(new octomap::OcTreeKey);
    *thisk=m_octree->coordToKey(pclimg->points[i].x,pclimg->points[i].y,pclimg->points[i].z);    
    std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator thisit = thismap.find(thisk);
    if(thisit!=thismap.end())
    {
      thisit->second->points.push_back(pclimg->points[i]);
    }
    else
    {
      tvk.push_back(thisk);
      pcl::PointCloud<PointXYZ>::Ptr pvector(new pcl::PointCloud<PointXYZ>);
      pvector->points.push_back(pclimg->points[i]);
      thismap[thisk]=pvector;      
    }
  }
  double t4=ros::Time::now().toSec();
  std::cout<<"oct索引时间"<<t4-t3<<std::endl;
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
        center_plane->points.push_back(pppp);
        // std::cout<<"This cloud size is "<<it->second->points.size()<<std::endl;
  }  
  double t5=ros::Time::now().toSec();
  std::cout<<"平面拟合时间"<<t5-t4<<std::endl;
  std::cout<<"总时间"<<t5-t0<<std::endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); //创建视窗对象，定义标题栏名称“3D Viewer”
	viewer->addPointCloud<pcl::PointXYZ>(center_plane, "original_cloud");	//将点云添加到视窗对象中，并定义一个唯一的ID“original_cloud”
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "original_cloud"); //点云附色，三个字段，每个字段范围0-1
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_plane, normals, 1, 0.6, "normals");	//每十个点显示一个法线，长度为0.05
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  ROS_ASSERT(0);
}
//这个文件用于测试pcl中的滤波器函数
bool Teclas::fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes)
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

int main (int argc, char** argv)
{

  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  // Teclas T(nh);
  // m_octree = new octomap::OcTree(0.25);//?
  // m_octree->setProbHit(0.7);
  // m_octree->setProbMiss(0.4);
  // m_octree->setClampingThresMin(0.12);
  // m_octree->setClampingThresMax(0.97);  

  f_map::FrontierOcTree* thist;
  thist = new f_map::FrontierOcTree(0.25);
  
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return (0);

}

 