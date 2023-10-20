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
#include <set>
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
//自定义点云的情况下,回调函数的执行时间如下.
// 深度转换时间0.055
// 滤波时间0.038
// oct索引时间0.026
// 平面拟合时间0.007
// 总时间0.127
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
  f_map::FrontierOcTree* m_octree;
  std::set<f_map::FrontierOcTreeNode *> frontier;
  std::set<f_map::FrontierOcTreeNode *> newnode_set;//存放新加入所有节点
  std::set<f_map::FrontierOcTreeNode *> chenode_set;//存放活跃区已经是frontier的节点
  Teclas(const ros::NodeHandle &nh_);
  float fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes);  
  float K_inv[9] = {0.00143679,0,-0.919547,0,0.00143679,-0.517245,0,0,1};  
  float camera_depth=8;  
  float xita_thred=20.0/57.2957;//如果超过20度，则认为小平面不可穿行。
  void insertCloudCallback(const sensor_msgs::ImageConstPtr & msg);  
  //点云转换的时间大概是70ms
  int cameracct=0;int temc=0;
};

Teclas::Teclas(const ros::NodeHandle &nh_):nh(nh_)
{
ims=nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw",5,&Teclas::insertCloudCallback,this);
states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
m_octree = new f_map::FrontierOcTree(0.25);
};
void Teclas::insertCloudCallback(const sensor_msgs::ImageConstPtr & msg)
{
  if(cameracct%50!=0){cameracct++;return;}
  temc++;
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
  //所有点进行分类
  for(int i =0;i<pclimg->size();i++)
  {
    octomap::OcTreeKey* thisk(new octomap::OcTreeKey);
    *thisk=m_octree->coordToKey(pclimg->points[i].x,pclimg->points[i].y,pclimg->points[i].z);    
    f_map::FrontierOcTreeNode * thisn=m_octree->search(*thisk);//先在octomap中找到这个节点。
    if(thisn) 
    { 
      i=i+1;
      continue;
    }//加速脱离已知区域
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
  pclimg.reset(new pcl::PointCloud<PointXYZ>);//释放内存
  double t4=ros::Time::now().toSec();
  std::cout<<"oct索引时间"<<t4-t3<<std::endl;
  pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  float abs_dis=0;
  for(map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  {        
    if(it->second->points.size()<120)continue;
    abs_dis = fit_plane_to_cloud(plane_model,it->second,0.065);
    m_octree->updateNode(*(it->first),true); //这里把未知的状态标记为已知       
    if(abs_dis<0)
    m_octree->set_node_info(*(it->first),-1,plane_model->values[0],plane_model->values[1],plane_model->values[2],plane_model->values[3],-1);
    else//这里先把frontier和可穿行区域不做区分。
    {
    m_octree->set_node_info(*(it->first),1,plane_model->values[0],plane_model->values[1],plane_model->values[2],plane_model->values[3],abs_dis);    
    std::cout<<abs_dis<<std::endl;
    }
    delete it->first;     it->second.reset(new pcl::PointCloud<PointXYZ>);  //释放内存
  }  
  double t5=ros::Time::now().toSec();float ttt=0;
  std::cout<<"平面拟合时间"<<t5-t4<<std::endl;
  std::cout<<"总时间"<<t5-t0<<std::endl;
    ROS_ASSERT(0);
  if(temc==200)
  {  
  pcl::PointXYZ pppp;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//法向量计算结果
  pcl::PointCloud<pcl::PointXYZ>::Ptr center_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::Normal nt;
  for(f_map::FrontierOcTree::leaf_iterator it = m_octree->begin_leafs(m_octree->getTreeDepth()), end = m_octree->end_leafs();it != end; ++it)
  {
    if(it->get_state()<0)continue; 
    it->get_plane(nt.normal_x,nt.normal_y,nt.normal_z,ttt);
    normals->points.push_back(nt);
    pppp.x=it.getX();      pppp.y=it.getY();      pppp.z=it.getZ();
    center_plane->points.push_back(pppp);
  }
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

  // float aa=0;  float bb=0;  float cc=0;  float dd=0; float avd=0;  int state=0;  
  // int cccct=0;
  // for(map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  // {
  //   cccct++;
  //   f_map::FrontierOcTreeNode * thisn=m_octree->search(*(it->first));
  //   if(thisn)
  //   {
  //   state=thisn->get_state();    
  //   thisn->get_plane(aa,bb,cc,dd);    
  //   avd=thisn->get_avd();    
  //   std::cout<<cccct<<" "<<aa<<" "<<bb<<" "<<cc<<" "<<dd<<" "<<state<<" "<<avd<<std::endl;      
  //   }
  //   // else 
  //   // std::cout<<"Non"<<std::endl;
  // }
}

//返回<0,表示平面不可穿行，返回>=0，表示散点到平面的距离绝对值之和。
float Teclas::fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes)
{
  float sqd = 0;
  float all_dis=0;
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
    float a=coefficients->values[0];
    float b=coefficients->values[1];
    float c=coefficients->values[2];
    float d=coefficients->values[3];
    sqd=std::sqrt(a*a+b*b+c*c);
    float xita = acos(c/sqd);//法向量与垂直z的夹角，相当平面与水平面的夹角。
    if(xita>xita_thred||(inliers->indices.size()/cloud->points.size()<0.7))
    {
      std::cout<<"Not"<<std::endl;
      return -1;//角度超过阈值，放弃。
    } 
    all_dis=0;
    for(int i=0;i<cloud->points.size();i++)
    {
      all_dis+=std::abs(a*cloud->points[i].x+
                        b*cloud->points[i].y+
                        c*cloud->points[i].z+
                        d);
    }
    all_dis = all_dis/cloud->points.size();
  }
  catch (...) 
  {
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    return -1;
  }
    return all_dis/sqd;
}

int main (int argc, char** argv)
{

  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  Teclas T(nh);
  // m_octree = new octomap::OcTree(0.25);//?
  // m_octree->setProbHit(0.7);
  // m_octree->setProbMiss(0.4);
  // m_octree->setClampingThresMin(0.12);
  // m_octree->setClampingThresMax(0.97);  
  // f_map::FrontierOcTree* thist;
  // thist = new f_map::FrontierOcTree(0.25);

  while(ros::ok())
  {
    ros::spinOnce();
  }
  return (0);

}

 