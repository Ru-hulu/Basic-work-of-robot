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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <subspace.hpp>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Quaternion.h>
#include<tf/tf.h>
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


typedef union USHORT_UNION 
{
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
  ros::Publisher showp;
  ros::Publisher showMap;
  ros::ServiceClient states_client;
  f_map::FrontierOcTree* m_octree;
  std::set<octomap::OcTreeKey*,CompareKey> frontier;
  Teclas(const ros::NodeHandle &nh_);
  int fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes,float& precent_inline,float& varia);
  float K_inv[9] = {0.00143679,0,-0.919547,0,0.00143679,-0.517245,0,0,1};  
  float camera_depth=8;  
  float xita_thred=20.0/57.2957;//如果超过20度，则认为小平面不可穿行。
  float resolu=0.25;  float resolu_half=0.125;  
  vector<pcl::PointXYZ> cuboidpoint;
  void insertCloudCallback(const sensor_msgs::ImageConstPtr & msg);  
  void GetPlaneNeib(float vcx,float vcy,float vcz,float vpa,float vpb,float vpc,float vpd,vector<pcl::PointXYZ>*neicenter);
  void GetPlaneNeib(float vcx,float vcy,float vcz,vector<pcl::PointXYZ>*neicenter);
  void PublishFrontierMarker();
  void PublishPlane();
  void PublishPlane_USLESS();
  void Update_SubspaceFrontier(octomap::OcTreeKey* tk);
  //点云转换的时间大概是70ms
  int cameracct=0;int temc=0;
  sub_space::Subspace* subspace_array[8][8];//所有的subspace都存储在内
};

Teclas::Teclas(const ros::NodeHandle &nh_):nh(nh_)
{
ims=nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw",5,&Teclas::insertCloudCallback,this);
showp = nh.advertise<visualization_msgs::MarkerArray>("Frontier_Marker",5);
showMap = nh.advertise<geometry_msgs::PoseArray>("PlaneMap",2);
states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
m_octree = new f_map::FrontierOcTree(0.25);
cuboidpoint.clear();
for(int i=0;i<=7;i++)
for(int j=0;j<=7;j++)
{
  subspace_array[i][j]=new sub_space::Subspace;
  subspace_array[i][j]->state = sub_space::UNEXPLORED;
}//初始化
pcl::PointXYZ tp;
tp.x=-0.125;tp.y=-0.125;tp.z=0.125;
cuboidpoint.push_back(tp);//1
tp.x=-0.125;tp.y=0.125;tp.z=0.125;
cuboidpoint.push_back(tp);//2
tp.x=0.125;tp.y=0.125;tp.z=0.125;
cuboidpoint.push_back(tp);//3
tp.x=0.125;tp.y=-0.125;tp.z=0.125;
cuboidpoint.push_back(tp);//4
tp.x=-0.125;tp.y=-0.125;tp.z=-0.125;
cuboidpoint.push_back(tp);//5
tp.x=-0.125;tp.y=0.125;tp.z=-0.125;
cuboidpoint.push_back(tp);//6
tp.x=0.125;tp.y=0.125;tp.z=-0.125;
cuboidpoint.push_back(tp);//7
tp.x=0.125;tp.y=-0.125;tp.z=-0.125;
cuboidpoint.push_back(tp);//8
};
void Teclas::PublishPlane()
{
  visualization_msgs::MarkerArray PlaneMarker;//节点数据结构，用于可视化
  visualization_msgs::Marker marker;    //Ｍａｒｋｅｒ是一个结构体模板
  //设置frame ID 和 timstamp ，具体信息可以参考TF 教程
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  //为marker设置命名空间和ＩＤ，创建不同的ＩＤ
  //任何带有相同的命名空间和ＩＤ被发送，都会覆盖先前的那个marker
  marker.ns = "basic_shapes";
  //设置marker的类型。初始化是CUBE,　接着循环SPHERE,ARROW,CYLINDER
  marker.type = visualization_msgs::Marker::ARROW;
  //设置marker的操作：ADD, DELETE, and new in ROS indigo:3(DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  //设置marker的位姿。这是6个自由度，相对于header中特定的坐标系和时间戳
  //设定marker的大小----- 1x1x1 表示边长为１m
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
  marker.scale.x = 0.2;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  float pa=0;  float pb=0;  float pc=0;
  float p1a=0;  float p1b=0;  float p1c=0;
  float ttt;
//向量起点A，就是Pose的position.
//计算将向量[1,0,0]旋转到向量AB方向的四元数。
//target_v = AB/|AB| //单位向量
//r = [1,0,0] X target_v
//r = r/|r| //单位向量
//Theta = arccos(target_v*[1,0,0]) //旋转角
//quaternion = (r*sin(Theta/2),cos(Theta/2))
  float cr1=0;  float cr2=0;
  int attc=0;
  for(f_map::FrontierOcTree::leaf_iterator it = m_octree->begin_leafs(m_octree->getTreeDepth()), end = m_octree->end_leafs();it != end; ++it)
  {
    attc++;
    // if(it->get_state()<0)continue; 
    it->get_plane(pa,pb,pc,ttt);
    // pa = 1;    pb = 1;    pc = 1;
    octomap::point3d pppp =  it.getCoordinate();    
    marker.id = attc;
    marker.pose.position.x = pppp.x();
    marker.pose.position.y = pppp.y();
    // marker.pose.position.z = pppp.z();
    marker.pose.position.z = pa*pppp.x()+pb*pppp.y()+pc*pppp.z()+ttt;
    ttt = sqrt(pa*pa+pb*pb+pc*pc);
    pa/=ttt;    pb/=ttt;    pc/=ttt;//target_v
    p1a = 0;    p1b = 0-pc; p1c = pb;//[1 0 0 ]cross pa pb pc
    ttt = sqrt(p1a*p1a+p1b*p1b+p1c*p1c);
    p1a =0; p1b = p1b/ttt; p1c = p1c / ttt;//normalize r
    ttt = pa*1+pb*0+pc*0;
    ttt = std::acos(ttt);//Theta
    geometry_msgs::Quaternion q4;
    q4.x = p1a*sin(ttt/2);    q4.y = p1b*sin(ttt/2);    q4.z = p1c*sin(ttt/2);
    q4.w = cos(ttt/2);
    marker.pose.orientation.x = q4.x;
    marker.pose.orientation.y = q4.y;
    marker.pose.orientation.z = q4.z;
    marker.pose.orientation.w = q4.w;
    cr1 = 1-it->get_per();
    cr2 = (it->get_avd())/0.25;
    if(it->get_state()<0)
    {
    marker.color.r = 1.0;
    marker.color.g = 0;
    }
    else
    {
    marker.color.r = 2*(std::max(cr1*10,cr2*10));
    marker.color.g = 1-marker.color.r;
    }
    PlaneMarker.markers.push_back(marker);
  }
    showp.publish(PlaneMarker);
}
void Teclas::PublishPlane_USLESS()
{
  float pa=0;  float pb=0;  float pc=0;
  float p1a=0;  float p1b=0;  float p1c=0;
  float ttt;
  geometry_msgs::PoseArray PosA;//节点数据结构，用于可视化
  geometry_msgs::Pose this_pos;
  PosA.poses.clear();
  PosA.header.frame_id = "map";
//向量起点A，就是Pose的position.
//计算将向量[1,0,0]旋转到向量AB方向的四元数。
//target_v = AB/|AB| //单位向量
//r = [1,0,0] X target_v
//r = r/|r| //单位向量
//Theta = arccos(target_v*[1,0,0]) //旋转角
//quaternion = (r*sin(Theta/2),cos(Theta/2))
  for(f_map::FrontierOcTree::leaf_iterator it = m_octree->begin_leafs(m_octree->getTreeDepth()), end = m_octree->end_leafs();it != end; ++it)
  {
    // if(it->get_state()<0)continue; 
    it->get_plane(pa,pb,pc,ttt);
    // pa = 1;    pb = 1;    pc = 1;
    octomap::point3d pppp =  it.getCoordinate();    
    this_pos.position.x=pppp.x();
    this_pos.position.y=pppp.y();
    this_pos.position.z=pppp.z();
    // this_pos.position.z=pa*pppp.x()+pb*pppp.y()+pc*pppp.z()+ttt;
    ttt = sqrt(pa*pa+pb*pb+pc*pc);
    pa/=ttt;    pb/=ttt;    pc/=ttt;//target_v
    p1a = 0;    p1b = 0-pc; p1c = pb;//[1 0 0 ]cross pa pb pc
    ttt = sqrt(p1a*p1a+p1b*p1b+p1c*p1c);
    p1a =0; p1b = p1b/ttt; p1c = p1c / ttt;//normalize r
    ttt = pa*1+pb*0+pc*0;
    ttt = std::acos(ttt);//Theta
    geometry_msgs::Quaternion q4;
    q4.x = p1a*sin(ttt/2);    q4.y = p1b*sin(ttt/2);    q4.z = p1c*sin(ttt/2);
    q4.w = cos(ttt/2);
    this_pos.orientation.x = q4.x;
    this_pos.orientation.y = q4.y;
    this_pos.orientation.z = q4.z;
    this_pos.orientation.w = q4.w;
    PosA.poses.push_back(this_pos);
  }
  showMap.publish(PosA);
}

void Teclas::PublishFrontierMarker()
{
  visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
  visualization_msgs::Marker bbox_marker;
  pathNodes.markers.clear();
  bbox_marker.header.frame_id = "map";
  bbox_marker.frame_locked = true;
  bbox_marker.ns = "basic_shapes";
  bbox_marker.color.r = 1.0f;
  bbox_marker.color.g = 0.0f;
  bbox_marker.color.b = 0.0f;
  bbox_marker.color.a = 1.0;
  bbox_marker.scale.x = 0.25;
  bbox_marker.scale.y = 0.25;
  bbox_marker.scale.z = 0.25;
  int ct=0;
  for(auto i = frontier.begin();i!=frontier.end();i++)
  {
    octomap::point3d otpp = m_octree->keyToCoord(**i);
    bbox_marker.id = ct;
    bbox_marker.pose.position.x=otpp.x();
    bbox_marker.pose.position.y=otpp.y();
    bbox_marker.pose.position.z=otpp.z();
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    // bbox_marker.action = visualization_msgs::Marker::ADD;
    pathNodes.markers.push_back(bbox_marker);
    ct++;
  }
  showp.publish(pathNodes);
}

void Teclas::GetPlaneNeib(float vcx,float vcy,float vcz,float vpa,float vpb,float vpc,float vpd,vector<pcl::PointXYZ>*neicenter)
{
  int flag_plane[6]={0,0,0,0,0,0};
  int flag_point[8]={0,0,0,0,0,0,0,0};
  for(int i=0;i<=7;i++)
  {
    float dd=vpa*(vcx+cuboidpoint[i].x)+vpb*(vcy+cuboidpoint[i].y)+vpc*(vcz+cuboidpoint[i].z)+vpd;
    if(dd>0)flag_point[i]=1;//依次检测正方体的八个点在平面的上方还是下方。
  }
  flag_plane[0]=flag_point[0]+flag_point[1]+flag_point[2]+flag_point[3];//上
  flag_plane[1]=flag_point[4]+flag_point[5]+flag_point[6]+flag_point[7];//下
  flag_plane[2]=flag_point[0]+flag_point[1]+flag_point[4]+flag_point[5];//左
  flag_plane[3]=flag_point[2]+flag_point[3]+flag_point[6]+flag_point[7];//右
  flag_plane[4]=flag_point[1]+flag_point[2]+flag_point[5]+flag_point[6];//前
  flag_plane[5]=flag_point[0]+flag_point[3]+flag_point[4]+flag_point[7];//后
  //只有四个点全部在上方，或者全部在下方，才说明拟合的平面没有穿过正方体的一个面
  pcl::PointXYZ tp;
  //==0说明全部在下方，==4说明全部在上方
  if((flag_plane[0]!=0)&&(flag_plane[0]!=4))
  {
    tp.x=vcx;    tp.y=vcy;    tp.z=vcz+resolu;
    neicenter->push_back(tp);    
  }//上

  if((flag_plane[1]!=0)&&(flag_plane[1]!=4))
  {
    tp.x=vcx;    tp.y=vcy;    tp.z=vcz-resolu;
    neicenter->push_back(tp);    
  }//下

  if((flag_plane[2]!=0)&&(flag_plane[2]!=4))
  {
    tp.x=vcx-resolu;    tp.y=vcy;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//左

  if((flag_plane[3]!=0)&&(flag_plane[3]!=4))
  {
    tp.x=vcx+resolu;    tp.y=vcy;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//左

  if((flag_plane[4]!=0)&&(flag_plane[4]!=4))
  {
    tp.x=vcx;    tp.y=vcy+resolu;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//前

  if((flag_plane[5]!=0)&&(flag_plane[5]!=4))
  {
    tp.x=vcx;    tp.y=vcy-resolu;    tp.z=vcz;
    neicenter->push_back(tp);    
  }//后
}


void Teclas::GetPlaneNeib(float vcx,float vcy,float vcz,vector<pcl::PointXYZ>*neicenter)
{
    pcl::PointXYZ tp;
    for(int ge_x=-1;ge_x<=1;ge_x=ge_x+2)
    {
     tp.x=vcx+resolu*float(ge_x);    
     tp.y=vcy;    
     tp.z=vcz;
     neicenter->push_back(tp);    
    }
    for(int ge_y=-1;ge_y<=1;ge_y=ge_y+2)
    {
     tp.x=vcx;    
     tp.y=vcy+resolu*float(ge_y);    
     tp.z=vcz;
     neicenter->push_back(tp);    
    }
    for(int ge_z=-1;ge_z<=1;ge_z=ge_z+2)
    {
     tp.x=vcx;
     tp.y=vcy;    
     tp.z=vcz+resolu*float(ge_z);  
     neicenter->push_back(tp);    
    }
}



// 正常一帧下，时间开销为
// 深度转换时间0.011
// 滤波时间0.026
// oct索引时间0.007
// 平面拟合时间0
// Frontier动态更新时间0
// 总时间0.045
// 深度信息是50帧，一秒拟合两次，每次用2帧

void Teclas::insertCloudCallback(const sensor_msgs::ImageConstPtr & msg)
{
  if(cameracct%12!=0){cameracct++;return;}
  cameracct = 1;
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
  // float yaw_h = tf::getYaw(rq.response.pose.orientation);//这里是车的yaw角
  tf::Quaternion qw1={rq.response.pose.orientation.x,rq.response.pose.orientation.y,rq.response.pose.orientation.z,rq.response.pose.orientation.w};
  tf::Matrix3x3 matrixw1;
  matrixw1.setRotation(qw1);
  double xita =0.6 ;
  Twi<<matrixw1[0][0],matrixw1[0][1],matrixw1[0][2],posx,
       matrixw1[1][0],matrixw1[1][1],matrixw1[1][2],posy,
       matrixw1[2][0],matrixw1[2][1],matrixw1[2][2],posz,
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

  // std::vector<octomap::OcTreeKey*> tvk;
  std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr,CompareKey> thismap;//这一帧中，地图新探索到的octonode
  //所有点进行分类
  for(int i =0;i<pclimg->size();i++)
  {
    octomap::OcTreeKey* thisk(new octomap::OcTreeKey);
    *thisk=m_octree->coordToKey(pclimg->points[i].x,pclimg->points[i].y,pclimg->points[i].z);    
    f_map::FrontierOcTreeNode * thisn=m_octree->search(*thisk);//先在octomap中找到这个节点。
    if(thisn) //如果全局地图中的octonode已经被探索，跳过这个点。
    { 
      // i=i+1;
      delete thisk;
      continue;
    }//加速脱离已知区域

  //////////能运行这一段，说明地图中有新的octonode被探索到了////////////////////////////////    
    std::map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator thisit = thismap.find(thisk);
    if(thisit!=thismap.end())
    {
      thisit->second->points.push_back(pclimg->points[i]);//在当前帧已经标记
    }
    else
    { //在当前帧中还未标记
      // tvk.push_back(thisk);
      pcl::PointCloud<PointXYZ>::Ptr pvector(new pcl::PointCloud<PointXYZ>);
      pvector->points.push_back(pclimg->points[i]);
      thismap[thisk]=pvector;      
    }
  //////////能运行这一段，说明地图中有新的octonode被探索到了////////////////////////////////    
  }
  pclimg.reset(new pcl::PointCloud<PointXYZ>);//释放内存
  double t4=ros::Time::now().toSec();
  std::cout<<"oct索引时间"<<t4-t3<<std::endl;
  pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  float abs_dis=0;
  float per_inline=0;
  int tra_flag = 0;
//关于指针https://blog.csdn.net/qq_36403227/article/details/96595707
//////////这一段进行平面拟合工作////////////////////////////////    
  for(map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  {
    if(it->second->points.size()<250)
    { //无效的平面标记一下
      it->second->points.clear();
      continue;
    }
    tra_flag = fit_plane_to_cloud(plane_model,it->second,0.06,per_inline,abs_dis);
    // std::cout<<"2"<<std::endl;
    m_octree->updateNode(*(it->first),true); //这里把未知的状态标记为已知,在后续frontier搜索中非常关键。
    // std::cout<<"1"<<std::endl;
    if(tra_flag<0)//角度很大的平面，内点太少，认为是不可穿行区域
      m_octree->set_node_info(*(it->first),-1,plane_model->values[0],plane_model->values[1],plane_model->values[2],plane_model->values[3],abs_dis,per_inline);
    else//这里先把frontier和可穿行区域不做区分。0是frontier,1是traversable
      m_octree->set_node_info(*(it->first),1,plane_model->values[0],plane_model->values[1],plane_model->values[2],plane_model->values[3],abs_dis,per_inline);    
  }
  double t5=ros::Time::now().toSec();
  std::cout<<"平面拟合时间"<<t5-t4<<std::endl;
//////////这一段进行平面拟合工作，标记了状态是可穿行，不可穿行。////////////////////////////////    

  float fac_a=0;  float fac_b=0;  float fac_c=0;  float fac_d=0;
  float center_x=0;  float center_y=0;  float center_z=0;//小平面中心点
  float extend_x=0;  float extend_y=0;  float extend_z=0;//小平面延伸点

  //对所有新加入的node进行遍历 ，把指针分配到chenode_set和newfnode_set中，没有被分配的指针释放了。
  std::set<octomap::OcTreeKey*,CompareKey> newfnode_set;//存放新加入所有节点
  vector<octomap::OcTreeKey*> newFrontier_Set;  
  vector<octomap::OcTreeKey*> deleteFrontier_Set;  
  std::set<octomap::OcTreeKey*,CompareKey> chenode_set;//存放活跃区已经是frontier的节点

  for(map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  {
    //上面因为没有到达120个点所以释放了。
    if(it->second->points.size()==0)
    {
      delete (it->first);  //注意这里会产生野指针！！！！但是没有别的办法！！！
      continue;
    }
    f_map::FrontierOcTreeNode *tnode = m_octree->search(*(it->first));//这里node一定不是nullptr
    // if(tnode) std::cout<<"OK"<<std::endl;
    tnode->get_plane(fac_a,fac_b,fac_c,fac_d);//拿到新node平面参数
    vector<pcl::PointXYZ> neibvox;
    octomap::point3d oc_cent = m_octree->keyToCoord(*(it->first));//新node的中心
    int im_fronter=0;//判定当前的it是否为新的frontier
    if(tnode->get_state()!=-1)
    { //新node可穿行
      //1、拿到周围的所有点，并且将frotier放入check_list。
      //2、如果周围所有点中出现了一个未知点，自己加入new_list
      GetPlaneNeib(oc_cent.x(),oc_cent.y(),oc_cent.z(),fac_a,fac_b,fac_c,fac_d,&neibvox);      
      //新node平面扩展得到所有邻居
      for(vector<pcl::PointXYZ>::iterator itnei=neibvox.begin();itnei!=neibvox.end();itnei++)
      {//对邻居进行索引
        f_map::FrontierOcTreeNode *temnode;//邻居node
        octomap::OcTreeKey* thisnei(new octomap::OcTreeKey);
        *thisnei = m_octree->coordToKey(itnei->x,itnei->y,itnei->z);//邻居key          
        std::cout<<" "<<itnei->x<<" "<<itnei->y<<" "<<itnei->z<<std::endl;
        temnode = m_octree->search(*thisnei);      //邻居node,可能不存在    
        //1、拿到周围的所有点，并且将frotier放入check_list。
        if(temnode)
        {//如果邻居node已知
          if(temnode->get_state()==0)//邻居node是frontier,这里只有可能是当前深度帧之前的voxel,因为当前帧的所有voxel都被标记为1了。
          chenode_set.insert(thisnei);            
          else 
          {
          delete thisnei;//这个点不是frontier，则回收空间          
          thisnei=NULL;
          }
        }
        else if((im_fronter==0)&&((tnode->get_state()!=0)))
        { //如果邻居node是一个未知点，自己还不是frontier,则自己加入new_list
          newFrontier_Set.push_back(it->first);
          im_fronter=1;
          delete thisnei;//这个点不是frontier，则回收空间
          thisnei = NULL;
        }
        else
        {
          delete thisnei;//这个点不是frontier，则回收空间      
          thisnei = NULL;
        } 
      }      
    }
    else
    {//不可穿行
      //1、拿到当前node周围的6个点，并且将frotier放入check_list。
      GetPlaneNeib(oc_cent.x(),oc_cent.y(),oc_cent.z(),&neibvox);      
      //拿到周围的6个点邻居
      for(vector<pcl::PointXYZ>::iterator itnei=neibvox.begin();itnei!=neibvox.end();itnei++)
      {
        f_map::FrontierOcTreeNode *temnode;//邻居node
        octomap::OcTreeKey* thisnei(new octomap::OcTreeKey);
        *thisnei = m_octree->coordToKey(itnei->x,itnei->y,itnei->z);//邻居key          
        temnode = m_octree->search(*thisnei);//邻居node          
        //1、拿到周围的所有点，并且将frotier放入check_list。
        if(temnode)//邻居node已知
        {
         if(temnode->get_state()==0)//周围的点是frontier
          {
            chenode_set.insert(thisnei);            
          }
          else 
          {
          delete thisnei;//这个点不是frontier，则回收空间
          thisnei = NULL;
          }
        }
        else 
        {
          delete thisnei;//这个点未知，则回收空间
          thisnei = NULL;
        }
      }
    }
    //如果可穿行，但是不是frontier,或者不可穿行，那么就释放内存空间。
  }
  thismap.clear();
  //从thismap中抽解出chenode_set和newFrontier_Set
  //terrain estimation
  //对之前是frontier，并且需要确认的frontier进行确认。
  for(std::set<octomap::OcTreeKey*>::iterator it=chenode_set.begin();it!=chenode_set.end();it++)
  {
    f_map::FrontierOcTreeNode* tnode = m_octree->search(**it);//这里node一定不是nullptr
    tnode->get_plane(fac_a,fac_b,fac_c,fac_d);//拿到需要检测node平面参数
    octomap::point3d oc_cent = m_octree->keyToCoord(**it); //需要检测node的中心
    vector<pcl::PointXYZ> neibvox;
    GetPlaneNeib(oc_cent.x(),oc_cent.y(),oc_cent.z(),fac_a,fac_b,fac_c,fac_d,&neibvox);      
    //需要检测node平面扩展得到所有邻居
    int flag_stillfrontier=0;
    for(vector<pcl::PointXYZ>::iterator itnei=neibvox.begin();itnei!=neibvox.end();itnei++)
    {//对邻居进行索引
      f_map::FrontierOcTreeNode *temnode;//邻居node
      octomap::OcTreeKey thisnei;
      thisnei = m_octree->coordToKey(itnei->x,itnei->y,itnei->z);//邻居key          
      temnode = m_octree->search(thisnei);      //邻居node,可能不存在    
      if(temnode)
      {//如果邻居node已知
        continue;
      }
      else
      { //如果邻居node未知,说明现在还是frontier!!
        flag_stillfrontier=1;
        break;
      } 
    }
    if(flag_stillfrontier==0)
    {
      //如果现在已经不是frontier了
      frontier.erase(*it);

    octomap::point3d poi = m_octree->keyToCoord(**it);
    std::pair<int,int> thpp = sub_space::Frontier2Subspaceindex(poi.x(),poi.y(),poi.z());
    subspace_array[thpp.first][thpp.second]->deleteFrontier(*it);

      //TODO： DELETE Frontier from subspace
      delete (*it);//这里也会产生野指针！！！
    }
  }
  chenode_set.clear();

  f_map::FrontierOcTreeNode* tnode; 
  for(std::vector<octomap::OcTreeKey*>::iterator it=newFrontier_Set.begin();it!=newFrontier_Set.end();it++)
  {
    frontier.insert(*it);
    //TODO::ADD TO SUBSPACE
    octomap::point3d poi = m_octree->keyToCoord(**it);
    std::pair<int,int> thpp = sub_space::Frontier2Subspaceindex(poi.x(),poi.y(),poi.z());
    subspace_array[thpp.first][thpp.second]->addFrontier(*it);
    tnode = m_octree->search(**it);//这里node一定不是nullptr
    tnode->set_state(0);//状态设定为frontier
  }
  // for(map<octomap::OcTreeKey*,pcl::PointCloud<PointXYZ>::Ptr>::iterator it = thismap.begin();it!=thismap.end();it++)
  //   delete (it->first); 不能delete的原因，是frontier的指针也在里面！！！
  newFrontier_Set.clear();
  // 已知frontier_list
  // 所有新覆盖的节点，如果在平面延续的8个点中出现了frontier，frotier放入check_list。
  // 如果出现了未知点，则将自己放入new_list
  // check_list遍历，如果不符合条件，从frontier_list中删除。
  // new_list加入frontier_list
  //在探索决策时，对frontier进行断层检测。即沿着frontier的两个法方向进行扩散得到两片的octonode，
  //如果发现两片octonode的z纬度差异过大，则认为是断层。
  double t6=ros::Time::now().toSec();
  std::cout<<"Frontier动态更新时间"<<t6-t5<<std::endl;
  std::cout<<"总时间"<<t6-t0<<std::endl;
  int d1=0;
  PublishPlane();  
  // PublishFrontierMarker();
  // for(int i=0;i<=7;i++)
  // for(int j=0;j<=7;j++)
  // {
  //   if(subspace_array[i][j]->state==sub_space::EXPLORING)
  //   {
  //     // std::cout<<i<<" "<<j<<" 状态"<<subspace_array[i][j]->frontiers.size()<<std::endl;
  //   }
  //   d1 += subspace_array[i][j]->frontiers.size();
  // }//初始化
  // std::cout<<d1<<frontier.size()<<std::endl;

  // ROS_ASSERT(0);
  // if(temc==500)
  // {  
  //   pcl::PointXYZ pppp;
  //   octomap::point3d otpp;
  //   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//法向量计算结果
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr center_plane(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::Normal nt;
  //   f_map::FrontierOcTreeNode* tnode; float ttt=0;
  //   for(std::set<octomap::OcTreeKey*>::iterator it=frontier.begin();it!=frontier.end();it++)
  //   {
  //     tnode = m_octree->search(**it);
  //     tnode->get_plane(nt.normal_x,nt.normal_y,nt.normal_z,ttt);
  //     normals->points.push_back(nt);
  //     otpp = m_octree->keyToCoord(**it);
  //     pppp.x=otpp.x();      pppp.y=otpp.y();      pppp.z=otpp.z();
  //     center_plane->points.push_back(pppp);
  //   }
  //   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); //创建视窗对象，定义标题栏名称“3D Viewer”
  //   viewer->addPointCloud<pcl::PointXYZ>(center_plane, "original_cloud");	//将点云添加到视窗对象中，并定义一个唯一的ID“original_cloud”
  //   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "original_cloud"); //点云附色，三个字段，每个字段范围0-1
  //   viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(center_plane, normals, 1, 0.6, "normals");	//每十个点显示一个法线，长度为0.05
  //   while (!viewer->wasStopped ())
  //   {
  //     viewer->spinOnce (100);
  //     std::this_thread::sleep_for(100ms);
  //   }    
  //   ROS_ASSERT(0);
  // }

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
int Teclas::fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes,float& precent_inline,float& varia)
{
  float sqd = 0;
  float all_dis=0;
  // std::cout<<"fit_plane_to_cloud"<<std::endl;
  try 
  {
  // std::cout<<"fit_plane_to_cloud1"<<std::endl;
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
  // std::cout<<"fit_plane_to_cloud2"<<std::endl;
    float xita = acos(c/sqd);//法向量与垂直z的夹角，相当平面与水平面的夹角。
    precent_inline = inliers->indices.size()/cloud->points.size();
    if(xita>xita_thred||(precent_inline<0.7))
    {
      varia=-1;
      std::cout<<"Not"<<std::endl;
      return -1;//内点不够多，角度太大，放弃
    } 
    all_dis=0;
    for(int i=0;i<cloud->points.size();i++)
    {
      all_dis+=std::abs(a*cloud->points[i].x+
                        b*cloud->points[i].y+
                        c*cloud->points[i].z+
                        d);
    }
  // std::cout<<"fit_plane_to_cloud3"<<std::endl;
  all_dis = all_dis/(cloud->points.size());
  varia=all_dis/sqd;
  std::cout<<"varia "<<varia<<"precent_inline "<<precent_inline<<std::endl;
  return 1;
  }
  catch (...) 
  {
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    return -1;
  }
}

//在subspace中删除frontier
//根据subspace中frontier的数量来更新subspace的状态
void Teclas::Update_SubspaceFrontier(octomap::OcTreeKey* tk)
{
  octomap::point3d tp = m_octree->keyToCoord(*tk);
  std::pair<int,int> tidx = sub_space::Frontier2Subspaceindex(tp.x(),tp.y(),tp.z());
  
}
int main (int argc, char** argv)
{

  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  Teclas T(nh);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return (0);

}

 