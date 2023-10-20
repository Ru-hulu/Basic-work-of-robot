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

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::chrono_literals;

  std::vector<double> rpy_from_plane(const pcl::ModelCoefficients plane_model)
  {
    std::vector<double> rpy({0.0, 0.0, 0.0});
    float roll = std::atan2(
      plane_model.values[1],
      plane_model.values[2]);
    float pitch = std::atan2(
      plane_model.values[0],
      plane_model.values[2]);
    rpy[0] = -roll;
    rpy[1] = pitch;
    // Yaw shouldnt matter at coordinate fitted plane,
    // it can be anything in between [-M_PI, M_PI]
    rpy[2] = 0.0;
    return rpy;
  }
  //输入 采样以前的点集、采样以后的点集、surfel半径。
  //输出：采样以后的点A，在采样以前的集合中周围surfel半径中的所有点构成的集合C. 返回vector<A,C>
  std::vector<std::pair<pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> surfelize_traversability_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pure_traversable_pcl,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr uniformly_sampled_nodes,
    double radius)
  { //输入 采样以前的点、采样以后的点、surfel半径。
    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(pure_traversable_pcl);//采样以前的点
    std::vector<std::pair<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> decomposed_cells;
    for (auto && searchPoint : uniformly_sampled_nodes->points)//对采样以后的点A遍历
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_within_this_cell(new pcl::PointCloud<pcl::PointXYZRGB>);//采样以前，A周围surfel半径中的所有点
      //以采样以后的一个点searchPoint为中心，以sufel半径在采样以前的点中做最近搜索
      if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0)
      {
        //搜索得到半径中的所有点
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) 
        {
          auto crr_point = pure_traversable_pcl->points[pointIdxRadiusSearch[i]];//搜索得到的一个点
          points_within_this_cell->points.push_back(crr_point);
        }
      }
      points_within_this_cell->height = 1;
      points_within_this_cell->width = points_within_this_cell->points.size();

      decomposed_cells.push_back(std::make_pair(searchPoint, points_within_this_cell));
    }
    return decomposed_cells;
  }
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
  Teclas(const ros::NodeHandle &nh_);
  void insertCloudCallback(const sensor_msgs::ImageConstPtr & msg);
  //点云转换的时间大概是70ms
};

Teclas::Teclas(const ros::NodeHandle &nh_):nh(nh_)
{
ims=nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw",5,&Teclas::insertCloudCallback,this);
states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
};
void Teclas::insertCloudCallback(const sensor_msgs::ImageConstPtr & msg)
{
  std::cout<<"IO"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclimg(new(pcl::PointCloud<pcl::PointXYZ>));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcfree(new(pcl::PointCloud<pcl::PointXYZ>));
  Eigen::Matrix3d K;    
  Eigen::Matrix3d K_inv1;
  K << 695.995117, 0.0, 640.0, 0.0, 695.995117, 360.0, 0.0, 0.0, 1.0;
  K_inv1 = K.inverse();
  std::cout<<K_inv1<<std::endl;
  float K_inv[9] = {0.00143679,0,-0.919547,0,0.00143679,-0.517245,0,0,1};  
  float camera_depth=8;  
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

    double ts=ros::Time::now().toSec();

  int downfactor=2;
  for(v=20;v<he;v++)
  for(u=0;u<wi;u=u+downfactor)
  {
    int idx = u+v*msg->width;
    ushort tmp = Littlesw(msg->data[idx * 2], msg->data[idx * 2 + 1]);
    z_c = float(tmp)/1000;    
    if(((z_c>0)&&(z_c<=camera_depth)))
    {
    thisp.x =  z_c*(K_inv[0]*u+K_inv[1]*v+K_inv[2]*1);   
    thisp.y =  z_c*(K_inv[3]*u+K_inv[4]*v+K_inv[5]*1);   
    thisp.z =  z_c;   
    pclimg->points.push_back(thisp);
    }
    else
    {
    z_c=camera_depth;
    thisp.x =  z_c*(K_inv[0]*u+K_inv[1]*v+K_inv[2]*1);   
    thisp.y =  z_c*(K_inv[3]*u+K_inv[4]*v+K_inv[5]*1);   
    thisp.z =  z_c;   
    pcfree->points.push_back(thisp);
    }    
  }
  double ts1=ros::Time::now().toSec();
  pclimg->width=1;
  pclimg->height=pclimg->points.size();
  std::cout<<"The size "<<pclimg->points.size()<<"  "<<ts1-ts<<std::endl;
  pcl::transformPointCloud(*pclimg, *pclimg, Twi*Tic);
pcl::io::savePCDFile("/home/r/Mysoftware/qhull_test/img_pcd.pcd", *pclimg); 
ROS_ASSERT(0);
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
  // for(int i=0;i<=1000;i++)
  // {
  //   std::cout<<"point data"<<i<<" "<< cloud_org1->points[i].x<<" "<< cloud_org1->points[i].y<<" "<< cloud_org1->points[i].z<<std::endl;
  //   cloud_filter->points.push_back(cloud_org1->points[i]);
  // }


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
    pcl::Normal nt;
    int idx =0;

  for(int u=0;u<=719;u=u+7)
  {
    double st_time = ros::Time().now().toSec();
    for(int v=0;v<=1279;v=v+7)
    {
      if(u+6>719||v+6>1279) continue;
      for(int uu=u;uu<=u+6;uu++)
      {
        for(int vv=v;vv<=v+6;vv++)
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
        // std::cout<<u<<" "<<v<<std::endl;      

        // cloud_surcenter->points.push_back(cloud_org1->points[idx]);
    }
        double st_time1 = ros::Time().now().toSec();
        std::cout<<"Time "<<st_time1-st_time<<std::endl;
  }

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer")); //创建视窗对象，定义标题栏名称“3D Viewer”
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_org1, "original_cloud");	//将点云添加到视窗对象中，并定义一个唯一的ID“original_cloud”
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "original_cloud"); //点云附色，三个字段，每个字段范围0-1
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud_org1, normals, 10, 0.6, "normals");	//每十个点显示一个法线，长度为0.05


  // for (const auto& point: *cloud_org1)
  // {
  //   pcl::PointXYZRGB thiss;
  //   thiss.x=point.x;
  //   thiss.y=point.y;
  //   thiss.z=point.z;
  //   if(isnan(thiss.z)||isnan(thiss.y)||isnan(thiss.x))continue;
  //   thiss.r=0.0;
  //   thiss.g=255.0;
  //   thiss.b=0.0;
  //   cloud_org->points.push_back(thiss);
  // }
  // std::cout<<"Delet nan size is "<< cloud_org->points.size()<<std::endl;
	// sor.setInputCloud(cloud_org);
	// sor.setDownsampleAllData(1);
	// sor.setLeafSize(0.1, 0.1, 0.1);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
  // // sor.setMinimumPointsNumberPerVoxel(10);//设置每个体素中的点的个数
	// sor.filter(*cloud_filter);
  // std::cout<<"0.1 downsample size is "<< cloud_filter->points.size()<<std::endl;
  //     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
  //     sor1.setInputCloud(cloud_filter);
  //     sor1.setMeanK(10);
  //     sor1.setStddevMulThresh(1.0);
  //     sor1.filter(*cloud_filter1);
  // //去掉外点。
  //   pcl::UniformSampling<pcl::PointXYZRGB> filter;
  //   filter.setInputCloud(cloud_filter1);
  //   filter.setRadiusSearch(0.2);
  //   filter.filter(*cloud_filter2);
  //   cloud_filter2->height = 1;
  //   cloud_filter2->width = cloud_filter2->points.size();
  // //进行一次降采样。
  // std::cout<<"0.2 downsample size is "<< cloud_filter2->points.size()<<std::endl;
  //   auto surfels = surfelize_traversability_cloud(cloud_filter1,cloud_filter2,3);
  // 	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//法向量计算结果
  //   pcl::Normal nt;
  //   int ct =0;
  //   for (auto && i : surfels) 
  //   {
  //     auto surfel_center_point = i.first;//采样以后的一个点
  //     auto surfel_cloud = i.second;//采样以前的一个点集
  //     // fit a plane to this surfel cloud, in order to et its orientation
  //     pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  //     fit_plane_to_cloud(plane_model,surfel_cloud,0.2);
  //     // auto rpy = rpy_from_plane(*plane_model);//拟合出的平面的法向量      
  //     // nt.normal_x = rpy[0];
  //     // nt.normal_y = rpy[1]; 
  //     // nt.normal_z = rpy[2]; 
  //     nt.normal_x = plane_model->values[0];
  //     nt.normal_y = plane_model->values[1]; 
  //     nt.normal_z = plane_model->values[2]; 
  //     normals->points.push_back(nt);
  //     cloud_surcenter->points.push_back(surfel_center_point);
  //     int s = surfel_cloud->points.size();
  //     ct = ct+s;
  //     // std::cout<<rpy[0]<<rpy[1]<<rpy[2]<<s<<std::endl;
  //     // std::cout<<"The size is "<<s<<std::endl;
  //   }
  // std::cout<<"centor size is "<< cloud_surcenter->points.size()<<std::endl;    
	//-------------------------- 法线可视化 --------------------------


  // std::cout<<"T"<<std::endl;
  // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // viewer->setBackgroundColor (0, 0, 0);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> handsur(cloud_org,0, 255, 0);
  // viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filter, handsur, "sample cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  // viewer->addCoordinateSystem (1.0);
  // viewer->initCameraParameters ();





// int main ()
// {pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);pcl::visualization::CloudViewer viewer ("My Cloud Viewer"); viewer.runOnVisualizationThreadOnce (viewerOneOff);while (!viewer.wasStopped ()){}return 0;
// }



  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }

  return (0);

}

 