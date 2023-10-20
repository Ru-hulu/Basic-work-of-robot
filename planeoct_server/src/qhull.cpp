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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::chrono_literals;
//这个文件用于测试qhull算法，论文OVPC mesh中，将点云快速生成三角面。可用于探索算法中的前沿表示。
int main (int argc, char** argv)
{
  std::cout<<"1111111111111111"<<std::endl;
  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  std::cout<<"1111111111111111"<<std::endl;  
	srand((unsigned int)time(NULL));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orgf(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout<<"2222222222222"<<std::endl;  
  if (pcl::io::loadPCDFile("/home/r/Mysoftware/qhull_test/test_pcd.pcd", *cloud_org) == -1) 
  {
      PCL_ERROR("Couldn't read file  \n");
      return (-1);
  }
  pcl::PointCloud<pcl::PointSurfel> thissurfel;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  for (const auto& point: *cloud_org)
  {
    pcl::PointSurfel thiss;
    thiss.x=point.x;
    thiss.y=point.y;
    thiss.z=point.z;
    if(isnan(thiss.z)||isnan(thiss.y)||isnan(thiss.x))continue;
    thiss.normal_x=0;
    thiss.normal_y=0;
    thiss.normal_z=1;
    thiss.r=100;
    thiss.g=0;
    thiss.b=100;
    thiss.a=50;
    thiss.radius = 0.5;
    thissurfel.points.push_back(thiss);
  }
  int temps = thissurfel.points.size();
  thissurfel.width=temps;
  thissurfel.height=1;
  pcl::io::savePCDFileASCII("/home/r/Mysoftware/qhull_test/result_surfel.pcd", thissurfel);
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  // pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<pcl::PointSurfel> handsur(thissurfel.makeShared());
  // viewer->addPointCloud<pcl::PointSurfel> (thissurfel.makeShared(), handsur, "sample cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 3, "sample cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handsur(cloud_org,0, 255, 0);
	sor.setInputCloud(cloud_org);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(15, 15, 15);//设置滤波器处理时采用的体素大小的参数
  // sor.setMinimumPointsNumberPerVoxel(2);//设置每个体素中的点的个数
	sor.filter(*cloud_filter);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_filter, handsur, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();




  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  // viewer->addCoordinateSystem (1.0);
  // viewer->initCameraParameters ();


  //   std::cout << "    " << point.x
  //             << " "    << point.y
  //             << " "    << point.z << std::endl;

  PointCloud<PointXYZ> cloud_org_useful;//原始点云中,映射后构建凸包,会在凸包上的点
  PointCloud<PointXYZ> cloud_out;//映射后的在凸包上的点
  float tx;  float ty;  float tz;  float tr_flat = 0;  int copo = 0;
  // cloud_in->points.resize(360*5*31);
  int ptsize = cloud_org->points.size();
  double tt1 = ros::Time::now().toSec();
  double gamma = -0.03;float dis_p=0;float disexp_p=0;
  for(int i=0;i<ptsize;i++)
  {
        tx = cloud_org->points[i].x;
        ty = cloud_org->points[i].y;
        tz = cloud_org->points[i].z;
        if(isnan(tx)||isnan(ty)||isnan(tz))continue;
        pcl::PointXYZ thisp;
        thisp.x=tx;thisp.y=ty;thisp.z=tz;
        cloud_orgf->points.push_back(thisp);
        dis_p = sqrt(tx*tx+ty*ty+tz*tz);
        disexp_p = pow(dis_p,gamma);
        tx = disexp_p/dis_p*tx;
        ty = disexp_p/dis_p*ty;
        tz = disexp_p/dis_p*tz;
        thisp.x=tx;thisp.y=ty;thisp.z=tz;
        cloud_in->points.push_back(thisp);
  }
    for (const auto& point: *cloud_in)
    std::cout << "    " << point.x
              << " "    << point.y
              << " "    << point.z << std::endl;

  // for(float i =0;i<360;i=i+0.2)
  // {
  //   for(float j =-15;j<=15;j=j+1)
  //   {
  //     float num_org = rand() % 20 + 1;//生成0~21+1的随机数
  //     // if(i<50&&i>=10)num_org=5;
  //     // if(i<200&&i>=150)num_org=15;
  //       float num = pow(num_org,gamma);
  //       tr_flat = num*cos(j/57.2957);
  //       tz = num*sin(j/57.2957);
  //       tx = tr_flat*cos(i/57.2957);
  //       ty = tr_flat*sin(i/57.2957);
  //       cloud_in->points[copo].x = tx;    
  //       cloud_in->points[copo].y = ty;    
  //       cloud_in->points[copo].z = tz;    

  //     tr_flat = num_org*cos(j/57.2957);
  //     tz = num_org*sin(j/57.2957);
  //     tx = tr_flat*cos(i/57.2957);
  //     ty = tr_flat*sin(i/57.2957);
  //     cloud_org->points[copo].x = tx;    
  //     cloud_org->points[copo].y = ty;    
  //     cloud_org->points[copo].z = tz;    
  //     copo++;
  //   }
  // }
  double tt2 = ros::Time::now().toSec();
  std::cout<<"T1"<<tt2-tt1<<std::endl;
  std::vector< pcl::Vertices > polygonsthsi; //映射以后的凸包面片
  std::vector< pcl::Vertices > polygonsorg; //映射以前的凸包面片
  ConvexHull<PointXYZ> convex_hull;  // convex_hull.setSearchMethod();
  std::cout<<"1111111111111111"<<std::endl;
  convex_hull.setInputCloud (cloud_in);
  // convex_hull.reconstruct(cloud_out,polygonsthsi);  // convex_hull.reconstruct (mesh_out);
  std::cout<<"1111111111111111"<<std::endl;
  convex_hull.reconstruct(cloud_out,polygonsthsi);
  double tt3 = ros::Time::now().toSec();
  std::cout<<"T2"<<tt3-tt2<<std::endl;
  pcl::PointIndices hull_point_indices;//cloud_out和cloud_org的对应index关系
  convex_hull.getHullPointIndices(hull_point_indices);
  int surfel_size = polygonsthsi.size();
  pcl::Vertices org_thisv;
  int t_i;//cloud_org上的index
  for(int i =0;i<cloud_out.points.size();i++)
  {
    t_i = hull_point_indices.indices[i];
    cloud_org_useful.points.push_back(cloud_orgf->points[t_i]);
  }
  double tt4 = ros::Time::now().toSec();
  std::cout<<"T3"<<tt4-tt3<<std::endl;
  // for(int i =0;i<surfel_size;i++)
  // {
  //   org_thisv.vertices.clear();
  //   int a = polygonsthsi[i].vertices[0];
  //   int b = polygonsthsi[i].vertices[1];
  //   int c = polygonsthsi[i].vertices[2];//映射后面片的三个点索引
  //   int tri_org_a=hull_point_indices.indices[a];    int tri_org_b=hull_point_indices.indices[b];    int tri_org_c=hull_point_indices.indices[c];
  //   //找到之前的三个点索引
  //   org_thisv.vertices.push_back(tri_org_a);        org_thisv.vertices.push_back(tri_org_b);        org_thisv.vertices.push_back(tri_org_c);
  //   polygonsorg.push_back(org_thisv);
  //   // std::cout<<"Vertice "<<i<<" "<<tri_org_a<<" "<<tri_org_b<<" "<<tri_org_c<<" "<<std::endl;    
  //   std::cout<<"Vertice "<<i<<" "<<org_thisv<<std::endl;    
  // }
  pcl::PCLPointCloud2* local_map_pc2_ptr = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2(cloud_org_useful, *local_map_pc2_ptr);
  PolygonMesh mesh_out;
  mesh_out.cloud = *local_map_pc2_ptr;
  mesh_out.polygons = polygonsthsi; 
  std::cout<<"number "<<cloud_out.size()<<" "<<cloud_org_useful.points.size()<<std::endl;    
  // pcl::PCLPointCloud2* local_map_pc2_ptr = new pcl::PCLPointCloud2;
  // pcl::toPCLPointCloud2(cloud_out, *local_map_pc2_ptr);
  // PolygonMesh mesh_out;
  // mesh_out.cloud = *local_map_pc2_ptr;
  // mesh_out.polygons = polygonsthsi; 
  io::saveVTKFile ("qhu.vtk", mesh_out);
  double tt5 = ros::Time::now().toSec();
  std::cout<<"T4"<<tt5-tt4<<std::endl;
  std::cout<<"Total "<<tt5-tt1<<std::endl;
  return (0);
}


      // cloud_in->points.resize(7);
      // cloud_in->points[0].x = 0;    
      // cloud_in->points[0].y = 0;    
      // cloud_in->points[0].z = 0;    
      // cloud_in->points[1].x = 2;    
      // cloud_in->points[1].y = 0;    
      // cloud_in->points[1].z = 0;    
      // cloud_in->points[2].x = 0.5;    
      // cloud_in->points[2].y = 0.5;    
      // cloud_in->points[2].z = 0.5;    
      // cloud_in->points[3].x = 0;    
      // cloud_in->points[3].y = 2;    
      // cloud_in->points[3].z = 0;    
      // cloud_in->points[4].x = 0;    
      // cloud_in->points[4].y = 0;    
      // cloud_in->points[4].z = 2;    
      // cloud_in->points[5].x = 0.01; cloud_in->points[6].x = 2;    
      // cloud_in->points[5].y = 0.01; cloud_in->points[6].y = 2;    
      // cloud_in->points[5].z = 0.01; cloud_in->points[6].z = 0;    
      // tx = (1+2*(eq_r-num)/num)*tx;
      // ty = (1+2*(eq_r-num)/num)*ty;
      // tz = (1+2*(eq_r-num)/num)*tz;