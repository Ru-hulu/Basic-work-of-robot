#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
# include <opencv2/core/core.hpp>
# include <opencv2/highgui/highgui.hpp>
#include "librealsense2/rs.hpp" // Include RealSense Cross Platform API
#include "librealsense2/rsutil.h"
#include <unistd.h>
#define ESC 27
/*
This version has depth data...
*/
using namespace cv;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "vins");
  ros::NodeHandle n;
  rs2::pipeline pipe;
  rs2::config pipe_config;
  ros::Publisher imgpub  = n.advertise<sensor_msgs::Image>("/camera/image_raw", 5);

//  pipe_config.enable_stream(RS2_STREAM_DEPTH,720,540,RS2_FORMAT_Z16,30);
  pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
//  rs2::align align_to_color(RS2_STREAM_COLOR);
//  cv::namedWindow("windowtest");
  rs2::pipeline_profile profile = pipe.start(pipe_config);
  std_msgs::Header imghead;
  imghead.frame_id = "map";
  imghead.seq=1;
//  while(cv::waitKey(30) < 0)
  while(cv::waitKey(3))
  { 
    rs2::frameset frameset = pipe.wait_for_frames();
    if(&frameset == nullptr) continue;
//    frameset = align_to_color.process(frameset);
  
        //取深度图和彩色图
    rs2::frame colorFrame = frameset.get_color_frame();//processed.first(align_to);
//    rs2::frame deepFrame = frameset.get_depth_frame();
    if((&colorFrame != nullptr)) 
    {
       imghead.stamp = ros::Time::now();
       cv::Mat ReferenceFrame(Size(640,480),CV_8UC3,(void*)colorFrame.get_data(),Mat::AUTO_STEP);
       sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(imghead, "bgr8", ReferenceFrame).toImageMsg();  
       imgpub.publish(imgmsg);
//       cv::imshow("windowtest",ReferenceFrame);		  
    }
  }
  return 0; 
}
