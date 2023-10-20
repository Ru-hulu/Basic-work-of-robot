#include <stdio.h>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
using namespace std;
 
int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_node");
    ros::NodeHandle nh;
    ros::Publisher Vel_pub = nh.advertise<geometry_msgs::Twist>("vel_cmd",5);
    geometry_msgs::Twist tmsg;
    ros::Rate tz(10);
    char tt;
    while(ros::ok())
    {
        ros::spinOnce();
        std::cin>>tt;
        switch (tt)
        {
        case 'i':
            tmsg.angular.x=0;
            tmsg.linear.x=0.250;   
            break;
        case 'k':
            tmsg.angular.x=0;
            tmsg.linear.x=-0.250;   
            break;
        case 'j':
            tmsg.angular.x=0.250;
            tmsg.linear.x=0.250;   
            break;
        case 'l':
            tmsg.angular.x=-0.250;
            tmsg.linear.x=0.250;   
            break;        
        case '0':
            tmsg.angular.x=0;
            tmsg.linear.x=0;   
            break;        
        }     
        Vel_pub.publish(tmsg);
        tz.sleep();
    }

}
