#include<stdlib.h>
#include<iostream>
#include<ros/ros.h>
#include<gazebo_msgs/SetModelState.h>
#include<gazebo_msgs/SetJointProperties.h>
#include<geometry_msgs/Twist.h>
#include <cstdio>
int main(int argc,char** argv)
{
    ros::init(argc,argv,"control_node");
    ros::NodeHandle nh;
    ros::ServiceClient al =  nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Publisher pal =  nh.advertise<geometry_msgs::Twist>("/cmd_vel",5);
    gazebo_msgs::SetModelState model_states;
    model_states.request.model_state.reference_frame="scout";
    model_states.request.model_state.model_name="scout";
 
    int cy=0;
    char tt;
    ros::Rate rz(50);
    
    while(ros::ok())
    {
        if(cy>1000) cy=0;
        if(cy%2==0)
        std::cin>>tt;
        cy++;
        switch (tt)
        {
        case 'i':
            model_states.request.model_state.pose.position.x=0.1;
            model_states.request.model_state.pose.position.y=0;
            model_states.request.model_state.pose.position.z=0;
            model_states.request.model_state.pose.orientation.x=0;
            model_states.request.model_state.pose.orientation.y=0;
            model_states.request.model_state.pose.orientation.z=0;
            model_states.request.model_state.pose.orientation.w=0;
            break;
        case 'k':
            model_states.request.model_state.pose.position.x=0;
            model_states.request.model_state.pose.position.y=0;
            model_states.request.model_state.pose.position.z=0;
            model_states.request.model_state.pose.orientation.x=0;
            model_states.request.model_state.pose.orientation.y=0;
            model_states.request.model_state.pose.orientation.z=0;
            model_states.request.model_state.pose.orientation.w=0;
            break;
        case 'j':
            model_states.request.model_state.pose.position.x=0;
            model_states.request.model_state.pose.position.y=0;
            model_states.request.model_state.pose.position.z=0;
            model_states.request.model_state.pose.orientation.x=0;
            model_states.request.model_state.pose.orientation.y=0;
            model_states.request.model_state.pose.orientation.z=0.087;
            model_states.request.model_state.pose.orientation.w=0.996;
            break;
        case 'l':
            model_states.request.model_state.pose.position.x=0;
            model_states.request.model_state.pose.position.y=0;
            model_states.request.model_state.pose.position.z=0;
            model_states.request.model_state.pose.orientation.x=0;
            model_states.request.model_state.pose.orientation.y=0;
            model_states.request.model_state.pose.orientation.z=-0.087;
            model_states.request.model_state.pose.orientation.w=0.996;
            break;        
        }
        al.call(model_states);
        rz.sleep();
    }
    return 1;
}