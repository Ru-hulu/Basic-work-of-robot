#include<subspace.hpp>
#include<ros/ros.h>
std::pair<int,int>  sub_space::Frontier2Subspaceindex(float x,float y,float z)
{       
    int ix = x/0.25;
    int iy = y/0.25;
    int rx = ix/20 +4;
    int ry = iy/20 +4;//因为frontier是世界坐标下的，现在转到8 8 下
    return std::pair<int,int>(rx,ry);
}

int sub_space::Subspace::deleteFrontier(octomap::OcTreeKey* tk)
{
    if (frontiers.size()==0)
        ROS_ASSERT(0);
    frontiers.erase(tk);
    if (frontiers.size()==0)
    state = sub_space::EXPLORED;
    return state;    
}//可以从正在探索 到 探索完毕

int sub_space::Subspace::addFrontier(octomap::OcTreeKey* tk)
{
    frontiers.insert(tk);
    if(state==sub_space::UNEXPLORED)
    state = sub_space::EXPLORING;
    return state;    
}//可以从为探索到正在探索