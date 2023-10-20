cmake_minimum_required(VERSION 3.5)
project(cam_node)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
set(OpenCV_DIR /usr/share/OpenCV/)
set(DEPENDENCIES realsense2)
######################
### Packages
######################
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    roslib
    # msg
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    # cv
    cv_bridge
    # pcl
    pcl_conversions
    # msg generation
    message_generation
)
find_package(realsense2 REQUIRED) 
find_package(OpenCV 4.2.0 REQUIRED)

######################
### Catkin
######################
catkin_package()
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

link_directories(
    ${OpenCV_LIBRARY_DIRS}
)

message(${OpenCV_LIBS})
message(${OpenCV_LIBRARIES})

add_executable(cam_node src/cam_node.cpp)
target_link_libraries(cam_node  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY})
add_dependencies(cam_node ${catkin_EXPORTED_TARGETS})
