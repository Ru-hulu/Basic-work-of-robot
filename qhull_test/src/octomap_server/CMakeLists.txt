cmake_minimum_required(VERSION 3.0.2)
project(octomap_server)
# SET(CMAKE_BUILD_TYPE Debug) #debug模式
SET(CMAKE_BUILD_TYPE Release) #Releas
set(
  PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  dynamic_reconfigure
  nodelet
  octomap_ros
  octomap_msgs
  )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(octomap REQUIRED)
find_package(CUDA REQUIRED) 
# find_package(octomap 1.9.8 REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS} 
  ${CUDA_INCLUDE_DIRS}
)
catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)




message("---------${CUDA_include_directory}-------")
message("---------${CUDA_LIBRARIES}-------")
message("---------${CUDA_LIBRARIES}-------")

# set(LINK_LIBS ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${CUDA_LIBRARIES} pcl_visualization)
# cuda_add_executable(cudatest src/cudatest.cpp src/addition.cu)
# target_link_libraries(cudatest ${LINK_LIBS})

set(LINK_LIBS ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} pcl_visualization)
add_executable(test1 src/test1.cpp src/surfeltree.cpp src/subspace.cpp)
target_link_libraries(test1 ${LINK_LIBS})
 
