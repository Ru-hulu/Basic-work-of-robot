cmake_minimum_required(VERSION 3.0.2)
project(control_handle)
set(CMAKE_BUILD_TYPE Debug)
set(PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
  std_srvs
  gazebo_msgs  
  geometry_msgs
  )
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/local/include/yaml-cpp)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

link_directories(/usr/lib/x86_64-linux-gnu)
link_directories(${PROJECT_SOURCE_DIR}/lib)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  libcontrolcan.so 
  yaml-cpp
)


add_executable(control_node src/main.cpp)
target_link_libraries(control_node ${LINK_LIBS})