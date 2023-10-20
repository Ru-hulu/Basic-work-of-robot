cmake_minimum_required(VERSION 3.0.2)
project(control_handle)
set(CMAKE_BUILD_TYPE Debug)
set(PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
  std_srvs
  gazebo_msgs  
  )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
)

add_executable(control_node src/keyboard.cpp)
target_link_libraries(control_node ${LINK_LIBS})
