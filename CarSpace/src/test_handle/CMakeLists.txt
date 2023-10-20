cmake_minimum_required(VERSION 3.0.2)
project(test_handle)
set(CMAKE_BUILD_TYPE Debug)
set(PACKAGE_DEPENDENCIES
  roscpp
  std_msgs
  std_srvs
  gazebo_msgs  
  geometry_msgs
  )

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
)


add_executable(test_node src/main.cpp)
target_link_libraries(test_node ${LINK_LIBS})