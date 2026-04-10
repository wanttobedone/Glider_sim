cmake_minimum_required(VERSION 3.0.2)
project(ug_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(depth_converter_node src/depth_converter_node.cpp)
target_link_libraries(depth_converter_node ${catkin_LIBRARIES})
