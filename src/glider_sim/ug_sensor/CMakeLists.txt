cmake_minimum_required(VERSION 3.0.2)
project(ug_sensor)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  sensor_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES ug_pressure_sensor_plugin
  CATKIN_DEPENDS gazebo_ros roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# 压力传感器 (深度) 插件
add_library(ug_pressure_sensor_plugin SHARED
  src/ug_pressure_sensor_plugin.cpp
)
target_link_libraries(ug_pressure_sensor_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
