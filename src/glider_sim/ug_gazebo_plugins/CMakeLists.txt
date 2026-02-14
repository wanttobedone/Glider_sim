cmake_minimum_required(VERSION 3.0.2)
project(ug_gazebo_plugins)

## Use C++11
add_compile_options(-std=c++11)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  gazebo_ros
)

## Find Gazebo
find_package(gazebo REQUIRED)

## Generate messages
add_message_files(
  FILES
  BuoyancyEngineStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES buoyancy_engine_plugin
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime gazebo_ros
  DEPENDS GAZEBO
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Build the plugin
add_library(buoyancy_engine_plugin
  src/BuoyancyEnginePlugin.cc
)

target_link_libraries(buoyancy_engine_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(buoyancy_engine_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Install
install(TARGETS buoyancy_engine_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
