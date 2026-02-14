cmake_minimum_required(VERSION 3.0.2)
project(ug_hardware_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ug_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hardware_sim_plugin
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs ug_msgs gazebo_ros
  DEPENDS GAZEBO
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(hardware_sim_plugin
  src/HardwareSimPlugin.cc
)

target_link_libraries(hardware_sim_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(hardware_sim_plugin
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS hardware_sim_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
