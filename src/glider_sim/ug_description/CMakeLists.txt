cmake_minimum_required(VERSION 3.0.2)
project(ug_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  robot_state_publisher
  xacro
  uuv_descriptions
  uuv_gazebo_ros_plugins
)

catkin_package()

# Install meshes, urdf, launch, config
install(DIRECTORY meshes urdf launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
