cmake_minimum_required(VERSION 3.0.2)
project(ug_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  ug_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros ug_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 状态转换节点
add_executable(state_converter_node src/state_converter_node.cpp)
target_link_libraries(state_converter_node ${catkin_LIBRARIES})
add_dependencies(state_converter_node ${catkin_EXPORTED_TARGETS})

# PID 控制节点
add_executable(control_node src/pid_controller.cpp src/control_node.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES})
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})

install(TARGETS state_converter_node control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/ launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
