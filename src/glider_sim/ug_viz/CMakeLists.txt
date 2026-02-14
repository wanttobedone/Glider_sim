cmake_minimum_required(VERSION 3.0.2)
project(ug_viz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  ug_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros ug_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# C++ 可视化节点
add_executable(glider_viz_node src/glider_viz_node.cpp)
target_link_libraries(glider_viz_node ${catkin_LIBRARIES})
add_dependencies(glider_viz_node ${catkin_EXPORTED_TARGETS})

# 安装
install(TARGETS glider_viz_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resource/plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS
  scripts/glider_dashboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
