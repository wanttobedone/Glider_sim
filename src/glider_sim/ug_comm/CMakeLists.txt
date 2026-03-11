cmake_minimum_required(VERSION 3.0.2)
project(ug_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ug_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs ug_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# 异常场探测节点
add_executable(anomaly_field_node src/anomaly_field_node.cpp)
target_link_libraries(anomaly_field_node ${catkin_LIBRARIES})
add_dependencies(anomaly_field_node ${catkin_EXPORTED_TARGETS})

# 通信代理节点（每台 Glider 一个实例）
add_executable(comm_agent_node src/comm_agent_node.cpp)
target_link_libraries(comm_agent_node ${catkin_LIBRARIES})
add_dependencies(comm_agent_node ${catkin_EXPORTED_TARGETS})

install(TARGETS anomaly_field_node comm_agent_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
