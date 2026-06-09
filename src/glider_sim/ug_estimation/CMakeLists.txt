cmake_minimum_required(VERSION 3.0.2)
project(ug_estimation)

# ------------------------------ 编译选项 ------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ------------------------------ catkin 依赖 ----------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ug_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ug_ekf_core
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_msgs
                 tf2 tf2_ros tf2_geometry_msgs ug_msgs
  DEPENDS EIGEN3
)

# ============================================================================
# ug_ekf_core: 平台无关 ESKF 核心 (sim 与 STM32 共用)
# ----------------------------------------------------------------------------
# 严格约束：不依赖 catkin/ROS、不开异常/RTTI、Release 优化。
# 编译时若误用 ROS 头会因为缺少 catkin include 而报错。
# ============================================================================
add_library(ug_ekf_core STATIC
  src/core/eskf.cpp
)
target_include_directories(ug_ekf_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(ug_ekf_core PUBLIC
  Eigen3::Eigen
)
target_compile_options(ug_ekf_core PRIVATE
  -Wall -Wextra -Wpedantic
  -fno-exceptions -fno-rtti
)
# 故意不 link catkin_LIBRARIES：core 不该见到 ROS。

# ============================================================================
# ROS 节点：depth_converter (保留, robot_localization 仍在用)
# ============================================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_converter_node src/depth_converter_node.cpp)
target_link_libraries(depth_converter_node ${catkin_LIBRARIES})

# ============================================================================
# ROS 节点：glider_ekf (自研 ESKF wrapper)
#   link ug_ekf_core (平台无关) + catkin。core 仍不 link ROS。
# ============================================================================
add_executable(glider_ekf_node src/glider_ekf_node.cpp)
target_link_libraries(glider_ekf_node ug_ekf_core ${catkin_LIBRARIES})
add_dependencies(glider_ekf_node ${catkin_EXPORTED_TARGETS})

# ============================================================================
# 单元测试 (catkin_add_gtest)
# ============================================================================
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_quaternion test/test_quaternion.cpp)
  if(TARGET test_quaternion)
    target_link_libraries(test_quaternion ug_ekf_core)
  endif()

  catkin_add_gtest(test_imu_propagator test/test_imu_propagator.cpp)
  if(TARGET test_imu_propagator)
    target_link_libraries(test_imu_propagator ug_ekf_core)
  endif()

  catkin_add_gtest(test_depth_update test/test_depth_update.cpp)
  if(TARGET test_depth_update)
    target_link_libraries(test_depth_update ug_ekf_core)
  endif()

  catkin_add_gtest(test_accel_tilt test/test_accel_tilt.cpp)
  if(TARGET test_accel_tilt)
    target_link_libraries(test_accel_tilt ug_ekf_core)
  endif()
endif()
