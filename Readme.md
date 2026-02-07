## 内容

  - **ug_description**: 滑翔机URDF模型描述
  - **uuv_simulator**: 修改后的UUV仿真器（含水动力插件）
  - **dave**: 修改后的DAVE海洋环境

  ## 修改

  ### uuv_simulator 修改内容
  - `BuoyantObject.hh/cc` - 优化浮力计算
  - `HydrodynamicModel.cc` - 调整水动力参数
  - `thruster_proportional.py` - 推进器模型修改
  - `gazebo_ros_image_sonar.cpp` - 声呐插件增强

  ### dave 修改内容
  - `dave_ocean_waves.world` - 自定义海洋世界
  - `gen_world.py` - 世界生成脚本

  ## 安装使用

  ### 依赖
  ```bash
  sudo apt-get install ros-noetic-gazebo-ros \
                       ros-noetic-robot-state-publisher \
                       ros-noetic-joint-state-publisher-gui

  ### 编译

  cd glider_ws
  catkin build
  source devel/setup.bash

  ### 运行仿真

  roslaunch ug_description gazebo_dave_ocean.launch

  ### 许可证

  - ug_description: Apache-2.0
  - uuv_simulator: Apache-2.0 (原始仓库: https://github.com/uuvsimulator/uuv_simulator)
  - dave: Apache-2.0 (原始仓库: https://github.com/Field-Robotics-Lab/dave)

  ## 注意: 本仓库包含对 uuv_simulator 和 dave 的修改版本。原始项目的版权归原作者所有。
