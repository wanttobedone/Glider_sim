## 内容

  - **glider_sim**: Glider仿真项目文件集合
  - **uuv_simulator**: 修改后的UUV仿真器（含水动力插件）
  - **dave**: 修改后的DAVE海洋环境

  # 代码包 (Packages)

* **`ug_description` **
  滑翔机的 3D 模型文件（STL）、URDF 配置，还有 Gazebo 水下海洋环境的 Launch 启动文件。

* **`ug_hardware_sim` **
  硬件仿真 Gazebo 插件，模拟现实中物理执行单元延时

* **`ug_gazebo_plugins` **
  浮力引擎插件

* **`ug_control` **
  控制包，PID控制器，分并联和级联控制
  并联：三个PID控制器分别控制深度（油囊），pitch（电池位置），航向（尾舵）；
  级联：深度误差驱动外环油囊，同时传递到内层pitch环，航向单独由尾舵控制

* **`ug_msgs` **
  自定义的各种 ROS 消息格式（`.msg`）。比如包含所有位姿数据的 `GliderState`，和下发硬件指令用的 `ActuatorCmd`。

* **`ug_viz` **
  RViz 可视化节点

  ## 修改

  ### uuv_simulator 修改内容
  - `BuoyantObject.hh/cc` - 优化浮力计算
  - `HydrodynamicModel.cc` - 调整水动力参数
  - `thruster_proportional.py` - 推进器模型修改

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

  roslaunch ug_description full_sim.launch

  ### 许可证

  - ug_description: Apache-2.0
  - uuv_simulator: Apache-2.0 (原始仓库: https://github.com/uuvsimulator/uuv_simulator)
  - dave: Apache-2.0 (原始仓库: https://github.com/Field-Robotics-Lab/dave)

