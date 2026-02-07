/**
 * @file BuoyancyEnginePlugin.hh
 * @brief 水下滑翔机浮力引擎（油囊）仿真插件
 *
 * 基于物理的浮力引擎插件，通过控制油囊体积来产生浮力差，驱动滑翔机升沉。
 *
 * 物理原理：
 *   F_buoyancy = rho * g * V
 *   delta_F = rho * g * (V_current - V_neutral)
 */

#ifndef UG_GAZEBO_PLUGINS_BUOYANCY_ENGINE_PLUGIN_HH_
#define UG_GAZEBO_PLUGINS_BUOYANCY_ENGINE_PLUGIN_HH_

#include <string>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ug_gazebo_plugins/BuoyancyEngineStatus.h>

namespace gazebo
{

class BuoyancyEnginePlugin : public ModelPlugin
{
public:
  /// 构造函数
  BuoyancyEnginePlugin();

  /// 析构函数
  virtual ~BuoyancyEnginePlugin();

  /// 加载插件
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// 重置插件
  virtual void Reset();

protected:
  /// 更新回调（每个仿真步调用）
  virtual void OnUpdate(const common::UpdateInfo &_info);

  /// ROS回调：接收体积指令
  void OnCmdVolume(const std_msgs::Float64::ConstPtr &_msg);

  /// 发布状态消息
  void PublishStatus();

private:
  /// Gazebo 模型指针
  physics::ModelPtr model_;

  /// 施加力的目标链接
  physics::LinkPtr link_;

  /// Gazebo 更新事件连接
  event::ConnectionPtr updateConnection_;

  /// ROS 节点句柄
  std::unique_ptr<ros::NodeHandle> rosNode_;

  /// ROS 订阅者：体积指令
  ros::Subscriber cmdVolumeSub_;

  /// ROS 发布者：状态
  ros::Publisher statusPub_;

  /// ROS 发布者：施加的力（调试用）
  ros::Publisher forcePub_;

  /// 上次更新时间
  common::Time lastUpdateTime_;

  // ===== 物理参数 =====

  /// 流体密度 [kg/m^3]
  double fluidDensity_;

  /// 重力加速度 [m/s^2]
  double gravity_;

  /// 中性浮力油囊体积（此时浮力=重力）[m^3]
  double neutralVolume_;

  /// 油囊最小体积 [m^3]
  double minVolume_;

  /// 油囊最大体积 [m^3]
  double maxVolume_;

  /// 油泵流速（体积变化速率）[m^3/s]
  double pumpRate_;

  /// 油囊位置（相对于link坐标系）[m]
  ignition::math::Vector3d bladderPosition_;

  // ===== 状态变量 =====

  /// 当前油囊体积 [m^3]
  double currentVolume_;

  /// 目标油囊体积 [m^3]
  double targetVolume_;

  /// 油泵方向：1=充油, -1=排油, 0=停止
  int pumpDirection_;

  /// 话题命名空间
  std::string namespace_;
};

// 向Gazebo注册插件
GZ_REGISTER_MODEL_PLUGIN(BuoyancyEnginePlugin)

}  // namespace gazebo

#endif  // UG_GAZEBO_PLUGINS_BUOYANCY_ENGINE_PLUGIN_HH_
