/**
 * @file BuoyancyEnginePlugin.hh
 * @brief 水下滑翔机浮力引擎（油囊）仿真插件 - 纯物理层
 * 收外部设定的油囊体积，计算并施加浮力。
 * 不含泵动力学（速率限制等由 ug_hardware_sim 负责）。
 *
 * 物理原理：
 *   delta_F = rho * g * (V_current - V_neutral)
 *   力在油囊位置施加，产生俯仰力矩。
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
  BuoyancyEnginePlugin();
  virtual ~BuoyancyEnginePlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();

protected:
  /// 每个仿真步调用：计算并施加浮力
  virtual void OnUpdate(const common::UpdateInfo &_info);

  /// ROS回调：外部设定油囊体积（已经过硬件层动力学滤波）
  void OnSetVolume(const std_msgs::Float64::ConstPtr &_msg);

  void PublishStatus();

private:
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  std::unique_ptr<ros::NodeHandle> rosNode_;
  ros::Subscriber volumeSub_;
  ros::Publisher statusPub_;
  ros::Publisher forcePub_;

  // 物理参数
  double fluidDensity_;
  double gravity_;
  double neutralVolume_;
  double minVolume_;
  double maxVolume_;
  ignition::math::Vector3d bladderPosition_;

  // 状态
  double currentVolume_;

  std::string namespace_;
};

GZ_REGISTER_MODEL_PLUGIN(BuoyancyEnginePlugin)

}  // namespace gazebo

#endif  // UG_GAZEBO_PLUGINS_BUOYANCY_ENGINE_PLUGIN_HH_
