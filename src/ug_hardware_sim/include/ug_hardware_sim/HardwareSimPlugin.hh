/**
 * @file HardwareSimPlugin.hh
 * @brief 水下滑翔机硬件抽象仿真层 - Gazebo 模型插件
 *
 * 参考 UUV FinPlugin 的运动学设定方式：
 *   SetPosition(0, pos, true) 直接设定关节位置，保留世界速度。
 *
 * 职责：
 *   1. 执行机构动力学模拟（速率限制、一阶滞后）
 *   2. SetPosition 驱动关节（电池、尾舵），preserveWorldVelocity=true
 *   3. 发布实际体积给浮力引擎插件
 *   4. 发布 /joint_states（维护 TF 树）
 *   5. 发布 ActuatorState（反馈给控制层）
 *   6. 看门狗安全保护（timeout <= 0 时禁用）
 */

#ifndef UG_HARDWARE_SIM_HARDWARE_SIM_PLUGIN_HH_
#define UG_HARDWARE_SIM_HARDWARE_SIM_PLUGIN_HH_

#include <string>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ug_msgs/ActuatorCmd.h>
#include <ug_msgs/ActuatorState.h>

namespace gazebo
{

class HardwareSimPlugin : public ModelPlugin
{
public:
  HardwareSimPlugin();
  virtual ~HardwareSimPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();

protected:
  void OnUpdate(const common::UpdateInfo &_info);
  void OnActuatorCmd(const ug_msgs::ActuatorCmd::ConstPtr &_msg);

private:
  double ApplyRateLimiter(double current, double target, double maxRate, double dt);
  double ApplyFirstOrderLag(double current, double target, double alpha);

  physics::ModelPtr model_;
  physics::JointPtr batteryJoint_;
  physics::JointPtr rudderJoint_;
  event::ConnectionPtr updateConnection_;

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber cmdSub_;
  ros::Publisher statePub_;
  ros::Publisher jointStatePub_;
  ros::Publisher volumePub_;      // -> BuoyancyEnginePlugin

  common::Time lastUpdateTime_;
  ros::Time lastCmdTime_;

  // 电池参数
  double batteryMaxSpeed_;
  double batteryMinPos_;
  double batteryMaxPos_;

  // 油囊参数
  double ballastPumpRate_;
  double ballastMinVol_;
  double ballastMaxVol_;

  // 尾舵参数
  double rudderAlpha_;
  double rudderMinAngle_;
  double rudderMaxAngle_;

  // 看门狗 (timeout <= 0 时禁用)
  double watchdogTimeout_;

  // 安全状态
  double safeBatteryPos_;
  double safeBallastVol_;
  double safeRudderAngle_;

  // 当前实际状态（经过动力学滤波）
  double currentBatteryPos_;
  double currentBallastVol_;
  double currentRudderAngle_;

  // 目标状态（来自控制层指令）
  double targetBatteryPos_;
  double targetBallastVol_;
  double targetRudderAngle_;

  std::string namespace_;
};

GZ_REGISTER_MODEL_PLUGIN(HardwareSimPlugin)

}  // namespace gazebo

#endif  // UG_HARDWARE_SIM_HARDWARE_SIM_PLUGIN_HH_
