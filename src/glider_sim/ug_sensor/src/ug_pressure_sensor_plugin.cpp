/**
 * UG Pressure Sensor Plugin
 *
 * 简洁的 Gazebo ModelPlugin，模拟水下压力传感器。
 * 替代 UUV Simulator 的 SubseaPressureROSPlugin（有 segfault 问题）。
 *
 *   读取指定 link 的世界坐标 Z（ENU，水下为负）
 *   计算压力: P = P_atm + rho * g * depth
 *   加入高斯噪声
 *   发布 sensor_msgs/FluidPressure
 *
 * SDF 参数：
 *   <robot_namespace>  ROS 命名空间
 *   <link_name>        绑定的 link 名称
 *   <sensor_topic>     发布话题名 (默认 "pressure")
 *   <update_rate>      更新频率 Hz (默认 10)
 *   <noise_sigma>      压力噪声标准差 Pa (默认 100)
 *   <standard_pressure> 大气压 Pa (默认 101325)
 *   <water_density>    水密度 kg/m^3 (默认 1028)
 *   <gravity>          重力加速度 m/s^2 (默认 9.81)
 */

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <random>

namespace gazebo
{

class UGPressureSensorPlugin : public ModelPlugin
{
public:
  UGPressureSensorPlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    world_ = _model->GetWorld();

    // 读取 SDF 参数
    std::string ns = "ug_glider";
    if (_sdf->HasElement("robot_namespace"))
      ns = _sdf->Get<std::string>("robot_namespace");

    std::string linkName;
    if (_sdf->HasElement("link_name"))
      linkName = _sdf->Get<std::string>("link_name");
    else
    {
      gzerr << "[UGPressure] 缺少 <link_name> 参数\n";
      return;
    }

    link_ = _model->GetLink(linkName);
    if (!link_)
    {
      gzerr << "[UGPressure] 找不到 link: " << linkName << "\n";
      gzerr << "[UGPressure] 模型中可用的 links:\n";
      for (auto& l : _model->GetLinks())
        gzerr << "  - " << l->GetName() << "\n";
      return;
    }

    std::string topic = "pressure";
    if (_sdf->HasElement("sensor_topic"))
      topic = _sdf->Get<std::string>("sensor_topic");

    if (_sdf->HasElement("update_rate"))
      updateRate_ = _sdf->Get<double>("update_rate");
    if (_sdf->HasElement("noise_sigma"))
      noiseSigma_ = _sdf->Get<double>("noise_sigma");
    if (_sdf->HasElement("standard_pressure"))
      standardPressure_ = _sdf->Get<double>("standard_pressure");
    if (_sdf->HasElement("water_density"))
      waterDensity_ = _sdf->Get<double>("water_density");
    if (_sdf->HasElement("gravity"))
      gravity_ = _sdf->Get<double>("gravity");

    noiseDist_ = std::normal_distribution<double>(0.0, noiseSigma_);

    // ROS 初始化
    rosNode_.reset(new ros::NodeHandle(ns));
    pub_ = rosNode_->advertise<sensor_msgs::FluidPressure>(topic, 10);

    // 注册更新回调
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&UGPressureSensorPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[UGPressure] 插件加载完成 link=" << linkName
          << ", topic=/" << ns << "/" << topic
          << ", rate=" << updateRate_ << " Hz"
          << ", noise_sigma=" << noiseSigma_ << " Pa\n";
  }

private:
  void OnUpdate(const common::UpdateInfo& _info)
  {
    // 频率控制
    if (updateRate_ > 0.0)
    {
      double dt = (_info.simTime - lastUpdateTime_).Double();
      if (dt < 1.0 / updateRate_)
        return;
    }
    lastUpdateTime_ = _info.simTime;

    // 获取 link 世界坐标 (ENU)
    ignition::math::Vector3d pos = link_->WorldPose().Pos();

    // ENU 中 Z 为高度，水下为负。depth = -z (depth > 0 表示水下)
    double depth = std::max(0.0, -pos.Z());

    // 压力 = 大气压 + 水柱压力
    double pressure = standardPressure_ + waterDensity_ * gravity_ * depth;

    // 加噪声
    pressure += noiseDist_(rng_);

    // 发布
    sensor_msgs::FluidPressure msg;
    msg.header.stamp.sec = _info.simTime.sec;
    msg.header.stamp.nsec = _info.simTime.nsec;
    msg.header.frame_id = link_->GetName();
    msg.fluid_pressure = pressure;
    msg.variance = noiseSigma_ * noiseSigma_;

    pub_.publish(msg);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  boost::shared_ptr<ros::NodeHandle> rosNode_;
  ros::Publisher pub_;

  double updateRate_ = 10.0;
  double noiseSigma_ = 100.0;        // Pa
  double standardPressure_ = 101325.0; // Pa
  double waterDensity_ = 1028.0;      // kg/m^3
  double gravity_ = 9.81;             // m/s^2

  common::Time lastUpdateTime_;
  std::default_random_engine rng_{std::random_device{}()};
  std::normal_distribution<double> noiseDist_{0.0, 100.0};
};

GZ_REGISTER_MODEL_PLUGIN(UGPressureSensorPlugin)

}  // namespace gazebo
