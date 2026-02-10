/**
 * @file BuoyancyEnginePlugin.cc
 * @brief 水下滑翔机浮力引擎 - 纯物理层实现
 *
 * 重构说明：泵动力学（速率限制）已移至 ug_hardware_sim。
 * 本插件仅负责：接收体积 → 计算浮力 → 在油囊位置施力。
 */

#include <ug_gazebo_plugins/BuoyancyEnginePlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
BuoyancyEnginePlugin::BuoyancyEnginePlugin()
    : fluidDensity_(1028.0),
      gravity_(9.81),
      neutralVolume_(0.0005),
      minVolume_(0.0),
      maxVolume_(0.001),
      bladderPosition_(0, 0, 0),
      currentVolume_(0.0005),
      namespace_("")
{
}

/////////////////////////////////////////////////
BuoyancyEnginePlugin::~BuoyancyEnginePlugin()
{
  if (this->updateConnection_)
    this->updateConnection_.reset();
  if (this->rosNode_)
    this->rosNode_->shutdown();
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model_ = _model;

  // 命名空间
  if (_sdf->HasElement("namespace"))
    this->namespace_ = _sdf->Get<std::string>("namespace");
  else
    this->namespace_ = this->model_->GetName();

  // 获取目标 link
  std::string linkName = "base_link";
  if (_sdf->HasElement("link_name"))
    linkName = _sdf->Get<std::string>("link_name");

  this->link_ = this->model_->GetLink(linkName);
  if (!this->link_)
    this->link_ = this->model_->GetLink(this->namespace_ + "/" + linkName);

  if (!this->link_)
  {
    gzerr << "[BuoyancyEngine] 错误：找不到链接 '" << linkName << "'" << std::endl;
    for (auto &link : this->model_->GetLinks())
      gzerr << "  - " << link->GetName() << std::endl;
    return;
  }
  gzmsg << "[BuoyancyEngine] 绑定到链接: " << this->link_->GetName() << std::endl;

  // 读取物理参数
  if (_sdf->HasElement("fluid_density"))
    this->fluidDensity_ = _sdf->Get<double>("fluid_density");

  if (_sdf->HasElement("neutral_volume"))
    this->neutralVolume_ = _sdf->Get<double>("neutral_volume");

  if (_sdf->HasElement("min_volume"))
    this->minVolume_ = _sdf->Get<double>("min_volume");

  if (_sdf->HasElement("max_volume"))
    this->maxVolume_ = _sdf->Get<double>("max_volume");

  if (_sdf->HasElement("initial_volume"))
    this->currentVolume_ = _sdf->Get<double>("initial_volume");
  else
    this->currentVolume_ = this->neutralVolume_;

  if (_sdf->HasElement("bladder_position"))
    this->bladderPosition_ = _sdf->Get<ignition::math::Vector3d>("bladder_position");

  gzmsg << "[BuoyancyEngine] 中性体积: " << this->neutralVolume_ * 1e6 << " cc" << std::endl;
  gzmsg << "[BuoyancyEngine] 体积范围: " << this->minVolume_ * 1e6
        << " ~ " << this->maxVolume_ * 1e6 << " cc" << std::endl;
  gzmsg << "[BuoyancyEngine] 油囊位置: ("
        << this->bladderPosition_.X() << ", "
        << this->bladderPosition_.Y() << ", "
        << this->bladderPosition_.Z() << ") m" << std::endl;

  // 初始化 ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "buoyancy_engine_plugin", ros::init_options::NoSigintHandler);
  }
  this->rosNode_.reset(new ros::NodeHandle(this->namespace_));

  // 订阅体积设定话题（由 ug_hardware_sim 发布，已经过动力学滤波）
  std::string cmdTopic = "buoyancy_engine/set_volume";
  if (_sdf->HasElement("command_topic"))
    cmdTopic = _sdf->Get<std::string>("command_topic");

  this->volumeSub_ = this->rosNode_->subscribe(
      cmdTopic, 1, &BuoyancyEnginePlugin::OnSetVolume, this);
  gzmsg << "[BuoyancyEngine] 订阅体积话题: /" << this->namespace_ << "/" << cmdTopic << std::endl;

  // 发布状态
  std::string statusTopic = "buoyancy_engine/status";
  if (_sdf->HasElement("status_topic"))
    statusTopic = _sdf->Get<std::string>("status_topic");

  this->statusPub_ = this->rosNode_->advertise<ug_gazebo_plugins::BuoyancyEngineStatus>(
      statusTopic, 1);

  this->forcePub_ = this->rosNode_->advertise<geometry_msgs::WrenchStamped>(
      "buoyancy_engine/applied_force", 1);

  // 注册更新回调
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyEnginePlugin::OnUpdate, this, std::placeholders::_1));

  gzmsg << "[BuoyancyEngine] 插件加载完成（纯物理层模式）" << std::endl;
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::Reset()
{
  this->currentVolume_ = this->neutralVolume_;
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // 计算浮力差：delta_F = rho * g * (V_current - V_neutral)
  double deltaVolume = this->currentVolume_ - this->neutralVolume_;
  double buoyancyForce = this->fluidDensity_ * this->gravity_ * deltaVolume;

  // 在油囊位置施加垂直力（+Z 向上）
  ignition::math::Vector3d forceWorld(0, 0, buoyancyForce);
  this->link_->AddForceAtRelativePosition(forceWorld, this->bladderPosition_);

  // 发布状态
  this->PublishStatus();

  // 发布力（调试用）
  geometry_msgs::WrenchStamped forceMsg;
  forceMsg.header.stamp = ros::Time::now();
  forceMsg.header.frame_id = this->link_->GetName();
  forceMsg.wrench.force.z = buoyancyForce;
  this->forcePub_.publish(forceMsg);
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::OnSetVolume(const std_msgs::Float64::ConstPtr &_msg)
{
  // 直接设定体积（限幅保护）
  this->currentVolume_ = std::max(this->minVolume_,
                                   std::min(this->maxVolume_, _msg->data));
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::PublishStatus()
{
  ug_gazebo_plugins::BuoyancyEngineStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = this->link_->GetName();

  msg.current_volume = this->currentVolume_;
  msg.neutral_volume = this->neutralVolume_;

  if (this->maxVolume_ > this->minVolume_)
  {
    msg.volume_percent = 100.0 * (this->currentVolume_ - this->minVolume_)
                         / (this->maxVolume_ - this->minVolume_);
  }
  else
  {
    msg.volume_percent = 50.0;
  }

  double deltaVolume = this->currentVolume_ - this->neutralVolume_;
  msg.buoyancy_force = this->fluidDensity_ * this->gravity_ * deltaVolume;

  this->statusPub_.publish(msg);
}

}  // namespace gazebo
