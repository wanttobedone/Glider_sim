/**
 * @file BuoyancyEnginePlugin.cc
 * @brief 水下滑翔机浮力引擎（油囊）仿真插件实现
 * 目前没有判断在不在水下的逻辑，意味着油囊一直会施加力
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
      pumpRate_(0.0001),
      bladderPosition_(0, 0, 0),
      currentVolume_(0.0005),
      targetVolume_(0.0005),
      pumpDirection_(0),
      namespace_("")
{
}

/////////////////////////////////////////////////
BuoyancyEnginePlugin::~BuoyancyEnginePlugin()
{
  // 断开更新事件连接
  if (this->updateConnection_)
  {
    this->updateConnection_.reset();
  }
  // 关闭ROS节点
  if (this->rosNode_)
  {
    this->rosNode_->shutdown();
  }
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // 保存模型指针
  this->model_ = _model;

  // 获取命名空间（用于ROS话题）
  if (_sdf->HasElement("namespace"))
  {
    this->namespace_ = _sdf->Get<std::string>("namespace");
  }
  else
  {
    this->namespace_ = this->model_->GetName();
  }

  // 获取目标link（使用model->GetLink避免命名空间问题）
  std::string linkName = "base_link";
  if (_sdf->HasElement("link_name"))
  {
    linkName = _sdf->Get<std::string>("link_name");
  }

  this->link_ = this->model_->GetLink(linkName);
  if (!this->link_)
  {
    // 尝试带命名空间的名称
    this->link_ = this->model_->GetLink(this->namespace_ + "/" + linkName);
  }

  if (!this->link_)
  {
    gzerr << "[BuoyancyEngine] 错误：找不到链接 '" << linkName << "'" << std::endl;
    gzerr << "[BuoyancyEngine] 可用的链接：" << std::endl;
    for (auto &link : this->model_->GetLinks())
    {
      gzerr << "  - " << link->GetName() << std::endl;
    }
    return;
  }

  gzmsg << "[BuoyancyEngine] 成功绑定到链接: " << this->link_->GetName() << std::endl;

  // 读取物理参数

  // 流体密度
  if (_sdf->HasElement("fluid_density"))
  {
    this->fluidDensity_ = _sdf->Get<double>("fluid_density");
  }
  gzmsg << "[BuoyancyEngine] 流体密度: " << this->fluidDensity_ << " kg/m^3" << std::endl;

  // 中性浮力体积
  if (_sdf->HasElement("neutral_volume"))
  {
    this->neutralVolume_ = _sdf->Get<double>("neutral_volume");
  }
  gzmsg << "[BuoyancyEngine] 中性体积: " << this->neutralVolume_ * 1e6 << " cc" << std::endl;

  // 最小体积
  if (_sdf->HasElement("min_volume"))
  {
    this->minVolume_ = _sdf->Get<double>("min_volume");
  }

  // 最大体积
  if (_sdf->HasElement("max_volume"))
  {
    this->maxVolume_ = _sdf->Get<double>("max_volume");
  }
  gzmsg << "[BuoyancyEngine] 体积范围: " << this->minVolume_ * 1e6
        << " ~ " << this->maxVolume_ * 1e6 << " cc" << std::endl;

  // 油泵流速
  if (_sdf->HasElement("pump_rate"))
  {
    this->pumpRate_ = _sdf->Get<double>("pump_rate");
  }
  gzmsg << "[BuoyancyEngine] 油泵流速: " << this->pumpRate_ * 1e6 << " cc/s" << std::endl;

  // 初始体积
  if (_sdf->HasElement("initial_volume"))
  {
    this->currentVolume_ = _sdf->Get<double>("initial_volume");
    this->targetVolume_ = this->currentVolume_;
  }
  else
  {
    this->currentVolume_ = this->neutralVolume_;
    this->targetVolume_ = this->neutralVolume_;
  }

  // 油囊位置
  if (_sdf->HasElement("bladder_position"))
  {
    this->bladderPosition_ = _sdf->Get<ignition::math::Vector3d>("bladder_position");
  }
  gzmsg << "[BuoyancyEngine] 油囊位置: ("
        << this->bladderPosition_.X() << ", "
        << this->bladderPosition_.Y() << ", "
        << this->bladderPosition_.Z() << ") m" << std::endl;

  // 初始化ROS

  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "buoyancy_engine_plugin", ros::init_options::NoSigintHandler);
  }

  this->rosNode_.reset(new ros::NodeHandle(this->namespace_));

  // 订阅体积指令话题
  std::string cmdTopic = "buoyancy_engine/cmd_volume";
  if (_sdf->HasElement("command_topic"))
  {
    cmdTopic = _sdf->Get<std::string>("command_topic");
  }
  this->cmdVolumeSub_ = this->rosNode_->subscribe(
      cmdTopic, 1, &BuoyancyEnginePlugin::OnCmdVolume, this);
  gzmsg << "[BuoyancyEngine] 订阅指令话题: /" << this->namespace_ << "/" << cmdTopic << std::endl;

  // 发布状态话题
  std::string statusTopic = "buoyancy_engine/status";
  if (_sdf->HasElement("status_topic"))
  {
    statusTopic = _sdf->Get<std::string>("status_topic");
  }
  this->statusPub_ = this->rosNode_->advertise<ug_gazebo_plugins::BuoyancyEngineStatus>(
      statusTopic, 1);

  // 发布力话题（调试用）
  this->forcePub_ = this->rosNode_->advertise<geometry_msgs::WrenchStamped>(
      "buoyancy_engine/applied_force", 1);

  // 注册更新回调

  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyEnginePlugin::OnUpdate, this, std::placeholders::_1));

  this->lastUpdateTime_ = this->model_->GetWorld()->SimTime();

  gzmsg << "[BuoyancyEngine] 插件加载完成！" << std::endl;
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::Reset()
{
  // 重置到中性体积
  this->currentVolume_ = this->neutralVolume_;
  this->targetVolume_ = this->neutralVolume_;
  this->pumpDirection_ = 0;
  this->lastUpdateTime_ = this->model_->GetWorld()->SimTime();
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // 计算时间步长
  double dt = (_info.simTime - this->lastUpdateTime_).Double();
  this->lastUpdateTime_ = _info.simTime;

  if (dt <= 0.0)
    return;

  // 模拟油泵动态

  const double tolerance = 1e-9;  // 体积容差

  if (this->currentVolume_ < this->targetVolume_ - tolerance)
  {
    // 需要充油（增加浮力，趋向上浮）
    this->currentVolume_ += this->pumpRate_ * dt;
    this->currentVolume_ = std::min(this->currentVolume_, this->targetVolume_);
    this->currentVolume_ = std::min(this->currentVolume_, this->maxVolume_);
    this->pumpDirection_ = 1;
  }
  else if (this->currentVolume_ > this->targetVolume_ + tolerance)
  {
    // 需要排油（减少浮力，趋向下沉）
    this->currentVolume_ -= this->pumpRate_ * dt;
    this->currentVolume_ = std::max(this->currentVolume_, this->targetVolume_);
    this->currentVolume_ = std::max(this->currentVolume_, this->minVolume_);
    this->pumpDirection_ = -1;
  }
  else
  {
    this->pumpDirection_ = 0;
  }

  // 计算额外浮力

  // 浮力差 = 密度 * 重力 * (当前体积 - 中性体积)
  double deltaVolume = this->currentVolume_ - this->neutralVolume_;
  double buoyancyForce = this->fluidDensity_ * this->gravity_ * deltaVolume;

  // 力方向：+Z（向上）
  ignition::math::Vector3d forceWorld(0, 0, buoyancyForce);

  // 施加力 

  // 在油囊位置施加力（产生力矩效果）
  this->link_->AddForceAtRelativePosition(forceWorld, this->bladderPosition_);

  //发布状态

  this->PublishStatus();

  // 发布力（调试用）
  geometry_msgs::WrenchStamped forceMsg;
  forceMsg.header.stamp = ros::Time::now();
  forceMsg.header.frame_id = this->link_->GetName();
  forceMsg.wrench.force.z = buoyancyForce;
  this->forcePub_.publish(forceMsg);
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::OnCmdVolume(const std_msgs::Float64::ConstPtr &_msg)
{
  // 限制目标体积在有效范围内
  this->targetVolume_ = std::max(this->minVolume_,
                                  std::min(this->maxVolume_, _msg->data));

  ROS_DEBUG("[BuoyancyEngine] 收到体积指令: %.2f cc", this->targetVolume_ * 1e6);
}

/////////////////////////////////////////////////
void BuoyancyEnginePlugin::PublishStatus()
{
  ug_gazebo_plugins::BuoyancyEngineStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = this->link_->GetName();

  // 体积状态
  msg.current_volume = this->currentVolume_;
  msg.target_volume = this->targetVolume_;
  msg.neutral_volume = this->neutralVolume_;

  // 百分比（0%=空, 50%=中性, 100%=满）
  if (this->maxVolume_ > this->minVolume_)
  {
    msg.volume_percent = 100.0 * (this->currentVolume_ - this->minVolume_)
                         / (this->maxVolume_ - this->minVolume_);
  }
  else
  {
    msg.volume_percent = 50.0;
  }

  // 浮力
  double deltaVolume = this->currentVolume_ - this->neutralVolume_;
  msg.buoyancy_force = this->fluidDensity_ * this->gravity_ * deltaVolume;

  // 泵状态
  msg.is_at_target = (this->pumpDirection_ == 0);
  msg.is_pumping = (this->pumpDirection_ != 0);
  msg.pump_direction = this->pumpDirection_;

  this->statusPub_.publish(msg);
}

}  // namespace gazebo
