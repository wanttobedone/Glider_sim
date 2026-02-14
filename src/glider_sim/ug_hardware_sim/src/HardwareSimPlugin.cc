/**
 * HardwareSimPlugin.c水下滑翔机硬件抽象仿真层实现
 *
 * 数据流：
 *   ActuatorCmd (ROS) → 动力学滤波 → SetPosition(true) (Gazebo joints)
 *                                   → set_volume (BuoyancyEnginePlugin)
 *                                   → joint_states (TF)
 *                                   → ActuatorState (反馈)
 *
 * 关节用 SetPosition(0, pos, true) 运动学设定，
 * 第三参数 true = preserveWorldVelocity，避免重置子 link 世界速度。
 * 参考 UUV FinPlugin 的做法（SetPosition 运动学 + AddRelativeForce 力学）。
 */

#include <ug_hardware_sim/HardwareSimPlugin.hh>

namespace gazebo
{

/////////////////////////////////////////////////
HardwareSimPlugin::HardwareSimPlugin()
    : batteryMaxSpeed_(0.01),
      batteryMinPos_(-0.03385),
      batteryMaxPos_(0.03385),
      ballastPumpRate_(0.0001),
      ballastMinVol_(0.0),
      ballastMaxVol_(0.001),
      rudderAlpha_(0.3),
      rudderMinAngle_(-0.52),
      rudderMaxAngle_(0.52),
      watchdogTimeout_(1.0),
      safeBatteryPos_(0.0),
      safeBallastVol_(0.0005),
      safeRudderAngle_(0.0),
      currentBatteryPos_(0.0),
      currentBallastVol_(0.0005),
      currentRudderAngle_(0.0),
      targetBatteryPos_(0.0),
      targetBallastVol_(0.0005),
      targetRudderAngle_(0.0),
      namespace_("")
{
}

/////////////////////////////////////////////////
HardwareSimPlugin::~HardwareSimPlugin()
{
  if (this->updateConnection_)
    this->updateConnection_.reset();
  if (this->nh_)
    this->nh_->shutdown();
}

/////////////////////////////////////////////////
void HardwareSimPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model_ = _model;

  // 命名空间
  if (_sdf->HasElement("namespace"))
    this->namespace_ = _sdf->Get<std::string>("namespace");
  else
    this->namespace_ = this->model_->GetName();

  // 获取关节
  std::string batteryJointName = this->namespace_ + "/battery_joint";
  std::string rudderJointName = this->namespace_ + "/rudder_joint";

  if (_sdf->HasElement("battery_joint"))
    batteryJointName = _sdf->Get<std::string>("battery_joint");
  if (_sdf->HasElement("rudder_joint"))
    rudderJointName = _sdf->Get<std::string>("rudder_joint");

  this->batteryJoint_ = this->model_->GetJoint(batteryJointName);
  this->rudderJoint_ = this->model_->GetJoint(rudderJointName);

  if (!this->batteryJoint_)
  {
    gzerr << "[HardwareSim] 找不到电池关节: " << batteryJointName << std::endl;
    return;
  }
  if (!this->rudderJoint_)
  {
    gzerr << "[HardwareSim] 找不到尾舵关节: " << rudderJointName << std::endl;
    return;
  }

  gzmsg << "[HardwareSim] 电池关节: " << this->batteryJoint_->GetName() << std::endl;
  gzmsg << "[HardwareSim] 尾舵关节: " << this->rudderJoint_->GetName() << std::endl;

  // 初始化 ROS
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "hardware_sim_plugin", ros::init_options::NoSigintHandler);
  }
  this->nh_.reset(new ros::NodeHandle(this->namespace_));

  // 从参数服务器读取参数（可由 launch/yaml 覆盖）
  this->nh_->param("hardware_sim/battery/max_speed", this->batteryMaxSpeed_, 0.01);
  this->nh_->param("hardware_sim/battery/min_position", this->batteryMinPos_, -0.03385);
  this->nh_->param("hardware_sim/battery/max_position", this->batteryMaxPos_, 0.03385);
  this->nh_->param("hardware_sim/battery/initial_position", this->currentBatteryPos_, 0.0);

  this->nh_->param("hardware_sim/ballast/pump_rate", this->ballastPumpRate_, 0.0001);
  this->nh_->param("hardware_sim/ballast/min_volume", this->ballastMinVol_, 0.0);
  this->nh_->param("hardware_sim/ballast/max_volume", this->ballastMaxVol_, 0.001);
  this->nh_->param("hardware_sim/ballast/initial_volume", this->currentBallastVol_, 0.0005);

  this->nh_->param("hardware_sim/rudder/alpha", this->rudderAlpha_, 0.3);
  this->nh_->param("hardware_sim/rudder/min_angle", this->rudderMinAngle_, -0.52);
  this->nh_->param("hardware_sim/rudder/max_angle", this->rudderMaxAngle_, 0.52);
  this->nh_->param("hardware_sim/rudder/initial_angle", this->currentRudderAngle_, 0.0);

  this->nh_->param("hardware_sim/watchdog/timeout", this->watchdogTimeout_, 1.0);

  this->nh_->param("hardware_sim/safe_state/battery_position", this->safeBatteryPos_, 0.0);
  this->nh_->param("hardware_sim/safe_state/ballast_volume", this->safeBallastVol_, 0.0005);
  this->nh_->param("hardware_sim/safe_state/rudder_angle", this->safeRudderAngle_, 0.0);

  // 初始化目标 = 当前
  this->targetBatteryPos_ = this->currentBatteryPos_;
  this->targetBallastVol_ = this->currentBallastVol_;
  this->targetRudderAngle_ = this->currentRudderAngle_;

  // 设置关节初始位置
  this->batteryJoint_->SetPosition(0, this->currentBatteryPos_, true);
  this->rudderJoint_->SetPosition(0, this->currentRudderAngle_, true);

  // ROS 话题
  this->cmdSub_ = this->nh_->subscribe(
      "actuator_cmd", 1, &HardwareSimPlugin::OnActuatorCmd, this);

  this->statePub_ = this->nh_->advertise<ug_msgs::ActuatorState>(
      "actuator_state", 1);

  this->jointStatePub_ = this->nh_->advertise<sensor_msgs::JointState>(
      "joint_states", 1);

  this->volumePub_ = this->nh_->advertise<std_msgs::Float64>(
      "buoyancy_engine/set_volume", 1);

  // 初始化时间
  this->lastCmdTime_ = ros::Time::now();
  this->lastUpdateTime_ = this->model_->GetWorld()->SimTime();

  // 注册更新回调
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&HardwareSimPlugin::OnUpdate, this, std::placeholders::_1));

  gzmsg << "[HardwareSim] 电池速度限制: " << this->batteryMaxSpeed_ << " m/s" << std::endl;
  gzmsg << "[HardwareSim] 油泵速率: " << this->ballastPumpRate_ * 1e6 << " cc/s" << std::endl;
  gzmsg << "[HardwareSim] 尾舵滞后系数: " << this->rudderAlpha_ << std::endl;
  if (this->watchdogTimeout_ <= 0.0)
    gzmsg << "[HardwareSim] 看门狗: 已禁用" << std::endl;
  else
    gzmsg << "[HardwareSim] 看门狗超时: " << this->watchdogTimeout_ << " s" << std::endl;
  gzmsg << "[HardwareSim] 插件加载完成" << std::endl;
}

/////////////////////////////////////////////////
void HardwareSimPlugin::Reset()
{
  this->currentBatteryPos_ = 0.0;
  this->currentBallastVol_ = 0.0005;
  this->currentRudderAngle_ = 0.0;
  this->targetBatteryPos_ = 0.0;
  this->targetBallastVol_ = 0.0005;
  this->targetRudderAngle_ = 0.0;
  this->lastUpdateTime_ = this->model_->GetWorld()->SimTime();
  this->lastCmdTime_ = ros::Time::now();
}

/////////////////////////////////////////////////
void HardwareSimPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  double dt = (_info.simTime - this->lastUpdateTime_).Double();
  this->lastUpdateTime_ = _info.simTime;

  if (dt <= 0.0)
    return;

  // === 看门狗检查 (timeout <= 0 时禁用) ===
  if (this->watchdogTimeout_ > 0.0)
  {
    double timeSinceCmd = (ros::Time::now() - this->lastCmdTime_).toSec();
    if (timeSinceCmd > this->watchdogTimeout_)
    {
      this->targetBatteryPos_ = this->safeBatteryPos_;
      this->targetBallastVol_ = this->safeBallastVol_;
      this->targetRudderAngle_ = this->safeRudderAngle_;

      static double lastWarnTime = 0;
      if (_info.simTime.Double() - lastWarnTime > 5.0)
      {
        ROS_WARN("[HardwareSim] 看门狗触发：%.1fs 未收到指令，回归安全状态", timeSinceCmd);
        lastWarnTime = _info.simTime.Double();
      }
    }
  }

  // === 执行机构动力学 ===

  // 电池：速率限制
  this->currentBatteryPos_ = ApplyRateLimiter(
      this->currentBatteryPos_, this->targetBatteryPos_,
      this->batteryMaxSpeed_, dt);
  this->currentBatteryPos_ = std::max(this->batteryMinPos_,
                                       std::min(this->batteryMaxPos_, this->currentBatteryPos_));

  // 油囊：速率限制
  this->currentBallastVol_ = ApplyRateLimiter(
      this->currentBallastVol_, this->targetBallastVol_,
      this->ballastPumpRate_, dt);
  this->currentBallastVol_ = std::max(this->ballastMinVol_,
                                       std::min(this->ballastMaxVol_, this->currentBallastVol_));

  // 尾舵：一阶滞后
  this->currentRudderAngle_ = ApplyFirstOrderLag(
      this->currentRudderAngle_, this->targetRudderAngle_,
      this->rudderAlpha_);
  this->currentRudderAngle_ = std::max(this->rudderMinAngle_,
                                        std::min(this->rudderMaxAngle_, this->currentRudderAngle_));

  // === 驱动 Gazebo 关节 (preserveWorldVelocity = true) ===
  this->batteryJoint_->SetPosition(0, this->currentBatteryPos_, true);
  this->rudderJoint_->SetPosition(0, this->currentRudderAngle_, true);

  // === 发布实际体积给浮力引擎插件 ===
  std_msgs::Float64 volMsg;
  volMsg.data = this->currentBallastVol_;
  this->volumePub_.publish(volMsg);

  // === 发布 joint_states（维护 TF 树）===
  sensor_msgs::JointState jsMsg;
  jsMsg.header.stamp = ros::Time::now();
  jsMsg.name.push_back(this->batteryJoint_->GetName());
  jsMsg.name.push_back(this->rudderJoint_->GetName());
  jsMsg.position.push_back(this->currentBatteryPos_);
  jsMsg.position.push_back(this->currentRudderAngle_);
  jsMsg.velocity.push_back(0.0);
  jsMsg.velocity.push_back(0.0);
  jsMsg.effort.push_back(0.0);
  jsMsg.effort.push_back(0.0);
  this->jointStatePub_.publish(jsMsg);

  // === 发布 ActuatorState 反馈 ===
  ug_msgs::ActuatorState stateMsg;
  stateMsg.header.stamp = ros::Time::now();
  stateMsg.battery_position = this->currentBatteryPos_;
  stateMsg.ballast_volume = this->currentBallastVol_;
  stateMsg.rudder_angle = this->currentRudderAngle_;
  this->statePub_.publish(stateMsg);
}

/////////////////////////////////////////////////
void HardwareSimPlugin::OnActuatorCmd(const ug_msgs::ActuatorCmd::ConstPtr &_msg)
{
  this->targetBatteryPos_ = _msg->battery_position;
  this->targetBallastVol_ = _msg->ballast_volume;
  this->targetRudderAngle_ = _msg->rudder_angle;
  this->lastCmdTime_ = ros::Time::now();
}

/////////////////////////////////////////////////
double HardwareSimPlugin::ApplyRateLimiter(
    double current, double target, double maxRate, double dt)
{
  double error = target - current;
  double maxStep = maxRate * dt;

  if (std::abs(error) > maxStep)
    return current + std::copysign(maxStep, error);
  else
    return target;
}

/////////////////////////////////////////////////
double HardwareSimPlugin::ApplyFirstOrderLag(
    double current, double target, double alpha)
{
  return alpha * target + (1.0 - alpha) * current;
}

}  // namespace gazebo
