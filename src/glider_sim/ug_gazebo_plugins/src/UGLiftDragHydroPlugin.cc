/**
 * 水下滑翔机流场耦合升阻力插件实现
 * 工作流，1. 取 link 世界速度减去洋流速度 = 相对流速
 *   2. 转局部坐标系，求攻角 alpha
 *   3. tanh 失速混合得 Cl, Cd
 *   4. 升力垂直于来流（在翼面平面内），阻力平行于来流反方向
 *   5. 在压力中心 CP 施力
 *   6. 发布调试话题
 *  mutex目的是使用RAII机制避免死锁
 */

#include <ug_gazebo_plugins/UGLiftDragHydroPlugin.hh>
#include <cmath>

namespace gazebo
{

UGLiftDragHydroPlugin::UGLiftDragHydroPlugin()
  : fluidDensity_(1028.0),
    wingArea_(0.035),
    cpOffset_(0, 0, 0),
    forward_(1, 0, 0),
    upward_(0, 0, 1),
    cla_(6.28),
    claStall_(1.2),
    cda_(0.02),
    cdaStall_(1.0),
    alphaStall_(0.3),
    stallK_(15.0),
    controlJointRadToCl_(0.0),
    currentTopic_("hydrodynamics/current_velocity"),
    velDeadband_(1e-3)
{
}

UGLiftDragHydroPlugin::~UGLiftDragHydroPlugin()
{
  if (rosNode_)
    rosNode_->shutdown();
}

void UGLiftDragHydroPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;

  //   解析命名空间  
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->Get<std::string>("namespace");
  else
    namespace_ = model_->GetName();

  //   解析 link  
  std::string linkName;
  if (_sdf->HasElement("link_name"))
    linkName = _sdf->Get<std::string>("link_name");
  else
  {
    gzerr << "[UGLiftDragHydro] 缺少 <link_name> 参数\n";
    return;
  }

  link_ = model_->GetLink(linkName);
  if (!link_)
  {
    // 尝试带命名空间的完整名称
    link_ = model_->GetLink(namespace_ + "/" + linkName);
  }
  if (!link_)
  {
    gzerr << "[UGLiftDragHydro] 找不到 link: " << linkName << "\n";
    return;
  }

  //   解析可选的控制关节（舵面） 
  if (_sdf->HasElement("control_joint_name"))
  {
    std::string jointName = _sdf->Get<std::string>("control_joint_name");
    controlJoint_ = model_->GetJoint(jointName);
    if (!controlJoint_)
      controlJoint_ = model_->GetJoint(namespace_ + "/" + jointName);
    if (!controlJoint_)
      gzwarn << "[UGLiftDragHydro] 找不到控制关节: " << jointName << "，忽略舵面控制\n";
  }

  //   解析物理参数  
  if (_sdf->HasElement("fluid_density"))
    fluidDensity_ = _sdf->Get<double>("fluid_density");

  if (_sdf->HasElement("area"))
    wingArea_ = _sdf->Get<double>("area");

  if (_sdf->HasElement("cp"))
    cpOffset_ = _sdf->Get<ignition::math::Vector3d>("cp");

  if (_sdf->HasElement("forward"))
  {
    forward_ = _sdf->Get<ignition::math::Vector3d>("forward");
    forward_.Normalize();
  }

  if (_sdf->HasElement("upward"))
  {
    upward_ = _sdf->Get<ignition::math::Vector3d>("upward");
    upward_.Normalize();
  }

  // 升力参数
  if (_sdf->HasElement("cla"))
    cla_ = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cla_stall"))
    claStall_ = _sdf->Get<double>("cla_stall");

  // 阻力参数
  if (_sdf->HasElement("cda"))
    cda_ = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cda_stall"))
    cdaStall_ = _sdf->Get<double>("cda_stall");

  // 失速参数
  if (_sdf->HasElement("alpha_stall"))
    alphaStall_ = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("stall_k"))
    stallK_ = _sdf->Get<double>("stall_k");

  // 舵面增益
  if (_sdf->HasElement("control_joint_rad_to_cl"))
    controlJointRadToCl_ = _sdf->Get<double>("control_joint_rad_to_cl");

  // 洋流话题
  if (_sdf->HasElement("current_velocity_topic"))
    currentTopic_ = _sdf->Get<std::string>("current_velocity_topic");

  // 死区阈值，如果速度极小（趋近于0），直接return，避免后续的atan2计算攻角会产生数值爆炸或高频震荡
  if (_sdf->HasElement("vel_deadband"))
    velDeadband_ = _sdf->Get<double>("vel_deadband");

  //   初始化 ROS  
  if (!ros::isInitialized())
  {
    gzerr << "[UGLiftDragHydro] ROS 未初始化\n";
    return;
  }

  rosNode_.reset(new ros::NodeHandle(namespace_));

  // 订阅洋流 (使用 Twist，与 UUV 插件一致)
  currentSub_ = rosNode_->subscribe(
      currentTopic_, 1, &UGLiftDragHydroPlugin::OnCurrentReceived, this);

  // 调试话题：发布升阻力
  std::string pluginName = "liftdrag_hydro";
  if (_sdf->HasElement("plugin_name"))
    pluginName = _sdf->Get<std::string>("plugin_name");

  debugForcePub_ = rosNode_->advertise<geometry_msgs::WrenchStamped>(
      pluginName + "/debug_force", 1);

  //   绑定物理更新回调  
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&UGLiftDragHydroPlugin::OnUpdate, this, std::placeholders::_1));

  gzmsg << "[UGLiftDragHydro] 插件加载完成 link=" << link_->GetName()
        << ", area=" << wingArea_
        << ", cla=" << cla_
        << ", cda=" << cda_
        << ", alpha_stall=" << alphaStall_
        << ", stall_k=" << stallK_
        << ", current_topic=" << currentTopic_
        << "\n";
}

void UGLiftDragHydroPlugin::OnCurrentReceived(
    const geometry_msgs::Twist::ConstPtr &_msg)
{
  std::lock_guard<std::mutex> lock(currentMutex_);
  currentVelocityBuffer_.Set(
      _msg->linear.x, _msg->linear.y, _msg->linear.z);
}

void UGLiftDragHydroPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  if (!link_)
    return;

  // 安全读取洋流
  ignition::math::Vector3d currentVel;
  {
    std::lock_guard<std::mutex> lock(currentMutex_);
    currentVel = currentVelocityBuffer_;
  }

  // 获取 link 世界速度和位姿 (必须在物理线程)
  const ignition::math::Vector3d linkVelWorld = link_->WorldLinearVel();
  const ignition::math::Pose3d linkPose = link_->WorldPose();

  // 相对流速，世界系
  const ignition::math::Vector3d relVelWorld = linkVelWorld - currentVel;//等于世界系下绝对速度减去洋流速度

  // 奇异点保护
  const double speed = relVelWorld.Length();
  if (speed < velDeadband_)
    return;

  // 转换到局部坐标系
  const ignition::math::Vector3d relVelLocal = linkPose.Rot().RotateVectorReverse(relVelWorld);

  // 将局部速度投影到 forward/upward 定义的翼面坐标系
  // forward = 弦长方向, upward = 翼面法向
  // span = forward × upward (展向)
  const ignition::math::Vector3d spanAxis = forward_.Cross(upward_);

  const double velForward = relVelLocal.Dot(forward_);
  const double velUpward = relVelLocal.Dot(upward_);
  const double velSpan = relVelLocal.Dot(spanAxis);

  // 在弦长平面内的速度（用于攻角计算）
  const double velInPlane = std::sqrt(velForward * velForward + velUpward * velUpward);

  if (velInPlane < velDeadband_)
    return;

  // 攻角
  double alpha = std::atan2(-velUpward, velForward);

  // 如果有控制关节（舵面），则叠加舵偏角对攻角的贡献
  double controlAngle = 0.0;
  if (controlJoint_)
  {
    controlAngle = controlJoint_->Position(0);
  }//保证随舵角变攻角也变

  // 失速混合
  const double fAlpha = 0.5 + 0.5 * std::tanh(stallK_ * (std::fabs(alpha) - alphaStall_));

  // 升力系数，线性区加上失速区混合
  double cl = (1.0 - fAlpha) * (cla_ * alpha) + fAlpha * (claStall_ * std::sin(2.0 * alpha));

  // 舵面控制：舵偏角直接叠加升力系数增量
  if (controlJoint_)
  {
    cl += controlJointRadToCl_ * controlAngle;
  }

  // 阻力系数，零升阻力加上失速后的形状阻力
  // 失速后使用平板阻力模型: Cd = Cd_stall * sin^2(alpha)
  const double cd = (1.0 - fAlpha) * cda_ + fAlpha * (cdaStall_ * std::sin(alpha) * std::sin(alpha));

  // 动压和力的大小
  const double dynPressure = 0.5 * fluidDensity_ * speed * speed;
  const double liftMag = dynPressure * wingArea_ * cl;
  const double dragMag = dynPressure * wingArea_ * std::fabs(cd);

  // 构造力方向，在局部坐标系
  // 阻力为来流反方向
  const ignition::math::Vector3d dragDirLocal = -relVelLocal.Normalized();

  // 升力垂直于来流，在翼面平面内 (forward-upward 平面)
  // liftDir = dragDir × spanAxis，确保升力在翼面平面内
  ignition::math::Vector3d liftDirLocal = dragDirLocal.Cross(spanAxis);
  const double liftDirLen = liftDirLocal.Length();
  if (liftDirLen < 1e-6)
    return;
  liftDirLocal /= liftDirLen;

  // 合力，局部坐标系
  const ignition::math::Vector3d forceLocal = liftDirLocal * liftMag + dragDirLocal * dragMag;

  // 转回世界坐标系
  const ignition::math::Vector3d forceWorld =
      linkPose.Rot().RotateVector(forceLocal);

  // 在压力中心 (CP) 施加力
  link_->AddForceAtRelativePosition(forceWorld, cpOffset_);

  // 调试，发布力信息，按需发布
  if (debugForcePub_.getNumSubscribers() > 0)
  {
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = link_->GetName();

    // force 分量：x = lift, y = drag, z = total
    // 用 wrench.force 存局部坐标系下的力
    msg.wrench.force.x = liftMag;       // 升力大小 [N]
    msg.wrench.force.y = dragMag;       // 阻力大小 [N]
    msg.wrench.force.z = speed;         // 相对流速 [m/s]

    // torque 分量存攻角等调试信息
    msg.wrench.torque.x = alpha;        // 攻角 [rad]
    msg.wrench.torque.y = cl;           // 升力系数
    msg.wrench.torque.z = cd;           // 阻力系数

    debugForcePub_.publish(msg);
  }
}

}  // namespace gazebo
