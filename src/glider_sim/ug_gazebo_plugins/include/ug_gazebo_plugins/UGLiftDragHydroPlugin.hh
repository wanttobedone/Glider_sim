/**
 * 水下滑翔机流场耦合升阻力插件
 *
 * 替代 Gazebo 内置 LiftDragPlugin，支持：实时订阅洋流，基于相对流速计算升阻力；大攻角 tanh 失速混合模型；可选舵面控制关节（舵偏角叠加攻角；调试用力反馈话题
 * ROS 回调线程仅写 currentVelocityBuffer_，Gazebo 物理线程仅读，mutex 保护，能够实现线程安全
 */

#ifndef UG_GAZEBO_PLUGINS_UG_LIFTDRAG_HYDRO_PLUGIN_HH_
#define UG_GAZEBO_PLUGINS_UG_LIFTDRAG_HYDRO_PLUGIN_HH_

#include <mutex>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <ug_gazebo_plugins/LiftDragDebug.h>

#include <gazebo/gazebo.hh> //Gazebo 的核心物理引擎 API，用于获取刚体状态并施加力
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{

class UGLiftDragHydroPlugin : public ModelPlugin
{
public:
  UGLiftDragHydroPlugin();
  virtual ~UGLiftDragHydroPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Gazebo 物理步回调，线程 B，~1000 Hz，是Gazebo自己的物理步频率
  void OnUpdate(const common::UpdateInfo &_info);

  /// Gazebo transport 洋流回调，与物理引擎同步
  void OnCurrentReceived(ConstVector3dPtr &_msg);

  //   Gazebo 句柄  
  physics::ModelPtr model_;
  physics::LinkPtr link_; //指向模型中实际受力部件，智能指针
  physics::JointPtr controlJoint_;   // 可选，舵面控制关节
  event::ConnectionPtr updateConnection_;

  //   Gazebo transport（洋流输入）
  transport::NodePtr gazeboNode_;
  transport::SubscriberPtr currentSub_;

  //   ROS 通信（调试输出）
  std::unique_ptr<ros::NodeHandle> rosNode_;
  ros::Publisher debugForcePub_;     // 调试用发布升阻力

  //   线程安全的洋流缓冲  
  std::mutex currentMutex_;
  ignition::math::Vector3d currentVelocityBuffer_;  //洋流数据的暂存区

  //   SDF 参数  
  std::string namespace_;

  // 流体
  double fluidDensity_;

  // 翼面几何
  double wingArea_;
  ignition::math::Vector3d cpOffset_;     // 压力中心相对 link 质心
  ignition::math::Vector3d forward_;      // 局部前进方向（弦长方向）
  ignition::math::Vector3d upward_;       // 局部法向/展向参考方向

  // 升力系数
  double cla_;            // 线性区升力斜率 [1/rad]
  double claStall_;       // 失速后峰值升力系数

  // 阻力系数
  double cda_;            // 零升阻力系数
  double cdaStall_;       // 失速后阻力系数

  // 失速参数
  double alphaStall_;     // 临界失速角 [rad]
  double stallK_;         // tanh 过渡陡峭系数

  // 舵面控制
  double controlJointRadToCl_;  // 舵偏角，升力系数增量的增益

  // 洋流话题
  std::string currentTopic_;

  // 奇异点保护阈值
  double velDeadband_;
};

GZ_REGISTER_MODEL_PLUGIN(UGLiftDragHydroPlugin)

}  // namespace gazebo

#endif
