/**
 * 水下滑翔机分层洋流插件
 *
 * Gazebo ModelPlugin，挂载在滑翔机模型上，实时根据深度插值分层洋流。
 * 数据源：SDF <layer> 标签（简单模式）或 CSV 文件，这是DAVE 兼容格式，真实海洋数据模式，从网站HYCOM获取真实洋流数据与预报数据，再通过脚本转化为CSV格式文件
 *   输出： Gazebo transport (msgs::Vector3d) — 喂给 UUV Fossen 和 UG LiftDrag（物理同步）
 *   ROS TwistStamped — 给 rqt_plot 调试（20Hz）
 *   ROS MarkerArray — 给 RViz 可视化（2Hz）
 *
 * 坐标约定为，所有发布的速度均为 ENU 世界坐标系 (X=东, Y=北, Z=上)
 */

#ifndef UG_GAZEBO_PLUGINS_UG_OCEAN_CURRENT_PLUGIN_HH_
#define UG_GAZEBO_PLUGINS_UG_OCEAN_CURRENT_PLUGIN_HH_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{

class UGOceanCurrentPlugin : public ModelPlugin
{
public:
  UGOceanCurrentPlugin();
  virtual ~UGOceanCurrentPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// 深度层数据结构
  struct DepthLayer
  {
    double depth;                           // 正值，水下深度 [m]
    ignition::math::Vector3d velocity;      // ENU 世界坐标系 [m/s]
  };

  /// 物理步回调 (~1kHz)
  void OnUpdate(const common::UpdateInfo &_info);

  /// 在给定深度进行线性插值
  ignition::math::Vector3d InterpolateAtDepth(double _depth) const;

  /// 加载 SDF <layer> 标签
  void LoadLayersFromSDF(sdf::ElementPtr _sdf);

  /// 加载 CSV 文件（DAVE 兼容格式：north, east, depth）
  bool LoadLayersFromCSV(const std::string &_path);

  /// 发布 RViz 可视化 markers
  void PublishMarkers(const ignition::math::Pose3d &_pose,
                      const ignition::math::Vector3d &_currentVel,
                      double _depth);

  //   Gazebo 句柄  
  physics::ModelPtr model_;
  event::ConnectionPtr updateConnection_;

  //   Gazebo transport（物理同步通道） 
  transport::NodePtr gazeboNode_;
  transport::PublisherPtr currentPub_;

  //   ROS 通信（调试 / 可视化通道） 
  std::unique_ptr<ros::NodeHandle> rosNode_;
  ros::Publisher rosPub_;         // TwistStamped
  ros::Publisher markerPub_;      // MarkerArray

  //   数据  
  std::vector<DepthLayer> layers_;
  std::string namespace_;
  std::string flowTopic_;

  //   降频计时  
  common::Time lastRosPubTime_;
  common::Time lastMarkerPubTime_;
  static constexpr double ROS_PUB_PERIOD = 0.05;     // 20 Hz
  static constexpr double MARKER_PUB_PERIOD = 0.5;    // 2 Hz
};

GZ_REGISTER_MODEL_PLUGIN(UGOceanCurrentPlugin)

}  // namespace gazebo

#endif
