/**
 *  RViz 可视化节点
 *   广播 world -> {ns}/base_link TF (来自 ground truth Odometry)
 *   发布可视化 MarkerArray (姿态/速度/目标深度/水面/目标航向)
 *   发布状态文字 Marker
 *   发布实际轨迹 Path
 *
 * 订阅话题，namespace现在为ug_glider:
 *   /{ns}/ground_truth/pose  (Odometry)
 *   /{ns}/glider_state        (GliderState)
 *   /{ns}/actuator_state      (ActuatorState)
 *   /{ns}/cmd/depth           (Float64)
 *   /{ns}/cmd/heading         (Float64)
 *
 * 发布话题:
 *   /{ns}/viz/markers         (MarkerArray)
 *   /{ns}/viz/status_text     (Marker)状态文字
 *   /{ns}/viz/actual_path     (Path)实际轨迹
 *   /{ns}/viz/target_path     (Path)目标轨迹，后续控制器实现轨迹跟踪
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ug_msgs/GliderState.h>
#include <ug_msgs/ActuatorState.h>

#include <cmath>
#include <sstream>
#include <iomanip>

class GliderVizNode
{
public:
  GliderVizNode()
  {
    ros::NodeHandle nh("~");

    nh.param<std::string>("namespace", ns_, "ug_glider");
    nh.param<double>("viz_rate", vizRate_, 10.0);
    nh.param<int>("max_path_points", maxPathPoints_, 5000);

    // 订阅者
    poseSub_ = nh_.subscribe("/" + ns_ + "/ground_truth/pose", 1,
                             &GliderVizNode::onPose, this);
    stateSub_ = nh_.subscribe("/" + ns_ + "/glider_state", 1,
                              &GliderVizNode::onGliderState, this);
    actuatorSub_ = nh_.subscribe("/" + ns_ + "/actuator_state", 1,
                                 &GliderVizNode::onActuatorState, this);
    depthSub_ = nh_.subscribe("/" + ns_ + "/cmd/depth", 1,
                              &GliderVizNode::onCmdDepth, this);
    headingSub_ = nh_.subscribe("/" + ns_ + "/cmd/heading", 1,
                                &GliderVizNode::onCmdHeading, this);

    // 发布者
    markerPub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/" + ns_ + "/viz/markers", 1);
    textPub_ = nh_.advertise<visualization_msgs::Marker>(
        "/" + ns_ + "/viz/status_text", 1);
    actualPathPub_ = nh_.advertise<nav_msgs::Path>(
        "/" + ns_ + "/viz/actual_path", 1);
    targetPathPub_ = nh_.advertise<nav_msgs::Path>(
        "/" + ns_ + "/viz/target_path", 1);

    // 轨迹初始化
    actualPath_.header.frame_id = "world";
    targetPath_.header.frame_id = "world";

    // 定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / vizRate_),
                             &GliderVizNode::publishViz, this);

    ROS_INFO("rviz可视化, 频率: %.0f Hz, 命名空间: %s",
             vizRate_, ns_.c_str());
  }

private:
  // 回调函数 

  void onPose(const nav_msgs::Odometry::ConstPtr &msg)
  {
    pose_ = *msg;
    hasPose_ = true;

    // 广播 TF: world -> {ns}/base_link
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "world";
    tf.child_frame_id = ns_ + "/base_link";
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation = msg->pose.pose.orientation;
    tfBroadcaster_.sendTransform(tf);

    // 轨迹累积，降采样，每5条消息取1条
    pathCounter_++;
    if (pathCounter_ % 5 == 0)
    {
      geometry_msgs::PoseStamped ps;
      ps.header.stamp = msg->header.stamp;
      ps.header.frame_id = "world";
      ps.pose = msg->pose.pose;
      actualPath_.poses.push_back(ps);
      if (static_cast<int>(actualPath_.poses.size()) > maxPathPoints_)
      {
        actualPath_.poses.erase(
            actualPath_.poses.begin(),
            actualPath_.poses.begin() +
                (actualPath_.poses.size() - maxPathPoints_));
      }
    }
  }

  void onGliderState(const ug_msgs::GliderState::ConstPtr &msg)
  {
    gliderState_ = *msg;
    hasGliderState_ = true;
  }

  void onActuatorState(const ug_msgs::ActuatorState::ConstPtr &msg)
  {
    actuatorState_ = *msg;
    hasActuatorState_ = true;
  }

  void onCmdDepth(const std_msgs::Float64::ConstPtr &msg)
  {
    targetDepth_ = msg->data;
  }

  void onCmdHeading(const std_msgs::Float64::ConstPtr &msg)
  {
    targetHeading_ = msg->data;
  }

  // 四元数旋转向量辅助函数

  void quatRotate(double qw, double qx, double qy, double qz,
                  double vx, double vy, double vz,
                  double &ox, double &oy, double &oz)
  {
    // t = 2 * cross(q_xyz, v)
    double tx = 2.0 * (qy * vz - qz * vy);
    double ty = 2.0 * (qz * vx - qx * vz);
    double tz = 2.0 * (qx * vy - qy * vx);
    // result = v + w * t + cross(q_xyz, t)
    ox = vx + qw * tx + (qy * tz - qz * ty);
    oy = vy + qw * ty + (qz * tx - qx * tz);
    oz = vz + qw * tz + (qx * ty - qy * tx);
  }

  // 可视化发布

  void publishViz(const ros::TimerEvent &)
  {
    if (!hasPose_)
      return;

    ros::Time now = ros::Time::now();
    visualization_msgs::MarkerArray markers;

    const auto &pos = pose_.pose.pose.position;
    const auto &ori = pose_.pose.pose.orientation;
    std::string baseFrame = ns_ + "/base_link";

    // 1姿态箭头（红色，base_link X方向 = 机头朝向）
    {
      visualization_msgs::Marker att;
      att.header.frame_id = baseFrame;
      att.header.stamp = now;
      att.ns = "attitude";
      att.id = 0;
      att.type = visualization_msgs::Marker::ARROW;
      att.action = visualization_msgs::Marker::ADD;
      att.pose.orientation.w = 1.0;

      geometry_msgs::Point p0, p1;
      p0.x = 0; p0.y = 0; p0.z = 0;
      p1.x = 1.5; p1.y = 0; p1.z = 0;
      att.points.push_back(p0);
      att.points.push_back(p1);

      att.scale.x = 0.03;  // 箭杆直径
      att.scale.y = 0.05;  // 箭头直径
      att.scale.z = 0.05;
      att.color.r = 1.0; att.color.g = 0.2; att.color.b = 0.2; att.color.a = 0.9;
      att.lifetime = ros::Duration(0.5);
      markers.markers.push_back(att);
    }

    // 速度箭头（绿色，world坐标系，长度正比速度 1m/s=2m）
    {
      double vbx = pose_.twist.twist.linear.x;
      double vby = pose_.twist.twist.linear.y;
      double vbz = pose_.twist.twist.linear.z;

      // 机体速度转世界速度
      double vwx, vwy, vwz;
      quatRotate(ori.w, ori.x, ori.y, ori.z, vbx, vby, vbz, vwx, vwy, vwz);
      double speed = std::sqrt(vwx * vwx + vwy * vwy + vwz * vwz);

      if (speed > 0.01)
      {
        visualization_msgs::Marker vel;
        vel.header.frame_id = "world";
        vel.header.stamp = now;
        vel.ns = "velocity";
        vel.id = 1;
        vel.type = visualization_msgs::Marker::ARROW;
        vel.action = visualization_msgs::Marker::ADD;

        double scale = 2.0;
        geometry_msgs::Point p0, p1;
        p0.x = pos.x; p0.y = pos.y; p0.z = pos.z;
        p1.x = pos.x + vwx * scale;
        p1.y = pos.y + vwy * scale;
        p1.z = pos.z + vwz * scale;
        vel.points.push_back(p0);
        vel.points.push_back(p1);

        vel.scale.x = 0.04;
        vel.scale.y = 0.08;
        vel.scale.z = 0.08;
        vel.color.r = 0.2; vel.color.g = 1.0; vel.color.b = 0.2; vel.color.a = 0.9;
        vel.lifetime = ros::Duration(0.5);
        markers.markers.push_back(vel);
      }
    }

    // 目标深度指示（半透明蓝色圆盘，world z = -target_depth）
    {
      visualization_msgs::Marker td;
      td.header.frame_id = "world";
      td.header.stamp = now;
      td.ns = "target_depth";
      td.id = 2;
      td.type = visualization_msgs::Marker::CYLINDER;
      td.action = visualization_msgs::Marker::ADD;
      td.pose.position.x = pos.x;
      td.pose.position.y = pos.y;
      td.pose.position.z = -targetDepth_;
      td.pose.orientation.w = 1.0;
      td.scale.x = 3.0; td.scale.y = 3.0; td.scale.z = 0.02;
      td.color.r = 0.3; td.color.g = 0.5; td.color.b = 1.0; td.color.a = 0.3;
      td.lifetime = ros::Duration(0.5);
      markers.markers.push_back(td);
    }

    // 水面参考，z=0 半透明青色平面，跟随滑翔机水平位置
    {
      visualization_msgs::Marker sf;
      sf.header.frame_id = "world";
      sf.header.stamp = now;
      sf.ns = "water_surface";
      sf.id = 3;
      sf.type = visualization_msgs::Marker::CUBE;
      sf.action = visualization_msgs::Marker::ADD;
      sf.pose.position.x = pos.x;
      sf.pose.position.y = pos.y;
      sf.pose.position.z = 0.0;
      sf.pose.orientation.w = 1.0;
      sf.scale.x = 200.0; sf.scale.y = 200.0; sf.scale.z = 0.01;
      sf.color.r = 0.0; sf.color.g = 0.8; sf.color.b = 0.9; sf.color.a = 0.15;
      sf.lifetime = ros::Duration(0.5);
      markers.markers.push_back(sf);
    }

    // 目标航向箭头，黄色，从滑翔机位置指向目标航向方向
    {
      double hx = std::cos(targetHeading_);
      double hy = std::sin(targetHeading_);

      visualization_msgs::Marker hd;
      hd.header.frame_id = "world";
      hd.header.stamp = now;
      hd.ns = "target_heading";
      hd.id = 4;
      hd.type = visualization_msgs::Marker::ARROW;
      hd.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point p0, p1;
      p0.x = pos.x; p0.y = pos.y; p0.z = pos.z;
      p1.x = pos.x + hx * 2.0;
      p1.y = pos.y + hy * 2.0;
      p1.z = pos.z;
      hd.points.push_back(p0);
      hd.points.push_back(p1);

      hd.scale.x = 0.01;
      hd.scale.y = 0.025;
      hd.scale.z = 0.05;
      hd.color.r = 1.0; hd.color.g = 1.0; hd.color.b = 0.2; hd.color.a = 0.7;
      hd.lifetime = ros::Duration(0.5);
      markers.markers.push_back(hd);
    }

    markerPub_.publish(markers);

    // 状态文字叠加，白色，显示在机器人上方
    if (hasGliderState_)
    {
      const auto &gs = gliderState_;
      double spd = std::sqrt(gs.surge * gs.surge +
                             gs.sway * gs.sway +
                             gs.heave * gs.heave);

      std::ostringstream oss;
      oss << std::fixed;
      oss << "Depth: " << std::setprecision(2) << gs.depth
          << " m  Target: " << std::setprecision(2) << targetDepth_ << " m\n";
      oss << "Pitch: " << std::setprecision(1) << (gs.pitch * 180.0 / M_PI)
          << " deg  Roll: " << std::setprecision(1) << (gs.roll * 180.0 / M_PI) << " deg\n";
      oss << "Heading: " << std::setprecision(1) << (gs.yaw * 180.0 / M_PI)
          << " deg  Target: " << std::setprecision(1) << (targetHeading_ * 180.0 / M_PI) << " deg\n";
      oss << "Speed: " << std::setprecision(3) << spd
          << " m/s  Surge: " << std::setprecision(3) << gs.surge;

      if (hasActuatorState_)
      {
        const auto &a = actuatorState_;
        oss << "\nBat: " << std::setprecision(1) << (a.battery_position * 1000.0)
            << "mm  Oil: " << std::setprecision(0) << (a.ballast_volume * 1e6)
            << "cc  Rud: " << std::setprecision(1) << (a.rudder_angle * 180.0 / M_PI) << "deg";
      }

      visualization_msgs::Marker tm;
      tm.header.frame_id = baseFrame;
      tm.header.stamp = now;
      tm.ns = "status_text";
      tm.id = 10;
      tm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      tm.action = visualization_msgs::Marker::ADD;
      tm.pose.position.x = 0;
      tm.pose.position.y = 0;
      tm.pose.position.z = 0.5;
      tm.pose.orientation.w = 1.0;
      tm.scale.z = 0.15;
      tm.color.r = 1.0; tm.color.g = 1.0; tm.color.b = 1.0; tm.color.a = 0.95;
      tm.text = oss.str();
      tm.lifetime = ros::Duration(0.5);
      textPub_.publish(tm);
    }

    // 发布轨迹
    actualPath_.header.stamp = now;
    actualPathPub_.publish(actualPath_);
    targetPath_.header.stamp = now;
    targetPathPub_.publish(targetPath_);
  }

  //成员变量
  ros::NodeHandle nh_;
  std::string ns_;
  double vizRate_ = 10.0;
  int maxPathPoints_ = 5000;

  // 状态存储
  nav_msgs::Odometry pose_;
  ug_msgs::GliderState gliderState_;
  ug_msgs::ActuatorState actuatorState_;
  double targetDepth_ = 0.0;
  double targetHeading_ = 0.0;
  bool hasPose_ = false;
  bool hasGliderState_ = false;
  bool hasActuatorState_ = false;
  int pathCounter_ = 0;

  // 轨迹累积
  nav_msgs::Path actualPath_;
  nav_msgs::Path targetPath_;

  // ROS 通信
  ros::Subscriber poseSub_, stateSub_, actuatorSub_, depthSub_, headingSub_;
  ros::Publisher markerPub_, textPub_, actualPathPub_, targetPathPub_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "glider_viz_node");
  GliderVizNode node;
  ros::spin();
  return 0;
}
