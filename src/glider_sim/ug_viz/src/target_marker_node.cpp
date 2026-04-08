/**
 * 目标点交互标记节点，未完善，先不用
 *
 * 功能:
 *   在RViz中创建可拖拽的3D Interactive Marker设定目标XY位置
 *   桥接RViz 2D Nav Goal到mission/target话题
 *   在目标点周围显示接受半径
 *
 * 订阅:
 *   /move_base_simple/goal        (PoseStamped)  RViz 2D Nav Goal
 *   /{ns}/mission/state           (MissionState) 任务状态 (用于显示半径)
 *
 * 发布:
 *   /{ns}/mission/target          (PoseStamped)  目标点
 *   /{ns}/viz/target_markers      (MarkerArray)  目标可视化
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ug_msgs/MissionState.h>
#include <cmath>
#include <clocale>

class TargetMarkerNode
{
public:
  TargetMarkerNode()
  {
    ros::NodeHandle nh("~");
    nh.param<std::string>("namespace", ns_, "ug_glider");
    ros::NodeHandle nhNs(ns_);

    // 发布
    targetPub_ = nhNs.advertise<geometry_msgs::PoseStamped>("mission/target", 1);
    vizPub_ = nhNs.advertise<visualization_msgs::MarkerArray>("viz/target_markers", 1);

    // 订阅
    // 多机模式下订阅带 namespace 的话题，避免多机冲突
    navGoalSub_ = nhNs.subscribe("move_base_simple/goal", 1,
                                 &TargetMarkerNode::onNavGoal, this);
    missionStateSub_ = nhNs.subscribe("mission/state", 1,
                                       &TargetMarkerNode::onMissionState, this);

    // Interactive Marker Server
    server_.reset(new interactive_markers::InteractiveMarkerServer(
        ns_ + "/target_marker"));

    createTargetMarker();
    server_->applyChanges();

    // 可视化定时器
    vizTimer_ = nh_.createTimer(ros::Duration(0.2),
                                 &TargetMarkerNode::publishViz, this);

    ROS_INFO("[TargetMarker] 交互标记节点启动, ns: %s", ns_.c_str());
  }

private:
  void createTargetMarker()
  {
    visualization_msgs::InteractiveMarker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.name = "target_waypoint";
    marker.description = "Drag to set target";
    marker.pose.position.x = 50.0;  // 默认位置: 北方50m
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;   // 水面
    marker.pose.orientation.w = 1.0;
    marker.scale = 3.0;

    targetX_ = 50.0;
    targetY_ = 0.0;

    // 可视化: 红色圆柱(旗杆) + 球体(旗头)
    visualization_msgs::InteractiveMarkerControl vizControl;
    vizControl.always_visible = true;

    // 旗杆 (竖直圆柱)
    visualization_msgs::Marker pole;
    pole.type = visualization_msgs::Marker::CYLINDER;
    pole.scale.x = 0.3;
    pole.scale.y = 0.3;
    pole.scale.z = 5.0;
    pole.pose.position.z = -2.5;
    pole.pose.orientation.w = 1.0;
    pole.color.r = 0.9; pole.color.g = 0.2; pole.color.b = 0.2; pole.color.a = 0.8;
    vizControl.markers.push_back(pole);

    // 旗头 (球体)
    visualization_msgs::Marker flag;
    flag.type = visualization_msgs::Marker::SPHERE;
    flag.scale.x = 1.5;
    flag.scale.y = 1.5;
    flag.scale.z = 1.5;
    flag.pose.position.z = 0.5;
    flag.pose.orientation.w = 1.0;
    flag.color.r = 1.0; flag.color.g = 0.3; flag.color.b = 0.1; flag.color.a = 0.9;
    vizControl.markers.push_back(flag);

    marker.controls.push_back(vizControl);

    // XY平面拖拽控制 (限制在水平面)
    visualization_msgs::InteractiveMarkerControl moveX;
    moveX.name = "move_x";
    moveX.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    moveX.orientation.w = 1.0;
    moveX.orientation.x = 1.0;  // X轴
    marker.controls.push_back(moveX);

    visualization_msgs::InteractiveMarkerControl moveY;
    moveY.name = "move_y";
    moveY.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    moveY.orientation.w = 1.0;
    moveY.orientation.z = 1.0;  // Y轴
    marker.controls.push_back(moveY);

    // XY平面自由拖拽
    visualization_msgs::InteractiveMarkerControl movePlane;
    movePlane.name = "move_plane";
    movePlane.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    movePlane.orientation.w = 1.0;
    movePlane.orientation.y = 1.0;  // XZ平面法向 = Y轴, 但我们要XY平面
    marker.controls.push_back(movePlane);

    server_->insert(marker,
                    boost::bind(&TargetMarkerNode::onMarkerFeedback, this, _1));

    // 右键菜单: 发送目标
    interactive_markers::MenuHandler::EntryHandle sendEntry =
        menuHandler_.insert("Send as target",
                            boost::bind(&TargetMarkerNode::onMenuSend, this, _1));
    menuHandler_.apply(*server_, "target_waypoint");
  }

  void onMarkerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (feedback->event_type ==
        visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
      targetX_ = feedback->pose.position.x;
      targetY_ = feedback->pose.position.y;

      // 固定Z在水面
      visualization_msgs::InteractiveMarker marker;
      if (server_->get("target_waypoint", marker))
      {
        marker.pose.position.z = 0.0;
        server_->setPose("target_waypoint", marker.pose);
        server_->applyChanges();
      }

      publishTarget();
      ROS_INFO("[TargetMarker] 目标设定: north=%.1f, east=%.1f",
               targetY_, targetX_);  // Y=North, X=East in ENU
    }
  }

  void onMenuSend(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    publishTarget();
    ROS_INFO("[TargetMarker] 菜单发送目标: north=%.1f, east=%.1f",
             targetY_, targetX_);  // Y=North, X=East in ENU
  }

  void onNavGoal(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    targetX_ = msg->pose.position.x;
    targetY_ = msg->pose.position.y;
    hasTarget_ = true;

    // 同步Interactive Marker位置
    visualization_msgs::InteractiveMarker marker;
    if (server_->get("target_waypoint", marker))
    {
      marker.pose.position.x = targetX_;
      marker.pose.position.y = targetY_;
      marker.pose.position.z = 0.0;
      server_->setPose("target_waypoint", marker.pose);
      server_->applyChanges();
    }

    publishTarget();
    ROS_INFO("[TargetMarker] 2D Nav Goal: north=%.1f, east=%.1f",
             targetY_, targetX_);  // Y=North, X=East in ENU
  }

  void onMissionState(const ug_msgs::MissionState::ConstPtr &msg)
  {
    missionState_ = *msg;
    hasMissionState_ = true;
  }

  void publishTarget()
  {
    geometry_msgs::PoseStamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "world";
    target.pose.position.x = targetX_;
    target.pose.position.y = targetY_;
    target.pose.position.z = 0.0;
    target.pose.orientation.w = 1.0;
    targetPub_.publish(target);
    hasTarget_ = true;
  }

  void publishViz(const ros::TimerEvent &)
  {
    if (!hasTarget_)
      return;

    ros::Time now = ros::Time::now();
    visualization_msgs::MarkerArray markers;

    double radius = 20.0;
    if (hasMissionState_)
      radius = missionState_.acceptance_radius;

    // 接受半径 (半透明绿色圆柱)
    {
      visualization_msgs::Marker ring;
      ring.header.frame_id = "world";
      ring.header.stamp = now;
      ring.ns = "acceptance_radius";
      ring.id = 100;
      ring.type = visualization_msgs::Marker::CYLINDER;
      ring.action = visualization_msgs::Marker::ADD;
      ring.pose.position.x = targetX_;
      ring.pose.position.y = targetY_;
      ring.pose.position.z = -15.0;  // 圆柱中心在水下
      ring.pose.orientation.w = 1.0;
      ring.scale.x = radius * 2.0;
      ring.scale.y = radius * 2.0;
      ring.scale.z = 30.0;  // 高度覆盖水面到下潜深度
      ring.color.r = 0.2; ring.color.g = 0.9; ring.color.b = 0.3; ring.color.a = 0.08;
      ring.lifetime = ros::Duration(0.5);
      markers.markers.push_back(ring);
    }

    // 目标点标签文字
    {
      visualization_msgs::Marker text;
      text.header.frame_id = "world";
      text.header.stamp = now;
      text.ns = "target_label";
      text.id = 101;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x = targetX_;
      text.pose.position.y = targetY_;
      text.pose.position.z = 3.0;
      text.scale.z = 1.0;
      text.color.r = 1.0; text.color.g = 0.9; text.color.b = 0.2; text.color.a = 0.9;

      char buf[128];
      snprintf(buf, sizeof(buf), "Target N:%.0f E:%.0f", targetY_, targetX_);
      text.text = buf;

      if (hasMissionState_ && missionState_.distance > 0)
      {
        char buf2[64];
        snprintf(buf2, sizeof(buf2), "\ndist: %.0f m", missionState_.distance);
        text.text += buf2;
      }

      text.lifetime = ros::Duration(0.5);
      markers.markers.push_back(text);
    }

    vizPub_.publish(markers);
  }

  // ROS
  ros::NodeHandle nh_;
  std::string ns_;
  ros::Subscriber navGoalSub_, missionStateSub_;
  ros::Publisher targetPub_, vizPub_;
  ros::Timer vizTimer_;

  // Interactive Marker
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menuHandler_;

  // 状态
  double targetX_ = 50.0;
  double targetY_ = 0.0;
  bool hasTarget_ = false;
  ug_msgs::MissionState missionState_;
  bool hasMissionState_ = false;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "target_marker_node");
  TargetMarkerNode node;
  ros::spin();
  return 0;
}
