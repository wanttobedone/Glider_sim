/**
 * 状态转换节点：Gazebo ground truth (ENU)到滑翔机状态 (NED)
 * 后续传感器节点加入后，完善状态估计节点，这个文件作为输入输出接口
 * 订阅 /ground_truth/pose (nav_msgs/Odometry, Gazebo P3D 输出)
 * 发布 /{ns}/glider_state (ug_msgs/GliderState, NED 坐标系)
 *
 * 坐标转换
 *   NED.north = ENU.x
 *   Gazebo 默认不是 ENU，而是 X=前, Y=左, Z=上。
 *   UUV simulator 在 NED 辅助节点中处理了这个，
 *   读 Gazebo 世界坐标并做映射：
 *     Gazebo world: X=North, Y=East (假设初始朝北), Z=Up
 *     NED: X=North, Y=East, Z=Down
 *   north=X, east=Y, depth=-Z
 *   roll/pitch 不变，yaw 不变（都是右手系到 NED 的映射）
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ug_msgs/GliderState.h>

class StateConverterNode
{
public:
  StateConverterNode()
  {
    ros::NodeHandle nh("~");
    ros::NodeHandle nhPub(nh.param<std::string>("namespace", "ug_glider"));

    std::string poseTopic;
    nh.param<std::string>("pose_topic", poseTopic, "/ug_glider/ground_truth/pose");

    poseSub_ = nh_.subscribe(poseTopic, 1, &StateConverterNode::onPose, this);
    statePub_ = nhPub.advertise<ug_msgs::GliderState>("glider_state", 1);

    ROS_INFO("[StateConverter] 订阅: %s", poseTopic.c_str());
    ROS_INFO("[StateConverter] 发布: /%s/glider_state", nhPub.getNamespace().c_str());
  }

  void onPose(const nav_msgs::Odometry::ConstPtr &msg)
  {
    ug_msgs::GliderState state;
    state.header.stamp = msg->header.stamp;
    state.header.frame_id = "world_ned";

    // 位置由Gazebo (X-North, Y-East, Z-Up)到NED (North, East, Down)
    state.north = msg->pose.pose.position.x;
    state.east = msg->pose.pose.position.y;
    state.depth = -msg->pose.pose.position.z;

    // 姿态由四元数变为欧拉角
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Gazebo Z-up 到 NED Z-down：pitch 和 roll 符号不变，yaw 不变
    state.roll = roll;
    state.pitch = pitch;
    state.yaw = yaw;

    // 速度（机体坐标系，直接传递）
    state.surge = msg->twist.twist.linear.x;
    state.sway = msg->twist.twist.linear.y;
    state.heave = -msg->twist.twist.linear.z;  // Z 翻转

    // 角速度（机体坐标系）
    state.roll_rate = msg->twist.twist.angular.x;
    state.pitch_rate = msg->twist.twist.angular.y;
    state.yaw_rate = -msg->twist.twist.angular.z;  // Z 翻转

    statePub_.publish(state);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber poseSub_;
  ros::Publisher statePub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_converter_node");
  StateConverterNode node;
  ros::spin();
  return 0;
}
