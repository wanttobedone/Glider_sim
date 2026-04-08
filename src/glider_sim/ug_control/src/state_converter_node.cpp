/**
 * 状态转换节点：Gazebo ground truth (ENU) → 滑翔机状态 (NED)
 * 后续传感器节点加入后，完善状态估计节点，这个文件作为输入输出接口
 * 订阅 /ground_truth/pose (nav_msgs/Odometry, Gazebo P3D 输出, ENU 世界帧)
 * 发布 /{ns}/glider_state (ug_msgs/GliderState, NED 坐标系)
 *
 * 坐标系说明
 *   Gazebo world 帧 = ENU: X=East, Y=North, Z=Up
 *   (由 uuv_assistants/publish_world_ned_frame.launch 的 world→world_ned 变换确认)
 *
 * 位置 ENU → NED:
 *   north = y_enu,  east = x_enu,  depth = -z_enu
 *
 * 姿态 ENU → NED (完整旋转变换):
 *   R_ned = R_w^n · R_b^w · T_b
 *   其中:
 *     R_b^w  = Gazebo 四元数对应旋转矩阵 (body → ENU world)
 *     R_w^n  = [0,1,0; 1,0,0; 0,0,-1]  (ENU world → NED world)
 *     T_b    = diag(1,-1,-1)            (ROS body X前Y左Z上 → NED body X前Y右Z下)
 *   再从 R_ned 提取 ZYX 欧拉角 (roll, pitch, yaw)
 *
 * 线速度 ROS body → NED body:
 *   surge = vx,  sway = -vy,  heave = -vz
 *
 * 角速度 ROS body → NED body:
 *   roll_rate = ωx,  pitch_rate = -ωy,  yaw_rate = -ωz
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ug_msgs/GliderState.h>
#include <clocale>

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

    // 位置: ENU (X=East, Y=North, Z=Up) → NED (North, East, Down)
    state.north = msg->pose.pose.position.y;
    state.east  = msg->pose.pose.position.x;
    state.depth = -msg->pose.pose.position.z;

    // 姿态: 完整旋转变换 R_ned = R_w^n · R_b^w · T_b
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 R_bw(q);

    // R_ned = R_w^n · R_b^w · T_b
    // 其中 R_w^n = [0,1,0; 1,0,0; 0,0,-1], T_b = diag(1,-1,-1)
    //
    // 分两步展开:
    // Step A: M = R_b^w · T_b (右乘 T_b = 第1、2列取反)
    //   M[i][0] =  R_bw[i][0]
    //   M[i][1] = -R_bw[i][1]
    //   M[i][2] = -R_bw[i][2]
    //
    // Step B: R_ned = R_w^n · M (左乘 R_w^n = 行重排)
    //   R_ned row0 = M row1
    //   R_ned row1 = M row0
    //   R_ned row2 = -M row2
    tf2::Matrix3x3 R_ned;
    R_ned.setValue(
         R_bw[1][0], -R_bw[1][1], -R_bw[1][2],   // row0 = M row1
         R_bw[0][0], -R_bw[0][1], -R_bw[0][2],   // row1 = M row0
        -R_bw[2][0],  R_bw[2][1],  R_bw[2][2]);  // row2 = -M row2

    double roll, pitch, yaw;
    R_ned.getRPY(roll, pitch, yaw);

    state.roll  = roll;
    state.pitch = pitch;
    state.yaw   = yaw;

    // 线速度: ROS body (X前Y左Z上) → NED body (X前Y右Z下)
    state.surge =  msg->twist.twist.linear.x;
    state.sway  = -msg->twist.twist.linear.y;
    state.heave = -msg->twist.twist.linear.z;

    // 角速度: ROS body → NED body
    state.roll_rate  =  msg->twist.twist.angular.x;
    state.pitch_rate = -msg->twist.twist.angular.y;
    state.yaw_rate   = -msg->twist.twist.angular.z;

    statePub_.publish(state);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber poseSub_;
  ros::Publisher statePub_;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "state_converter_node");
  StateConverterNode node;
  ros::spin();
  return 0;
}
