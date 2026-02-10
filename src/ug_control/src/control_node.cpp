/**
 * control_node.cpp
 * 滑翔机 PID 控制节点（层级3）
 * 并联三路 PID：
 *   Pitch控制环: error(目标Pitch - 实际Pitch) → 电池位置
 *   Heading控制环: error(目标Heading - 实际Heading) → 舵角
 *   Depth控制环: error(目标深度 - 实际深度) → 油囊体积
 * 订阅:
 *   /{ns}/glider_state       (GliderState, 来自状态转换节点)
 *   /{ns}/actuator_state     (ActuatorState, 来自硬件层反馈)
 *   /{ns}/cmd/pitch          (Float64, 目标俯仰角 [rad])
 *   /{ns}/cmd/heading        (Float64, 目标航向 [rad])
 *   /{ns}/cmd/depth          (Float64, 目标深度 [m])
 * 发布:
 *   /{ns}/actuator_cmd       (ActuatorCmd, 发给硬件层)
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ug_msgs/GliderState.h>
#include <ug_msgs/ActuatorCmd.h>
#include <ug_msgs/ActuatorState.h>
#include <ug_control/pid_controller.h>
#include <cmath>

class ControlNode
{
public:
  ControlNode()
  {
    ros::NodeHandle nh("~");
    std::string ns = nh.param<std::string>("namespace", "ug_glider");//命名空间
    ros::NodeHandle nhNs(ns);

    // 加载 PID 参数
    double kp, ki, kd;

    nh.param("pitch/kp", kp, 0.5);
    nh.param("pitch/ki", ki, 0.0);
    nh.param("pitch/kd", kd, 0.1);
    pitchPID_.setGains(kp, ki, kd);
    pitchPID_.setOutputLimits(-0.03385, 0.03385);  // 电池位置范围
    pitchPID_.setIntegralLimits(0.02);

    nh.param("heading/kp", kp, 0.3);
    nh.param("heading/ki", ki, 0.0);
    nh.param("heading/kd", kd, 0.05);
    headingPID_.setGains(kp, ki, kd);
    headingPID_.setOutputLimits(-0.52, 0.52);  // 舵角范围
    headingPID_.setIntegralLimits(0.3);

    nh.param("depth/kp", kp, 0.001);
    nh.param("depth/ki", ki, 0.0);
    nh.param("depth/kd", kd, 0.0);
    depthPID_.setGains(kp, ki, kd);
    depthPID_.setOutputLimits(0.0, 0.001);  // 油囊体积范围
    depthPID_.setIntegralLimits(0.0005);

    // 中性油量偏置
    neutralVolume_ = nh.param("depth/neutral_volume", 0.0005);

    // 控制频率
    controlRate_ = nh.param("control_rate", 20.0);

    // 订阅
    stateSub_ = nhNs.subscribe("glider_state", 1, &ControlNode::onGliderState, this);
    actuatorStateSub_ = nhNs.subscribe("actuator_state", 1, &ControlNode::onActuatorState, this);
    cmdPitchSub_ = nhNs.subscribe("cmd/pitch", 1, &ControlNode::onCmdPitch, this);
    cmdHeadingSub_ = nhNs.subscribe("cmd/heading", 1, &ControlNode::onCmdHeading, this);
    cmdDepthSub_ = nhNs.subscribe("cmd/depth", 1, &ControlNode::onCmdDepth, this);

    // 发布
    cmdPub_ = nhNs.advertise<ug_msgs::ActuatorCmd>("actuator_cmd", 1);

    // 定时器
    controlTimer_ = nh_.createTimer(
        ros::Duration(1.0 / controlRate_), &ControlNode::controlLoop, this);

    targetPitch_ = 0.0;
    targetHeading_ = 0.0;
    targetDepth_ = 0.0;
    stateReceived_ = false;

    ROS_INFO("[Control] PID 控制节点启动, 频率: %.0f Hz", controlRate_);
  }

private:
  void onGliderState(const ug_msgs::GliderState::ConstPtr &msg)
  {
    currentState_ = *msg;
    stateReceived_ = true;
  }

  void onActuatorState(const ug_msgs::ActuatorState::ConstPtr &msg)
  {
    currentActuator_ = *msg;
  }

  void onCmdPitch(const std_msgs::Float64::ConstPtr &msg)
  {
    targetPitch_ = msg->data;
  }

  void onCmdHeading(const std_msgs::Float64::ConstPtr &msg)
  {
    targetHeading_ = msg->data;
  }

  void onCmdDepth(const std_msgs::Float64::ConstPtr &msg)
  {
    targetDepth_ = msg->data;
  }

  void controlLoop(const ros::TimerEvent &event)
  {
    if (!stateReceived_)
      return;

    double dt = 1.0 / controlRate_;

    ug_msgs::ActuatorCmd cmd;
    cmd.header.stamp = ros::Time::now();

    // Pitch 调电池位置
    double pitchError = targetPitch_ - currentState_.pitch;
    cmd.battery_position = pitchPID_.compute(pitchError, dt);

    // Heading → 舵角
    // 航向误差需要处理 ±π 环绕
    double headingError = targetHeading_ - currentState_.yaw;
    while (headingError > M_PI) headingError -= 2.0 * M_PI;
    while (headingError < -M_PI) headingError += 2.0 * M_PI;
    cmd.rudder_angle = headingPID_.compute(headingError, dt);

    // Depth → 油囊体积
    // 深度误差：正值 = 需要更深 = 需要减小油量（增加负浮力）
    double depthError = targetDepth_ - currentState_.depth;
    // PID 输出作为偏置量，叠加在中性油量上
    // 深度正误差 → 需要下沉 → 减小油量
    double volumeOffset = -depthPID_.compute(depthError, dt);
    cmd.ballast_volume = neutralVolume_ + volumeOffset;
    cmd.ballast_volume = std::max(0.0, std::min(0.001, cmd.ballast_volume));

    cmdPub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber stateSub_, actuatorStateSub_;
  ros::Subscriber cmdPitchSub_, cmdHeadingSub_, cmdDepthSub_;
  ros::Publisher cmdPub_;
  ros::Timer controlTimer_;

  ug_control::PIDController pitchPID_, headingPID_, depthPID_;

  ug_msgs::GliderState currentState_;
  ug_msgs::ActuatorState currentActuator_;

  double targetPitch_, targetHeading_, targetDepth_;
  double neutralVolume_;
  double controlRate_;
  bool stateReceived_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_node");
  ControlNode node;
  ros::spin();
  return 0;
}
