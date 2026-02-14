/**
 *  目前误差驱动的映射逻辑写的比较简单，线性化，后续看要不要深入写，或者换控制器再写
 *   两种模式，模式0，MANUAL手动调试，并联独立 PID（原始逻辑）
 *     Pitch PID, cmd/pitch改变电池位置
 *     Depth PID,cmd/depth改油囊体积
 *   模式1级联控制，俯仰与深度解耦
 *     外环depth PID, cmd/depth 深度驱动调节目标俯仰角
 *     内环pitch PID, 目标俯仰角驱动调节电池位置
 *     油囊前馈(带死区), depth误差，调整油囊体积
 * 
 *     Heading PID, cmd/heading调整舵角
 * 订阅话题，namespace现在为ug_glider:
 *   /{ns}/glider_state       (GliderState, 来自状态转换节点)
 *   /{ns}/actuator_state     (ActuatorState, 来自硬件层反馈)
 *   /{ns}/cmd/pitch          (Float64, 目标俯仰角 [rad], 仅手动模式)
 *   /{ns}/cmd/heading        (Float64, 目标航向 [rad])
 *   /{ns}/cmd/depth          (Float64, 目标深度 [m])
 *   /{ns}/cmd/mode           (Int32, 0=MANUAL, 1=CASCADE)
 * 发布话题:  /{ns}/actuator_cmd       (ActuatorCmd, 发给硬件层)
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
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
    std::string ns = nh.param<std::string>("namespace", "ug_glider");
    ros::NodeHandle nhNs(ns);

    // 加载 PID 参数
    double kp, ki, kd;

    // 内环的Pitch PID；pitch向下为正，与电池包倾向一致
    nh.param("pitch/kp", kp, 0.5);
    nh.param("pitch/ki", ki, 0.0);
    nh.param("pitch/kd", kd, 0.1);
    pitchPID_.setGains(kp, ki, kd);
    pitchPID_.setOutputLimits(-0.03385, 0.03385);  // 电池位置范围 [m]
    pitchPID_.setIntegralLimits(0.02);

    // Heading PID参数，两种模式共用
    nh.param("heading/kp", kp, 0.3);
    nh.param("heading/ki", ki, 0.0);
    nh.param("heading/kd", kd, 0.05);
    headingPID_.setGains(kp, ki, kd);
    headingPID_.setOutputLimits(-0.52, 0.52);  // 舵角范围 [rad]
    headingPID_.setIntegralLimits(0.3);

    // 手动模式Depth PID，直接输出油囊体积的大小
    nh.param("depth/kp", kp, 0.001);
    nh.param("depth/ki", ki, 0.0);
    nh.param("depth/kd", kd, 0.0);
    depthPID_.setGains(kp, ki, kd);
    depthPID_.setOutputLimits(0.0, 0.001);  // 油囊体积范围 [m³]
    depthPID_.setIntegralLimits(0.0005);

    // 中性油量 
    neutralVolume_ = nh.param("depth/neutral_volume", 0.0005);

    // 级联模式，外环 depth PID，驱动输出目标俯仰角 [rad]
    double outerKp, outerKi, outerKd;
    nh.param("cascade/depth_outer/kp", outerKp, 0.3);
    nh.param("cascade/depth_outer/ki", outerKi, 0.01);
    nh.param("cascade/depth_outer/kd", outerKd, 0.1);
    depthOuterPID_.setGains(outerKp, outerKi, outerKd);

    nh.param("cascade/depth_outer/max_pitch", maxPitchCmd_, 0.5);  // [rad]大约30°，限制幅度
    depthOuterPID_.setOutputLimits(-maxPitchCmd_, maxPitchCmd_);
    depthOuterPID_.setIntegralLimits(maxPitchCmd_);

    // 级联控制，油囊前馈参数，线性映射误差与深度关系
    nh.param("cascade/ff_gain", ffGain_, 0.0002);       // [m³/m]
    nh.param("cascade/ff_deadzone", ffDeadzone_, 0.1);   // [m]

    // 控制模式, 0=MANUAL, 1=CASCADE
    controlMode_ = nh.param("control_mode", 1);

    // 控制频率, Hz
    controlRate_ = nh.param("control_rate", 20.0);

    // 订阅
    stateSub_ = nhNs.subscribe("glider_state", 1, &ControlNode::onGliderState, this);
    actuatorStateSub_ = nhNs.subscribe("actuator_state", 1, &ControlNode::onActuatorState, this);
    cmdPitchSub_ = nhNs.subscribe("cmd/pitch", 1, &ControlNode::onCmdPitch, this);
    cmdHeadingSub_ = nhNs.subscribe("cmd/heading", 1, &ControlNode::onCmdHeading, this);
    cmdDepthSub_ = nhNs.subscribe("cmd/depth", 1, &ControlNode::onCmdDepth, this);
    cmdModeSub_ = nhNs.subscribe("cmd/mode", 1, &ControlNode::onCmdMode, this);

    // 发布
    cmdPub_ = nhNs.advertise<ug_msgs::ActuatorCmd>("actuator_cmd", 1);

    // 定时器，执行controlLoop
    controlTimer_ = nh_.createTimer(
        ros::Duration(1.0 / controlRate_), &ControlNode::controlLoop, this);

    targetPitch_ = 0.0;
    targetHeading_ = 0.0;
    targetDepth_ = 0.0;
    stateReceived_ = false;

    ROS_INFO("[Control] 控制节点已启动, 频率: %.0f Hz, 模式: %s",
             controlRate_, controlMode_ == 1 ? "CASCADE" : "MANUAL");
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

  void onCmdMode(const std_msgs::Int32::ConstPtr &msg)
  {
    int newMode = msg->data;
    if (newMode != controlMode_)
    {
      // Bumpless transfer: reset 所有 PID 积分器和微分历史
      pitchPID_.reset();
      depthOuterPID_.reset();
      depthPID_.reset();
      controlMode_ = newMode;
      ROS_INFO("[Control] 模式切换: %s", newMode == 1 ? "CASCADE" : "MANUAL");
    }
  }

  void controlLoop(const ros::TimerEvent &event)
  {
    if (!stateReceived_)
      return;

    double dt = 1.0 / controlRate_;

    ug_msgs::ActuatorCmd cmd;
    cmd.header.stamp = ros::Time::now();

    // Heading由舵角控制
    double headingError = targetHeading_ - currentState_.yaw;
    while (headingError > M_PI) headingError -= 2.0 * M_PI;
    while (headingError < -M_PI) headingError += 2.0 * M_PI;
    cmd.rudder_angle = headingPID_.compute(headingError, dt);

    if (controlMode_ == 1)
    {
      // 级联控制器
      // 外环深度误差驱动目标俯仰角
      double depthError = targetDepth_ - currentState_.depth;
      double desiredPitch = depthOuterPID_.compute(depthError, dt);
      desiredPitch = std::max(-maxPitchCmd_, std::min(maxPitchCmd_, desiredPitch));

      // 内环俯仰误差驱动电池位置
      double pitchError = desiredPitch - currentState_.pitch;
      cmd.battery_position = pitchPID_.compute(pitchError, dt);

      // 油囊前馈，设置0.001的深度为滤波阈值，深度正误差下潜，减小油量
      double ffInput = 0.0;
      if (std::abs(depthError) > ffDeadzone_)
      {
        ffInput = depthError - std::copysign(ffDeadzone_, depthError);
      }
      double volumeOffset = -ffGain_ * ffInput;
      cmd.ballast_volume = neutralVolume_ + volumeOffset;
      cmd.ballast_volume = std::max(0.0, std::min(0.001, cmd.ballast_volume));
    }
    else
    {
      // 手动调试模式
      // Pitch由电池位置控制
      double pitchError = targetPitch_ - currentState_.pitch;
      cmd.battery_position = pitchPID_.compute(pitchError, dt);

      // Depth改油囊体积
      double depthError = targetDepth_ - currentState_.depth;
      double volumeOffset = -depthPID_.compute(depthError, dt);
      cmd.ballast_volume = neutralVolume_ + volumeOffset;
      cmd.ballast_volume = std::max(0.0, std::min(0.001, cmd.ballast_volume));
    }

    cmdPub_.publish(cmd);

    // // DEBUG，计算Munk力矩，姿态等
    // debugCounter_++;
    // if (debugCounter_ % (int)(2.0 * controlRate_) == 0)
    // {
    //   ROS_INFO("[DBG] depth=%.3f pitch=%.3f(%.1fdeg) surge=%.3f heave=%.3f pitch_rate=%.3f",
    //            currentState_.depth, currentState_.pitch,
    //            currentState_.pitch * 180.0 / M_PI,
    //            currentState_.surge, currentState_.heave,
    //            currentState_.pitch_rate);
    //   ROS_INFO("[DBG] cmd: bat=%.5f vol=%.6f(%.0fcc) rud=%.4f",
    //            cmd.battery_position, cmd.ballast_volume,
    //            cmd.ballast_volume * 1e6, cmd.rudder_angle);
    //   ROS_INFO("[DBG] act: bat=%.5f vol=%.6f rud=%.4f",
    //            currentActuator_.battery_position,
    //            currentActuator_.ballast_volume,
    //            currentActuator_.rudder_angle);
    //   double speed = std::sqrt(currentState_.surge * currentState_.surge +
    //                            currentState_.heave * currentState_.heave);
    //   // Munk moment estimate: (Ma11 - Ma33) * u * w
    //   double munk = (3.0 - 40.0) * currentState_.surge * currentState_.heave;
    //   ROS_INFO("[DBG] speed=%.3f munk_est=%.3f", speed, munk);
    // }
  }

  int debugCounter_ = 0;
  ros::NodeHandle nh_;
  ros::Subscriber stateSub_, actuatorStateSub_;
  ros::Subscriber cmdPitchSub_, cmdHeadingSub_, cmdDepthSub_, cmdModeSub_;
  ros::Publisher cmdPub_;
  ros::Timer controlTimer_;

  // PID 控制器
  ug_control::PIDController pitchPID_, headingPID_;
  ug_control::PIDController depthPID_;        // 手动模式: depth → 油囊体积
  ug_control::PIDController depthOuterPID_;   // 级联模式: depth → 目标pitch角

  ug_msgs::GliderState currentState_;
  ug_msgs::ActuatorState currentActuator_;

  double targetPitch_, targetHeading_, targetDepth_;
  double neutralVolume_;
  double controlRate_;
  bool stateReceived_;

  // 级联模式参数
  int controlMode_;       // 0=MANUAL, 1=CASCADE
  double maxPitchCmd_;    // 外环输出限幅 [rad]
  double ffGain_;         // 油囊前馈增益 [m³/m]
  double ffDeadzone_;     // 前馈死区 [m]
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_node");
  ControlNode node;
  ros::spin();
  return 0;
}
