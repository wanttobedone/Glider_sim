/**
 * 导航状态机，执行器
 *
 * 状态机：IDLE，DIVING与CLIMBING循环，ARRIVED zigzag海洋剖面覆盖循环
 *         LOITERING，停发指令，执行机构中立位，由 comm_agent 触发
 *         SURFACING， depth=0 全力上浮，由 comm_agent 触发
 *
 * IDLE 手动模式下通过 dynamic_reconfigure 设定 target_depth 直接控制深度
 * LOITERING/SURFACING 状态下拒绝 planner 的新航点
 *
 * 参数定义
 *   ~namespace           (string)  命名空间，单机 ug_glider，多机 ug_glider_0/1
 *   ~dive_depth          (double)  锯齿下潜深度 [m]
 *   ~climb_depth         (double)  锯齿上浮深度 [m]
 *   ~acceptance_radius   (double)  航点到达判定半径 [m]
 *   ~depth_threshold     (double)  深度切换阈值 [m]
 *   ~mission_rate        (double)  状态机更新频率 [Hz]
 *
 * 订阅话题
 *   /{ns}/glider_state          (GliderState)   当前 NED 状态
 *   /{ns}/mission/target        (PoseStamped)   目标航点 (world/ENU: x=east, y=north)
 *   /{ns}/move_base_simple/goal (PoseStamped)   RViz 2D Nav Goal
 *   /{ns}/mission/mode          (UInt8)         comm_agent 模式指令，4=LOITERING, 5=SURFACING
 *
 * 发布话题
 *   /{ns}/cmd/depth             (Float64)       深度指令
 *   /{ns}/cmd/heading           (Float64)       航向指令
 *   /{ns}/mission/state         (MissionState)  任务状态，供 viz 和 planner 使用
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <ug_msgs/GliderState.h>
#include <ug_msgs/MissionState.h>
#include <dynamic_reconfigure/server.h>
#include <ug_control/MissionConfig.h>
#include <cmath>
#include <clocale>

class MissionNode
{
public:
  MissionNode()
  {
    ros::NodeHandle nh("~");
    std::string ns = nh.param<std::string>("namespace", "ug_glider");
    ros::NodeHandle nhNs(ns);

    // 加载参数
    nh.param("dive_depth", diveDepth_, 30.0);
    nh.param("climb_depth", climbDepth_, 2.0);
    nh.param("acceptance_radius", acceptanceRadius_, 20.0);
    nh.param("depth_threshold", depthThreshold_, 1.0);
    double rate = nh.param("mission_rate", 1.0);

    // 订阅
    stateSub_ = nhNs.subscribe("glider_state", 1,
                                &MissionNode::onGliderState, this);
    targetSub_ = nhNs.subscribe("mission/target", 1,
                                 &MissionNode::onTarget, this);
    // 多机模式下订阅带 namespace 的话题，避免多机冲突
    navGoalSub_ = nhNs.subscribe("move_base_simple/goal", 1,
                                 &MissionNode::onNavGoal, this);

    // comm_agent_node 的模式指令
    modeSub_ = nhNs.subscribe("mission/mode", 1,
                               &MissionNode::onModeCmd, this);

    // 发布
    depthPub_ = nhNs.advertise<std_msgs::Float64>("cmd/depth", 1);
    headingPub_ = nhNs.advertise<std_msgs::Float64>("cmd/heading", 1);
    missionStatePub_ = nhNs.advertise<ug_msgs::MissionState>("mission/state", 1);

    // Dynamic Reconfigure
    dynServer_.setCallback(
        boost::bind(&MissionNode::onDynReconfig, this, _1, _2));

    // 定时器
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate),
                              &MissionNode::update, this);

    ROS_INFO("[Mission] 任务节点启动, 频率: %.1f Hz, ns: %s", rate, ns.c_str());
    ROS_INFO("[Mission] dive=%.1fm, climb=%.1fm, radius=%.1fm",
             diveDepth_, climbDepth_, acceptanceRadius_);
  }

private:
  enum State { IDLE = 0, DIVING = 1, CLIMBING = 2, ARRIVED = 3,
               LOITERING = 4, SURFACING = 5 };

  void onGliderState(const ug_msgs::GliderState::ConstPtr &msg)
  {
    state_ = *msg;
    hasState_ = true;
  }

  void onTarget(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    // LOITERING/SURFACING 下拒绝 planner 的新航点
    if (missionState_ == LOITERING || missionState_ == SURFACING)
      return;

    targetNorth_ = msg->pose.position.y;  // ENU Y = North
    targetEast_ = msg->pose.position.x;   // ENU X = East
    hasTarget_ = true;
    startMission();
  }

  void onNavGoal(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (missionState_ == LOITERING || missionState_ == SURFACING)
      return;

    targetNorth_ = msg->pose.position.y;  // ENU Y = North
    targetEast_ = msg->pose.position.x;   // ENU X = East
    hasTarget_ = true;
    startMission();
    ROS_INFO("[Mission] 2D Nav Goal: north=%.1f, east=%.1f",
             targetNorth_, targetEast_);
  }

  void onModeCmd(const std_msgs::UInt8::ConstPtr &msg)
  {
    if (msg->data == 4 && missionState_ != LOITERING)
    {
      missionState_ = LOITERING;
      ROS_WARN("[Mission] 切换 LOITERING, 执行机构中立位");
    }
    else if (msg->data == 5 && missionState_ != SURFACING)
    {
      missionState_ = SURFACING;
      ROS_WARN("[Mission] 切换 SURFACING (depth → 0)");
    }
  }

  void onDynReconfig(ug_control::MissionConfig &config, uint32_t level)
  {
    diveDepth_ = config.dive_depth;
    climbDepth_ = config.climb_depth;
    acceptanceRadius_ = config.acceptance_radius;
    depthThreshold_ = config.depth_threshold;

    // 取消任务
    if (config.cancel_mission)
    {
      if (missionState_ != IDLE)
      {
        ROS_INFO("[Mission] 任务取消");
        missionState_ = IDLE;
        hasTarget_ = false;
      }
      config.cancel_mission = false;
    }

    // IDLE模式下直接深度控制
    if (missionState_ == IDLE && config.target_depth > 0.01)
    {
      std_msgs::Float64 depthMsg;
      depthMsg.data = config.target_depth;
      depthPub_.publish(depthMsg);
      ROS_INFO("[Mission] 直接深度指令: %.1f m", config.target_depth);
    }

    ROS_INFO("[Mission] 参数更新: dive=%.1f, climb=%.1f, radius=%.1f, threshold=%.1f",
             diveDepth_, climbDepth_, acceptanceRadius_, depthThreshold_);
  }

  void startMission()
  {
    if (!hasState_)
    {
      ROS_WARN("[Mission] 未收到glider_state, 无法启动任务");
      return;
    }

    missionState_ = DIVING;

    double dx = targetNorth_ - state_.north;
    double dy = targetEast_ - state_.east;
    double dist = std::sqrt(dx * dx + dy * dy);

    ROS_INFO("[Mission] 任务启动: target=(%.1f, %.1f), dist=%.1f m",
             targetNorth_, targetEast_, dist);
    ROS_INFO("[Mission] zigzag: dive=%.1fm, climb=%.1fm, state=DIVING",
             diveDepth_, climbDepth_);
  }

  void update(const ros::TimerEvent &)
  {
    if (!hasState_)
      return;

    // 发布任务状态 (每次都发, viz需要)
    publishMissionState();

    if (missionState_ == IDLE || missionState_ == ARRIVED)
      return;

    // LOITERING：下发当前深度和航向，PID 误差归零，执行机构回中立位
    if (missionState_ == LOITERING)
    {
      std_msgs::Float64 depthMsg;
      depthMsg.data = state_.depth;
      depthPub_.publish(depthMsg);

      std_msgs::Float64 headingMsg;
      headingMsg.data = state_.yaw;
      headingPub_.publish(headingMsg);
      return;
    }

    // SURFACING，全力上浮（comm_agent 触发）
    if (missionState_ == SURFACING)
    {
      std_msgs::Float64 depthMsg;
      depthMsg.data = 0.0;
      depthPub_.publish(depthMsg);

      std_msgs::Float64 headingMsg;
      headingMsg.data = state_.yaw;
      headingPub_.publish(headingMsg);
      return;
    }

    // 计算到目标的距离和航向
    double dx = targetNorth_ - state_.north;
    double dy = targetEast_ - state_.east;
    double dist = std::sqrt(dx * dx + dy * dy);
    double heading = std::atan2(dy, dx);

    // 持续更新航向
    std_msgs::Float64 headingMsg;
    headingMsg.data = heading;
    headingPub_.publish(headingMsg);

    std_msgs::Float64 depthMsg;

    // 到达判定
    if (dist < acceptanceRadius_)
    {
      missionState_ = ARRIVED;
      depthMsg.data = state_.depth;  // 下发当前深度，保持悬停
      depthPub_.publish(depthMsg);
      ROS_INFO("[Mission] 到达目标 dist=%.1fm < radius=%.1fm", dist, acceptanceRadius_);
      return; // 立即退出本轮控制流，不再执行下潜/上浮指令
    }

    switch (missionState_)
    {
    case DIVING:
      depthMsg.data = diveDepth_;
      depthPub_.publish(depthMsg);

      if (state_.depth >= diveDepth_ - depthThreshold_)
      {
        missionState_ = CLIMBING;
        ROS_INFO("[Mission] 到达潜深 %.1fm, 切换CLIMBING, dist=%.1fm", state_.depth, dist);
      }
      break;

    case CLIMBING:
      depthMsg.data = climbDepth_;
      depthPub_.publish(depthMsg);

      if (state_.depth <= climbDepth_ + depthThreshold_)
      {
        // 只要浮到水面且没进上面的到达判定，就必定开启下一轮下潜
        missionState_ = DIVING;
        ROS_INFO("[Mission] 上浮到 %.1fm, 切换DIVING, dist=%.1fm", state_.depth, dist);
      }
      break;

    default:
      break;
    }
  }

  void publishMissionState()
  {
    ug_msgs::MissionState msg;
    msg.header.stamp = ros::Time::now();
    msg.state = static_cast<uint8_t>(missionState_);
    msg.target_north = targetNorth_;
    msg.target_east = targetEast_;
    msg.dive_depth = diveDepth_;
    msg.climb_depth = climbDepth_;
    msg.acceptance_radius = acceptanceRadius_;

    if (hasTarget_ && hasState_)
    {
      double dx = targetNorth_ - state_.north;
      double dy = targetEast_ - state_.east;
      msg.distance = std::sqrt(dx * dx + dy * dy);
    }

    missionStatePub_.publish(msg);
  }

  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber stateSub_, targetSub_, navGoalSub_, modeSub_;
  ros::Publisher depthPub_, headingPub_, missionStatePub_;
  ros::Timer timer_;
  dynamic_reconfigure::Server<ug_control::MissionConfig> dynServer_;

  // 状态
  ug_msgs::GliderState state_;
  bool hasState_ = false;
  bool hasTarget_ = false;
  State missionState_ = IDLE;

  // 目标
  double targetNorth_ = 0.0;
  double targetEast_ = 0.0;

  // 参数
  double diveDepth_ = 30.0;
  double climbDepth_ = 2.0;
  double acceptanceRadius_ = 20.0;
  double depthThreshold_ = 1.0;

};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "mission_node");
  MissionNode node;
  ros::spin();
  return 0;
}
