/**
 * 覆盖规划节点，多滑翔机 Boustrophedon覆盖，目前路径规划比较粗糙，演示用
 *
 * 将矩形覆盖区域按滑翔机数量均分，沿 east 方向切分，
 * 为每台生成 Boustrophedon 航点序列，依次下发并等 ARRIVED 后发下一个。
 * 监听水声通信总线，收到 TARGET_FOUND 后停止所有航点下发。
 *
 * 参数（/swarm/ 命名空间）：
 *   glider_namespaces   (string[])  滑翔机命名空间列表
 *   area_center_north   (double)    区域中心 north [m]
 *   area_center_east    (double)    区域中心 east [m]
 *   area_length         (double)    沿 north 方向长度 [m]
 *   area_width          (double)    沿 east 方向宽度 [m]
 *   swath_width         (double)    单条覆盖带宽度 [m]
 *
 * 订阅：
 *   /{ns}/mission/state        (MissionState) × N   等待 ARRIVED 触发下一航点
 *   /swarm/acoustic_channel    (SwarmComm)          监听 TARGET_FOUND 停止下发
 *
 * 发布：
 *   /{ns}/mission/target       (PoseStamped) × N   航点指令
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ug_msgs/MissionState.h>
#include <ug_msgs/SwarmComm.h>
#include <cmath>
#include <vector>
#include <string>
#include <clocale>

struct Waypoint
{
  double north;
  double east;
};

//在这里读launch 文件里的 glider_namespaces，有几个就构造几个Glider
struct GliderEntry
{
  std::string ns;
  int waypointIdx;
  bool arrived;
  bool complete;
  bool firstTargetSent;
  std::vector<Waypoint> waypoints;

  ros::Subscriber missionSub;
  ros::Publisher targetPub;
};

class SwarmPlannerNode
{
public:
  SwarmPlannerNode()
  {
    ros::NodeHandle nh("/swarm");  // 从全局 /swarm/ 命名空间读共享参数

    nh.param("area_center_north", areaCenterNorth_, 200.0);
    nh.param("area_center_east", areaCenterEast_, 0.0);
    nh.param("area_length", areaLength_, 400.0);
    nh.param("area_width", areaWidth_, 200.0);
    nh.param("swath_width", swathWidth_, 50.0);

    std::vector<std::string> namespaces;
    nh.getParam("glider_namespaces", namespaces);
    if (namespaces.empty())
    {
      ROS_ERROR("[SwarmPlanner] glider_namespaces 参数为空！");
      return;
    }

    int N = namespaces.size();
    gliders_.resize(N);

    double northMin = areaCenterNorth_ - areaLength_ / 2.0;
    double northMax = areaCenterNorth_ + areaLength_ / 2.0;
    double eastMin = areaCenterEast_ - areaWidth_ / 2.0;

    for (int k = 0; k < N; k++)
    {
      auto &g = gliders_[k];
      g.ns = namespaces[k];
      g.waypointIdx = -1;
      g.arrived = false;
      g.complete = false;
      g.firstTargetSent = false;

      // 子区域 east 范围
      double subEastMin = eastMin + k * (areaWidth_ / N);
      double subEastMax = eastMin + (k + 1) * (areaWidth_ / N);
      double subWidth = subEastMax - subEastMin;

      // Boustrophedon 航点
      int nStrips = std::max(1, (int)std::ceil(subWidth / swathWidth_));
      //生成割草机轨迹
      for (int j = 0; j < nStrips; j++)
      {
        double eastCenter = subEastMin + (j + 0.5) * (subWidth / nStrips);
        if (j % 2 == 0)
        {
          g.waypoints.push_back({northMin, eastCenter});
          g.waypoints.push_back({northMax, eastCenter});
        }
        else
        {
          g.waypoints.push_back({northMax, eastCenter});
          g.waypoints.push_back({northMin, eastCenter});
        }
      }

      ROS_INFO("[SwarmPlanner] %s: %zu 个航点, east [%.0f, %.0f]",
               g.ns.c_str(), g.waypoints.size(), subEastMin, subEastMax);

      ros::NodeHandle nhGlobal;
      g.targetPub = nhGlobal.advertise<geometry_msgs::PoseStamped>(
          "/" + g.ns + "/mission/target", 1);
      g.missionSub = nhGlobal.subscribe<ug_msgs::MissionState>(
          "/" + g.ns + "/mission/state", 1,
          [this, k](const ug_msgs::MissionState::ConstPtr &msg)
          { onMissionState(k, msg); });
    }

    // 监听水声通信总线，收到 TARGET_FOUND 停止下发
    ros::NodeHandle nhG;
    commSub_ = nhG.subscribe("/swarm/acoustic_channel", 10,
                              &SwarmPlannerNode::onAcousticMsg, this);

    timer_ = nh_.createTimer(ros::Duration(1.0), &SwarmPlannerNode::update, this);
    startDelay_ = ros::Time::now();

    ROS_INFO("[SwarmPlanner] 启动完成, %d 台滑翔机, 区域 %.0f×%.0f m",
             N, areaLength_, areaWidth_);
  }

private:
  void onMissionState(int idx, const ug_msgs::MissionState::ConstPtr &msg)
  {
    if (idx < 0 || idx >= (int)gliders_.size())
      return;
    gliders_[idx].arrived = (msg->state == 3); // ARRIVED
  }

  void onAcousticMsg(const ug_msgs::SwarmComm::ConstPtr &msg)
  {
    if (msg->opcode == ug_msgs::SwarmComm::TARGET_FOUND && !missionAborted_)
    {
      missionAborted_ = true;
      ROS_WARN("[SwarmPlanner] 收到 TARGET_FOUND from G%d, 停止航点下发", msg->sender_id);
    }
  }

  void update(const ros::TimerEvent &)
  {
    // 延迟 5s 启动，等各节点就绪
    if ((ros::Time::now() - startDelay_).toSec() < 5.0)
      return;

    // 收到目标发现信号后停止下发
    if (missionAborted_)
      return;

    for (auto &g : gliders_)
    {
      if (g.complete)
        continue;

      if (!g.firstTargetSent)
      {
        g.waypointIdx = 0;
        sendWaypoint(g);
        g.firstTargetSent = true;
        continue;
      }

      if (g.arrived)
      {
        g.waypointIdx++;
        if (g.waypointIdx >= (int)g.waypoints.size())
        {
          g.complete = true;
          ROS_INFO("[SwarmPlanner] %s 所有航点完成！", g.ns.c_str());
          continue;
        }
        sendWaypoint(g);
      }
    }

    bool allDone = true;
    for (const auto &g : gliders_)
      if (!g.complete)
        allDone = false;
    if (allDone)
    {
      ROS_INFO("[SwarmPlanner] 所有滑翔机完成覆盖！");
      timer_.stop();
    }
  }

  void sendWaypoint(GliderEntry &g)
  {
    if (g.waypointIdx < 0 || g.waypointIdx >= (int)g.waypoints.size())
      return;

    const auto &wp = g.waypoints[g.waypointIdx];
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose.position.x = wp.north;
    msg.pose.position.y = wp.east;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.w = 1.0;
    g.targetPub.publish(msg);
    g.arrived = false;

    ROS_INFO("[SwarmPlanner] %s → 航点 %d/%zu (%.0f, %.0f)",
             g.ns.c_str(), g.waypointIdx + 1, g.waypoints.size(),
             wp.north, wp.east);
  }

  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Time startDelay_;
  ros::Subscriber commSub_;
  bool missionAborted_ = false;

  double areaCenterNorth_, areaCenterEast_;
  double areaLength_, areaWidth_;
  double swathWidth_;

  std::vector<GliderEntry> gliders_;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "swarm_planner_node");
  SwarmPlannerNode node;
  ros::spin();
  return 0;
}
