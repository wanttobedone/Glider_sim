/**
 * 编队可视化节点 - 集中管理多机可视化
 *
 * 功能：
 *   1. 三维锯齿轨迹留存（LINE_STRIP），G0 荧光绿，G1 荧光蓝
 *   2. 动态扫海涂色（OccupancyGrid 栅格地图）
 *   3. 航点意图连线（ARROW + CYLINDER 目标标记）
 *
 * 参数：
 *   ~glider_namespaces  (string[])  滑翔机命名空间列表
 *   ~area_center_north  (double)    覆盖区域参数（与 swarm_planner 一致）
 *   ~area_center_east   (double)
 *   ~area_length        (double)
 *   ~area_width         (double)
 *   ~swath_width        (double)
 *
 * 订阅：
 *   /{ns}/ground_truth/pose   (Odometry) × N     位姿（画轨迹+涂色）
 *   /{ns}/mission/state       (MissionState) × N  任务状态（航点信息）
 *
 * 发布：
 *   /swarm/trajectories       (MarkerArray)       3D 锯齿轨迹
 *   /swarm/waypoints          (MarkerArray)       航点球 + 意图线 + 目标圆柱
 *   /swarm/coverage_grid      (OccupancyGrid)     覆盖涂色栅格
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ug_msgs/MissionState.h>
#include <cmath>
#include <vector>
#include <string>
#include <clocale>

struct GliderColors
{
  float r, g, b;
};

static const GliderColors COLORS[] = {
    {0.2f, 1.0f, 0.2f},   // 荧光绿
    {0.2f, 0.6f, 1.0f},   // 荧光蓝
    {1.0f, 0.6f, 0.2f},   // 荧光橙
    {0.8f, 0.2f, 1.0f}};  // 荧光紫

struct Waypoint
{
  double north;
  double east;
};

struct GliderVizState
{
  std::string ns;
  double currentNorth, currentEast, currentZ;
  bool hasPose;
  int poseCounter;

  // 任务状态
  double targetNorth, targetEast;
  bool hasTarget;
  int missionStateVal; // 0=IDLE,1=DIVING,2=CLIMBING,3=ARRIVED

  // 轨迹
  visualization_msgs::Marker trajectory;

  ros::Subscriber poseSub;
  ros::Subscriber missionSub;
};

class SwarmVizNode
{
public:
  SwarmVizNode()
  {
    ros::NodeHandle nh("~");

    // 区域参数（与 swarm_planner 共用同一参数或 launch 配置）
    nh.param("area_center_north", areaCenterNorth_, 200.0);
    nh.param("area_center_east", areaCenterEast_, 0.0);
    nh.param("area_length", areaLength_, 400.0);
    nh.param("area_width", areaWidth_, 200.0);
    nh.param("swath_width", swathWidth_, 50.0);

    std::vector<std::string> namespaces;
    nh.getParam("glider_namespaces", namespaces);
    if (namespaces.empty())
    {
      ROS_ERROR("[SwarmViz] glider_namespaces 参数为空！");
      return;
    }

    int N = namespaces.size();
    gliders_.resize(N);

    // Boustrophedon 航点生成（与 swarm_planner 算法一致，用于可视化）
    double northMin = areaCenterNorth_ - areaLength_ / 2.0;
    double northMax = areaCenterNorth_ + areaLength_ / 2.0;
    double eastMin = areaCenterEast_ - areaWidth_ / 2.0;

    for (int k = 0; k < N; k++)
    {
      auto &g = gliders_[k];
      g.ns = namespaces[k];
      g.hasPose = false;
      g.hasTarget = false;
      g.currentNorth = g.currentEast = g.currentZ = 0;
      g.targetNorth = g.targetEast = 0;
      g.poseCounter = 0;
      g.missionStateVal = 0;

      // 子区域航点
      double subEastMin = eastMin + k * (areaWidth_ / N);
      double subEastMax = eastMin + (k + 1) * (areaWidth_ / N);
      double subWidth = subEastMax - subEastMin;
      int nStrips = std::max(1, (int)std::ceil(subWidth / swathWidth_));
      for (int j = 0; j < nStrips; j++)
      {
        double eastCenter = subEastMin + (j + 0.5) * (subWidth / nStrips);
        if (j % 2 == 0)
        {
          waypoints_[k].push_back({northMin, eastCenter});
          waypoints_[k].push_back({northMax, eastCenter});
        }
        else
        {
          waypoints_[k].push_back({northMax, eastCenter});
          waypoints_[k].push_back({northMin, eastCenter});
        }
      }

      // ROS 话题
      ros::NodeHandle nhGlobal;
      g.poseSub = nhGlobal.subscribe<nav_msgs::Odometry>(
          "/" + g.ns + "/ground_truth/pose", 1,
          [this, k](const nav_msgs::Odometry::ConstPtr &msg)
          { onPose(k, msg); });
      g.missionSub = nhGlobal.subscribe<ug_msgs::MissionState>(
          "/" + g.ns + "/mission/state", 1,
          [this, k](const ug_msgs::MissionState::ConstPtr &msg)
          { onMissionState(k, msg); });

      // 轨迹 Marker 初始化
      int ci = k % 4;
      g.trajectory.header.frame_id = "world";
      g.trajectory.ns = "trajectory_" + g.ns;
      g.trajectory.id = 1000 + k;
      g.trajectory.type = visualization_msgs::Marker::LINE_STRIP;
      g.trajectory.action = visualization_msgs::Marker::ADD;
      g.trajectory.scale.x = 0.3;
      g.trajectory.color.r = COLORS[ci].r;
      g.trajectory.color.g = COLORS[ci].g;
      g.trajectory.color.b = COLORS[ci].b;
      g.trajectory.color.a = 0.9;
      g.trajectory.pose.orientation.w = 1.0;
    }

    // 发布者
    trajPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/trajectories", 1);
    waypointPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/waypoints", 1);
    gridPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/swarm/coverage_grid", 1, true);

    initCoverageGrid();

    // 2Hz 可视化
    vizTimer_ = nh_.createTimer(ros::Duration(0.5), &SwarmVizNode::publishViz, this);

    ROS_INFO("[SwarmViz] 启动完成, %d 台滑翔机", N);
  }

private:
  void onPose(int idx, const nav_msgs::Odometry::ConstPtr &msg)
  {
    auto &g = gliders_[idx];
    g.currentNorth = msg->pose.pose.position.x;
    g.currentEast = msg->pose.pose.position.y;
    g.currentZ = msg->pose.pose.position.z;
    g.hasPose = true;

    // 轨迹降采样（每10条取1条）
    g.poseCounter++;
    if (g.poseCounter % 10 == 0)
    {
      geometry_msgs::Point p;
      p.x = g.currentNorth;
      p.y = g.currentEast;
      p.z = g.currentZ;
      g.trajectory.points.push_back(p);
      if (g.trajectory.points.size() > 10000)
        g.trajectory.points.erase(g.trajectory.points.begin());
    }

    // 更新覆盖栅格
    updateCoverageGrid(g.currentNorth, g.currentEast);
  }

  void onMissionState(int idx, const ug_msgs::MissionState::ConstPtr &msg)
  {
    auto &g = gliders_[idx];
    g.missionStateVal = msg->state;
    g.targetNorth = msg->target_north;
    g.targetEast = msg->target_east;
    g.hasTarget = (msg->state > 0); // 非 IDLE 时有目标
  }

  // ── 覆盖栅格 ──

  void initCoverageGrid()
  {
    double resolution = 5.0;
    gridOriginNorth_ = areaCenterNorth_ - areaLength_ / 2.0;
    gridOriginEast_ = areaCenterEast_ - areaWidth_ / 2.0;
    gridResolution_ = resolution;
    gridWidth_ = (int)std::ceil(areaWidth_ / resolution);
    gridHeight_ = (int)std::ceil(areaLength_ / resolution);

    coverageGrid_.header.frame_id = "world";
    coverageGrid_.info.resolution = resolution;
    coverageGrid_.info.width = gridWidth_;
    coverageGrid_.info.height = gridHeight_;
    coverageGrid_.info.origin.position.x = gridOriginNorth_;
    coverageGrid_.info.origin.position.y = gridOriginEast_;
    coverageGrid_.info.origin.position.z = -1.0;
    coverageGrid_.info.origin.orientation.w = 1.0;
    coverageGrid_.data.assign(gridWidth_ * gridHeight_, -1);
  }

  void updateCoverageGrid(double north, double east)
  {
    double halfSwath = swathWidth_ / 2.0;
    int iy_min = std::max(0, (int)std::floor((east - halfSwath - gridOriginEast_) / gridResolution_));
    int iy_max = std::min(gridWidth_ - 1, (int)std::ceil((east + halfSwath - gridOriginEast_) / gridResolution_));
    int ix = (int)std::floor((north - gridOriginNorth_) / gridResolution_);

    if (ix < 0 || ix >= gridHeight_)
      return;

    for (int iy = iy_min; iy <= iy_max; iy++)
    {
      int cell = ix * gridWidth_ + iy;
      if (cell >= 0 && cell < (int)coverageGrid_.data.size())
        coverageGrid_.data[cell] = 0;
    }
  }

  // ── 可视化发布 ──

  void publishViz(const ros::TimerEvent &)
  {
    ros::Time now = ros::Time::now();

    // 1. 轨迹
    visualization_msgs::MarkerArray trajArray;
    for (auto &g : gliders_)
    {
      g.trajectory.header.stamp = now;
      trajArray.markers.push_back(g.trajectory);
    }
    trajPub_.publish(trajArray);

    // 2. 航点 + 意图线
    visualization_msgs::MarkerArray wpArray;
    int markerId = 0;

    for (int k = 0; k < (int)gliders_.size(); k++)
    {
      const auto &g = gliders_[k];
      int ci = k % 4;
      const auto &wps = waypoints_[k];

      // 当前航点索引推断：找最近的已走过航点
      int currentWpIdx = -1;
      if (g.hasTarget)
      {
        for (int j = 0; j < (int)wps.size(); j++)
        {
          if (std::abs(wps[j].north - g.targetNorth) < 1.0 &&
              std::abs(wps[j].east - g.targetEast) < 1.0)
          {
            currentWpIdx = j;
            break;
          }
        }
      }

      // 2a. 航点球体
      for (int j = 0; j < (int)wps.size(); j++)
      {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp = now;
        sphere.ns = "wp_" + g.ns;
        sphere.id = markerId++;
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.position.x = wps[j].north;
        sphere.pose.position.y = wps[j].east;
        sphere.pose.position.z = 0.0;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 3.0;
        sphere.scale.y = 3.0;
        sphere.scale.z = 3.0;

        bool passed = (currentWpIdx >= 0 && j < currentWpIdx);
        sphere.color.r = passed ? COLORS[ci].r * 0.3f : COLORS[ci].r;
        sphere.color.g = passed ? COLORS[ci].g * 0.3f : COLORS[ci].g;
        sphere.color.b = passed ? COLORS[ci].b * 0.3f : COLORS[ci].b;
        sphere.color.a = passed ? 0.3f : 0.8f;
        sphere.lifetime = ros::Duration(1.0);
        wpArray.markers.push_back(sphere);
      }

      // 2b. 航点序列连线
      if (wps.size() > 1)
      {
        visualization_msgs::Marker line;
        line.header.frame_id = "world";
        line.header.stamp = now;
        line.ns = "wp_line_" + g.ns;
        line.id = markerId++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = 0.5;
        line.color.r = COLORS[ci].r;
        line.color.g = COLORS[ci].g;
        line.color.b = COLORS[ci].b;
        line.color.a = 0.3;
        line.pose.orientation.w = 1.0;
        line.lifetime = ros::Duration(1.0);
        for (const auto &wp : wps)
        {
          geometry_msgs::Point p;
          p.x = wp.north;
          p.y = wp.east;
          p.z = 0.0;
          line.points.push_back(p);
        }
        wpArray.markers.push_back(line);
      }

      // 2c. 意图线（当前位置 → 当前目标航点）
      if (g.hasPose && g.hasTarget && g.missionStateVal > 0 && g.missionStateVal < 3)
      {
        // 箭头
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = now;
        arrow.ns = "intent_" + g.ns;
        arrow.id = markerId++;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.scale.x = 0.5;
        arrow.scale.y = 1.0;
        arrow.scale.z = 1.0;
        arrow.color.r = COLORS[ci].r;
        arrow.color.g = COLORS[ci].g;
        arrow.color.b = COLORS[ci].b;
        arrow.color.a = 0.6;
        arrow.lifetime = ros::Duration(1.0);
        arrow.pose.orientation.w = 1.0;
        geometry_msgs::Point p0, p1;
        p0.x = g.currentNorth;
        p0.y = g.currentEast;
        p0.z = g.currentZ;
        p1.x = g.targetNorth;
        p1.y = g.targetEast;
        p1.z = 0.0;
        arrow.points.push_back(p0);
        arrow.points.push_back(p1);
        wpArray.markers.push_back(arrow);

        // 目标圆柱
        visualization_msgs::Marker cyl;
        cyl.header.frame_id = "world";
        cyl.header.stamp = now;
        cyl.ns = "target_cyl_" + g.ns;
        cyl.id = markerId++;
        cyl.type = visualization_msgs::Marker::CYLINDER;
        cyl.action = visualization_msgs::Marker::ADD;
        cyl.pose.position.x = g.targetNorth;
        cyl.pose.position.y = g.targetEast;
        cyl.pose.position.z = -1.0;
        cyl.pose.orientation.w = 1.0;
        cyl.scale.x = 5.0;
        cyl.scale.y = 5.0;
        cyl.scale.z = 2.0;
        cyl.color.r = COLORS[ci].r;
        cyl.color.g = COLORS[ci].g;
        cyl.color.b = COLORS[ci].b;
        cyl.color.a = 0.3;
        cyl.lifetime = ros::Duration(1.0);
        wpArray.markers.push_back(cyl);
      }
    }
    waypointPub_.publish(wpArray);

    // 3. 覆盖栅格（1Hz）
    gridCounter_++;
    if (gridCounter_ % 2 == 0)
    {
      coverageGrid_.header.stamp = now;
      gridPub_.publish(coverageGrid_);
    }
  }

  // ── 成员变量 ──

  ros::NodeHandle nh_;
  ros::Timer vizTimer_;

  double areaCenterNorth_, areaCenterEast_;
  double areaLength_, areaWidth_;
  double swathWidth_;

  std::vector<GliderVizState> gliders_;
  std::map<int, std::vector<Waypoint>> waypoints_; // k -> 航点列表

  // 可视化
  ros::Publisher trajPub_;
  ros::Publisher waypointPub_;
  ros::Publisher gridPub_;

  // 覆盖栅格
  nav_msgs::OccupancyGrid coverageGrid_;
  double gridOriginNorth_, gridOriginEast_;
  double gridResolution_;
  int gridWidth_, gridHeight_;
  int gridCounter_ = 0;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "swarm_viz_node");
  SwarmVizNode node;
  ros::spin();
  return 0;
}
