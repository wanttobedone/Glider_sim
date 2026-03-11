/**
 * 编队可视化节点 - 集中管理多机可视化
 *
 * 功能：
 *   1. 三维锯齿轨迹留存（LINE_STRIP），G0 荧光绿，G1 荧光蓝，SURFACING 时变亮黄
 *   2. 三维体素覆盖涂色（CUBE_LIST），r=25m 探测球扫过的空间
 *   3. 探测范围球（SPHERE），半透明绿色，跟随滑翔机
 *   4. 航点意图连线（球体 + 连线）
 *   5. 覆盖率统计（std_msgs/Float64）
 *   6. 异常目标标记（红色圆柱 + 半透明探测球）
 *   7. 声波扩散环（每条通信消息触发，TARGET_FOUND 红色 / 其余黄色，3s 消散）
 *   8. 通信虚线（Glider 之间绿色虚线，15s 无通信自动消失）
 *   9. 卫星链路（SURFACING + depth<0.5m 时，向上黄色虚线 + 闪烁 "SAT LINK ACTIVE"）
 *
 * 参数（/swarm/ 命名空间）：
 *   glider_namespaces      (string[])  滑翔机命名空间列表
 *   area_center_north      (double)    覆盖区域中心 north [m]
 *   area_center_east       (double)    覆盖区域中心 east [m]
 *   area_length            (double)    区域 north 长度 [m]
 *   area_width             (double)    区域 east 宽度 [m]
 *   area_depth             (double)    覆盖深度范围 [m]（默认 100）
 *   swath_width            (double)    覆盖带宽 [m]
 *   detect_radius          (double)    探测半径 [m]（默认 25）
 *   voxel_size             (double)    体素边长 [m]（默认 5）
 *   anomaly_north/east/depth/detect_radius  异常目标参数
 *
 * 订阅：
 *   /{ns}/ground_truth/pose    (Odometry) × N      位姿（画轨迹 + 涂色）
 *   /{ns}/mission/state        (MissionState) × N   任务状态（航点 + 卫星链路判定）
 *   /swarm/acoustic_channel    (SwarmComm)          水声通信（声波环 + 虚线 + 轨迹变色）
 *
 * 发布：
 *   /swarm/trajectories        (MarkerArray)        3D 锯齿轨迹
 *   /swarm/waypoints           (MarkerArray)        航点球 + 连线
 *   /swarm/coverage_volume     (Marker)             3D 体素覆盖（CUBE_LIST）
 *   /swarm/detection_spheres   (MarkerArray)        探测范围球
 *   /swarm/coverage_percent    (Float64)            覆盖率百分比
 *   /swarm/anomaly_markers     (MarkerArray)        异常目标 + 声波环 + 通信虚线 + 卫星链路
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <ug_msgs/MissionState.h>
#include <ug_msgs/SwarmComm.h>
#include <cmath>
#include <vector>
#include <string>
#include <set>
#include <tuple>
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
  int missionStateVal; // 0=IDLE,1=DIVING,2=CLIMBING,3=ARRIVED,4=LOITERING,5=SURFACING

  // 轨迹
  visualization_msgs::Marker trajectory;

  ros::Subscriber poseSub;
  ros::Subscriber missionSub;
};

// 体素索引键
typedef std::tuple<int, int, int> VoxelKey;

class SwarmVizNode
{
public:
  SwarmVizNode()
  {
    ros::NodeHandle nh("/swarm");  // 从全局 /swarm/ 命名空间读共享参数

    // 区域参数（与 swarm_planner 共用 /swarm/ 下的参数）
    nh.param("area_center_north", areaCenterNorth_, 200.0);
    nh.param("area_center_east", areaCenterEast_, 0.0);
    nh.param("area_length", areaLength_, 400.0);
    nh.param("area_width", areaWidth_, 200.0);
    nh.param("area_depth", areaDepth_, 100.0);
    nh.param("swath_width", swathWidth_, 50.0);
    nh.param("detect_radius", detectRadius_, 25.0);
    nh.param("voxel_size", voxelSize_, 5.0);

    std::vector<std::string> namespaces;
    nh.getParam("glider_namespaces", namespaces);
    if (namespaces.empty())
    {
      ROS_ERROR("[SwarmViz] glider_namespaces 参数为空！");
      return;
    }

    // 计算覆盖区域总体素数（用于覆盖率）
    int vNorth = (int)std::ceil(areaLength_ / voxelSize_);
    int vEast = (int)std::ceil(areaWidth_ / voxelSize_);
    int vDepth = (int)std::ceil(areaDepth_ / voxelSize_);
    totalVoxels_ = vNorth * vEast * vDepth;
    // ROS_INFO("[SwarmViz] 覆盖区域体素总数: %d (%dx%dx%d)", totalVoxels_, vNorth, vEast, vDepth);

    // 预计算探测球内的体素偏移表（相对于中心体素的整数偏移）
    int maxOffset = (int)std::ceil(detectRadius_ / voxelSize_);
    for (int di = -maxOffset; di <= maxOffset; di++)
    {
      for (int dj = -maxOffset; dj <= maxOffset; dj++)
      {
        for (int dk = -maxOffset; dk <= maxOffset; dk++)
        {
          double dist = std::sqrt(di * di + dj * dj + dk * dk) * voxelSize_;
          if (dist <= detectRadius_)
          {
            sphereOffsets_.push_back({di, dj, dk});
          }
        }
      }
    }
    // ROS_INFO("[SwarmViz] 探测球体素偏移数: %d (r=%.0fm, voxel=%.0fm)",
    //          (int)sphereOffsets_.size(), detectRadius_, voxelSize_);

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

    // 异常目标参数
    nh.param("anomaly_north", anomalyNorth_, 150.0);
    nh.param("anomaly_east", anomalyEast_, 12.5);
    nh.param("anomaly_depth", anomalyDepth_, 25.0);
    nh.param("anomaly_detect_radius", anomalyDetectRadius_, 15.0);

    // 水声通信订阅（声波扩散环用）
    ros::NodeHandle nhGlob;
    commSub_ = nhGlob.subscribe("/swarm/acoustic_channel", 10,
                                 &SwarmVizNode::onAcousticMsg, this);

    // 发布者
    trajPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/trajectories", 1);
    waypointPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/waypoints", 1);
    coveragePub_ = nh_.advertise<visualization_msgs::Marker>("/swarm/coverage_volume", 1, true);
    spherePub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/detection_spheres", 1);
    percentPub_ = nh_.advertise<std_msgs::Float64>("/swarm/coverage_percent", 1, true);
    anomalyPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/swarm/anomaly_markers", 1, true);

    // 2Hz 可视化
    vizTimer_ = nh_.createTimer(ros::Duration(0.5), &SwarmVizNode::publishViz, this);

    ROS_INFO("[SwarmViz] 启动完成, %d 台滑翔机, 3D体素覆盖模式", N);
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

    // 更新3D体素覆盖（降频，每5次位姿更新1次）
    if (g.poseCounter % 5 == 0)
    {
      updateCoverageVolume(g.currentNorth, g.currentEast, g.currentZ);
    }
  }

  void onMissionState(int idx, const ug_msgs::MissionState::ConstPtr &msg)
  {
    auto &g = gliders_[idx];
    g.missionStateVal = msg->state;
    g.targetNorth = msg->target_north;
    g.targetEast = msg->target_east;
    g.hasTarget = (msg->state > 0);
  }

  //   3D 体素覆盖  

  void updateCoverageVolume(double north, double east, double z)
  {
    // 将位置转为体素索引
    int ci = (int)std::floor(north / voxelSize_);
    int cj = (int)std::floor(east / voxelSize_);
    int ck = (int)std::floor(z / voxelSize_);

    // 遍历预计算的球形偏移，将覆盖的体素加入 set
    for (const auto &off : sphereOffsets_)
    {
      int vk = ck + std::get<2>(off);
      // 海面以上的体素不染色（z>0 对应 vk>=0）
      if ((vk + 0.5) * voxelSize_ > 0.0)
        continue;
      VoxelKey key(ci + std::get<0>(off),
                   cj + std::get<1>(off),
                   vk);
      visitedVoxels_.insert(key);
    }
    coverageDirty_ = true;
  }

  //   可视化发布  

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

      // 当前航点索引推断
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
        sphere.scale.x = 0.5;
        sphere.scale.y = 0.5;
        sphere.scale.z = 0.5;

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
        line.scale.x = 0.1;
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

    }
    waypointPub_.publish(wpArray);

    // 3. 探测范围球（跟随滑翔机的半透明绿球）
    visualization_msgs::MarkerArray sphereArray;
    for (int k = 0; k < (int)gliders_.size(); k++)
    {
      const auto &g = gliders_[k];
      if (!g.hasPose)
        continue;

      visualization_msgs::Marker det;
      det.header.frame_id = "world";
      det.header.stamp = now;
      det.ns = "detection_sphere";
      det.id = k;
      det.type = visualization_msgs::Marker::SPHERE;
      det.action = visualization_msgs::Marker::ADD;
      det.pose.position.x = g.currentNorth;
      det.pose.position.y = g.currentEast;
      det.pose.position.z = g.currentZ;
      det.pose.orientation.w = 1.0;
      double diameter = detectRadius_ * 2.0;
      det.scale.x = diameter;
      det.scale.y = diameter;
      det.scale.z = diameter;
      det.color.r = 0.0;
      det.color.g = 1.0;
      det.color.b = 0.0;
      det.color.a = 0.12;
      det.lifetime = ros::Duration(1.0);
      sphereArray.markers.push_back(det);
    }
    spherePub_.publish(sphereArray);

    // 4. 3D 体素覆盖（1Hz，仅在有变化时重建）
    vizCounter_++;
    if (vizCounter_ % 2 == 0 && coverageDirty_)
    {
      publishCoverageVolume(now);
      coverageDirty_ = false;

      // 覆盖率
      if (totalVoxels_ > 0)
      {
        std_msgs::Float64 pct;
        pct.data = (double)visitedVoxels_.size() / totalVoxels_ * 100.0;
        percentPub_.publish(pct);
      }
    }

    // 5. 异常目标标记 + 声波扩散环
    publishAnomalyMarkers(now);
  }

  void publishCoverageVolume(const ros::Time &now)
  {
    visualization_msgs::Marker cubes;
    cubes.header.frame_id = "world";
    cubes.header.stamp = now;
    cubes.ns = "coverage_volume";
    cubes.id = 0;
    cubes.type = visualization_msgs::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::Marker::ADD;
    cubes.scale.x = voxelSize_;
    cubes.scale.y = voxelSize_;
    cubes.scale.z = voxelSize_;
    cubes.pose.orientation.w = 1.0;

    // 深度渐变色：浅层亮黄 → 深层暗橙
    // Z 在 Gazebo 里是 Up，水下为负值
    double zMin = -areaDepth_;  // 最深处
    double zMax = 0.0;          // 水面
    double zRange = zMax - zMin;

    cubes.points.reserve(visitedVoxels_.size());
    cubes.colors.reserve(visitedVoxels_.size());

    for (const auto &vk : visitedVoxels_)
    {
      geometry_msgs::Point p;
      p.x = (std::get<0>(vk) + 0.5) * voxelSize_;  // 体素中心
      p.y = (std::get<1>(vk) + 0.5) * voxelSize_;
      p.z = (std::get<2>(vk) + 0.5) * voxelSize_;
      cubes.points.push_back(p);

      // 深度归一化 t: 0=水面(亮黄) → 1=最深(暗橙)
      double t = 0.5;
      if (zRange > 0.01)
        t = std::max(0.0, std::min(1.0, (zMax - p.z) / zRange));

      std_msgs::ColorRGBA c;
      c.r = 0.0;
      c.g = 0.9 - 0.5 * t;   // 0.9(浅) → 0.4(深)
      c.b = 0.0 + 0.1 * t;   // 微微偏橙
      c.a = 0.05;
      cubes.colors.push_back(c);
    }

    coveragePub_.publish(cubes);

    // ROS_INFO_THROTTLE(10.0, "[SwarmViz] 已覆盖体素: %d / %d (%.1f%%)",
    //                   (int)visitedVoxels_.size(), totalVoxels_,
    //                   (double)visitedVoxels_.size() / totalVoxels_ * 100.0);
  }

  void onAcousticMsg(const ug_msgs::SwarmComm::ConstPtr &msg)
  {
    // 每条消息都触发声波扩散环
    int sid = msg->sender_id;
    if (sid >= 0 && sid < (int)gliders_.size() && gliders_[sid].hasPose)
    {
      waveActive_ = true;
      waveEventTime_ = ros::Time::now();
      waveOriginX_ = gliders_[sid].currentNorth;
      waveOriginY_ = gliders_[sid].currentEast;
      waveOriginZ_ = gliders_[sid].currentZ;
      // TARGET_FOUND 红色，其余黄色
      waveIsAlert_ = (msg->opcode == ug_msgs::SwarmComm::TARGET_FOUND);
    }

    // 记录通信活跃状态（用于 Glider 间虚线）
    commActive_ = true;
    lastCommTime_ = ros::Time::now();

    // G1 进入 SURFACING 时改轨迹颜色
    if (msg->opcode == ug_msgs::SwarmComm::REQ_SURFACE)
    {
      int target = msg->receiver_id;
      if (target >= 0 && target < (int)gliders_.size())
      {
        gliders_[target].trajectory.color.r = 1.0;
        gliders_[target].trajectory.color.g = 0.9;
        gliders_[target].trajectory.color.b = 0.0;
        gliders_[target].trajectory.color.a = 0.9;
      }
    }
  }

  void publishAnomalyMarkers(const ros::Time &now)
  {
    visualization_msgs::MarkerArray arr;

    // 红色圆柱体（海底目标）
    visualization_msgs::Marker cyl;
    cyl.header.frame_id = "world";
    cyl.header.stamp = now;
    cyl.ns = "anomaly_target";
    cyl.id = 0;
    cyl.type = visualization_msgs::Marker::CYLINDER;
    cyl.action = visualization_msgs::Marker::ADD;
    cyl.pose.position.x = anomalyNorth_;
    cyl.pose.position.y = anomalyEast_;
    cyl.pose.position.z = -anomalyDepth_;  // NED→ENU
    cyl.pose.orientation.w = 1.0;
    cyl.scale.x = 4.0;
    cyl.scale.y = 4.0;
    cyl.scale.z = 8.0;
    cyl.color.r = 1.0;
    cyl.color.g = 0.1;
    cyl.color.b = 0.1;
    cyl.color.a = 0.8;
    arr.markers.push_back(cyl);

    // 探测半径球（半透明红）
    visualization_msgs::Marker dSphere;
    dSphere.header = cyl.header;
    dSphere.ns = "anomaly_detect_range";
    dSphere.id = 1;
    dSphere.type = visualization_msgs::Marker::SPHERE;
    dSphere.action = visualization_msgs::Marker::ADD;
    dSphere.pose.position = cyl.pose.position;
    dSphere.pose.orientation.w = 1.0;
    double d = anomalyDetectRadius_ * 2.0;
    dSphere.scale.x = d;
    dSphere.scale.y = d;
    dSphere.scale.z = d;
    dSphere.color.r = 1.0;
    dSphere.color.g = 0.2;
    dSphere.color.b = 0.2;
    dSphere.color.a = 0.1;
    arr.markers.push_back(dSphere);

    // 声波扩散环（每次通信触发，3 秒内扩散消散）
    if (waveActive_)
    {
      double dt = (now - waveEventTime_).toSec();
      if (dt < 3.0)
      {
        visualization_msgs::Marker wave;
        wave.header.frame_id = "world";
        wave.header.stamp = now;
        wave.ns = "acoustic_wave";
        wave.id = 2;
        wave.type = visualization_msgs::Marker::CYLINDER;
        wave.action = visualization_msgs::Marker::ADD;
        wave.pose.position.x = waveOriginX_;
        wave.pose.position.y = waveOriginY_;
        wave.pose.position.z = waveOriginZ_;
        wave.pose.orientation.w = 1.0;
        double radius = dt * 50.0;
        wave.scale.x = radius * 2.0;
        wave.scale.y = radius * 2.0;
        wave.scale.z = 0.5;
        if (waveIsAlert_)
        {
          wave.color.r = 1.0; wave.color.g = 0.2; wave.color.b = 0.1; // 红色
        }
        else
        {
          wave.color.r = 1.0; wave.color.g = 0.9; wave.color.b = 0.0; // 黄色
        }
        wave.color.a = (float)(1.0 - dt / 3.0) * 0.4;
        wave.lifetime = ros::Duration(0.6);
        arr.markers.push_back(wave);
      }
      else
      {
        waveActive_ = false;
      }
    }

    // 通信虚线（Glider 之间，LINE_LIST 模拟虚线）
    if (commActive_ && gliders_.size() >= 2)
    {
      // 超过 15 秒无通信则隐藏
      if ((now - lastCommTime_).toSec() > 15.0)
        commActive_ = false;

      if (commActive_)
      {
        for (int i = 0; i < (int)gliders_.size(); i++)
        {
          for (int j = i + 1; j < (int)gliders_.size(); j++)
          {
            if (!gliders_[i].hasPose || !gliders_[j].hasPose)
              continue;

            visualization_msgs::Marker dash;
            dash.header.frame_id = "world";
            dash.header.stamp = now;
            dash.ns = "comm_link";
            dash.id = 100 + i * 10 + j;
            dash.type = visualization_msgs::Marker::LINE_LIST;
            dash.action = visualization_msgs::Marker::ADD;
            dash.scale.x = 0.3;
            dash.color.r = 0.2; dash.color.g = 1.0; dash.color.b = 0.4; dash.color.a = 0.6;
            dash.pose.orientation.w = 1.0;
            dash.lifetime = ros::Duration(1.0);

            // 两点之间生成间隔短线段（2m 画, 2m 跳）
            double x0 = gliders_[i].currentNorth, y0 = gliders_[i].currentEast, z0 = gliders_[i].currentZ;
            double x1 = gliders_[j].currentNorth, y1 = gliders_[j].currentEast, z1 = gliders_[j].currentZ;
            double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
            double len = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (len < 0.1) continue;
            double ux = dx / len, uy = dy / len, uz = dz / len;
            double seg = 2.0, gap = 2.0, step = seg + gap;

            for (double t = 0; t + seg <= len; t += step)
            {
              geometry_msgs::Point pa, pb;
              pa.x = x0 + ux * t;        pa.y = y0 + uy * t;        pa.z = z0 + uz * t;
              pb.x = x0 + ux * (t + seg); pb.y = y0 + uy * (t + seg); pb.z = z0 + uz * (t + seg);
              dash.points.push_back(pa);
              dash.points.push_back(pb);
            }
            arr.markers.push_back(dash);
          }
        }
      }
    }

    // 卫星链路（SURFACING 且 depth < 0.5m 时）
    for (int k = 0; k < (int)gliders_.size(); k++)
    {
      const auto &g = gliders_[k];
      // currentZ 是 ENU，水下为负，depth < 0.5m 对应 currentZ > -0.5
      if (!g.hasPose || g.missionStateVal != 5 || g.currentZ < -0.5)
        continue;

      // 向上虚线
      visualization_msgs::Marker satLine;
      satLine.header.frame_id = "world";
      satLine.header.stamp = now;
      satLine.ns = "sat_link";
      satLine.id = 200 + k;
      satLine.type = visualization_msgs::Marker::LINE_LIST;
      satLine.action = visualization_msgs::Marker::ADD;
      satLine.scale.x = 0.1;
      satLine.color.r = 1.0; satLine.color.g = 0.9; satLine.color.b = 0.0; satLine.color.a = 0.8;
      satLine.pose.orientation.w = 1.0;
      satLine.lifetime = ros::Duration(1.0);

      for (double h = 0; h + 3.0 <= 50.0; h += 5.0)
      {
        geometry_msgs::Point pa, pb;
        pa.x = g.currentNorth; pa.y = g.currentEast; pa.z = g.currentZ + h;
        pb.x = g.currentNorth; pb.y = g.currentEast; pb.z = g.currentZ + h + 3.0;
        satLine.points.push_back(pa);
        satLine.points.push_back(pb);
      }
      arr.markers.push_back(satLine);

      // 闪烁文字（vizCounter_ 奇数帧显示）
      if (vizCounter_ % 2 == 1)
      {
        visualization_msgs::Marker txt;
        txt.header.frame_id = "world";
        txt.header.stamp = now;
        txt.ns = "sat_text";
        txt.id = 300 + k;
        txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        txt.action = visualization_msgs::Marker::ADD;
        txt.pose.position.x = g.currentNorth;
        txt.pose.position.y = g.currentEast;
        txt.pose.position.z = g.currentZ + 55.0;
        txt.pose.orientation.w = 1.0;
        txt.scale.z = 5.0;
        txt.color.r = 1.0; txt.color.g = 0.9; txt.color.b = 0.0; txt.color.a = 1.0;
        txt.text = "SAT LINK ACTIVE";
        txt.lifetime = ros::Duration(0.6);
        arr.markers.push_back(txt);
      }
    }

    anomalyPub_.publish(arr);
  }

  //   成员变量

  ros::NodeHandle nh_;
  ros::Timer vizTimer_;

  double areaCenterNorth_, areaCenterEast_;
  double areaLength_, areaWidth_, areaDepth_;
  double swathWidth_;
  double detectRadius_;
  double voxelSize_;

  std::vector<GliderVizState> gliders_;
  std::map<int, std::vector<Waypoint>> waypoints_;

  // 3D 体素覆盖
  std::set<VoxelKey> visitedVoxels_;
  std::vector<VoxelKey> sphereOffsets_;  // 探测球内的体素偏移（预计算）
  int totalVoxels_ = 0;
  bool coverageDirty_ = false;

  // 异常目标
  double anomalyNorth_, anomalyEast_, anomalyDepth_, anomalyDetectRadius_;

  // 声波扩散环 + 通信状态
  ros::Subscriber commSub_;
  bool waveActive_ = false;
  bool waveIsAlert_ = false;
  ros::Time waveEventTime_;
  double waveOriginX_ = 0, waveOriginY_ = 0, waveOriginZ_ = 0;
  bool commActive_ = false;
  ros::Time lastCommTime_;

  // 发布者
  ros::Publisher trajPub_;
  ros::Publisher waypointPub_;
  ros::Publisher coveragePub_;
  ros::Publisher spherePub_;
  ros::Publisher percentPub_;
  ros::Publisher anomalyPub_;

  int vizCounter_ = 0;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "swarm_viz_node");
  SwarmVizNode node;
  ros::spin();
  return 0;
}
