/**
 * 水下滑翔机分层洋流插件实现
 * 每个物理步，获取滑翔机深度，再线性插值洋流然后通过Gazebo transport 发布
 * 降频处理，ROS TwistStamped (20Hz) + RViz MarkerArray (2Hz)
 */

#include <ug_gazebo_plugins/UGOceanCurrentPlugin.hh>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>

namespace gazebo
{

UGOceanCurrentPlugin::UGOceanCurrentPlugin()
  : flowTopic_("hydrodynamics/current_velocity")
{
}

UGOceanCurrentPlugin::~UGOceanCurrentPlugin()
{
  if (rosNode_)
    rosNode_->shutdown();
}

void UGOceanCurrentPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;

  //    解析命名空间   
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->Get<std::string>("namespace");
  else
    namespace_ = model_->GetName();

  //    解析话题名   
  if (_sdf->HasElement("flow_velocity_topic"))
    flowTopic_ = _sdf->Get<std::string>("flow_velocity_topic");

  //    加载数据源（CSV 优先）  
  bool loaded = false;
  if (_sdf->HasElement("database_csv"))
  {
    std::string csvPath = _sdf->Get<std::string>("database_csv");
    loaded = LoadLayersFromCSV(csvPath);
    if (loaded)
      gzmsg << "[UGOceanCurrent] 从 CSV 加载了 " << layers_.size()
            << " 个深度层: " << csvPath << std::endl;
  }

  if (!loaded)
  {
    LoadLayersFromSDF(_sdf);
    gzmsg << "[UGOceanCurrent] 从 SDF 加载了 " << layers_.size()
          << " 个深度层" << std::endl;
  }

  if (layers_.empty())
  {
    gzerr << "[UGOceanCurrent] 没有洋流数据层，插件不工作" << std::endl;
    return;
  }

  // 按深度排序
  std::sort(layers_.begin(), layers_.end(),
    [](const DepthLayer &a, const DepthLayer &b) { return a.depth < b.depth; });

  // 打印加载的分层表
  for (const auto &l : layers_)
  {
    gzmsg << "  depth=" << l.depth
          << "m  vel=(" << l.velocity.X() << ", "
          << l.velocity.Y() << ", " << l.velocity.Z() << ") m/s [ENU]"
          << std::endl;
  }

  //    初始化 Gazebo transport   
  gazeboNode_ = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 8
  gazeboNode_->Init(model_->GetWorld()->Name());
#else
  gazeboNode_->Init(model_->GetWorld()->GetName());
#endif
  currentPub_ = gazeboNode_->Advertise<msgs::Vector3d>(flowTopic_);
  gzmsg << "[UGOceanCurrent] Gazebo transport 话题: " << flowTopic_ << std::endl;

  //    初始化 ROS   
  if (ros::isInitialized())
  {
    rosNode_.reset(new ros::NodeHandle(namespace_));
    rosPub_ = rosNode_->advertise<geometry_msgs::TwistStamped>(
        "ocean_current", 10);
    markerPub_ = rosNode_->advertise<visualization_msgs::MarkerArray>(
        "ocean_current_markers", 1);
    gzmsg << "[UGOceanCurrent] ROS 话题: /" << namespace_
          << "/ocean_current" << std::endl;
  }
  else
  {
    gzwarn << "[UGOceanCurrent] ROS 未初始化，跳过 ROS 发布" << std::endl;
  }

  //    绑定物理步回调   
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&UGOceanCurrentPlugin::OnUpdate, this, std::placeholders::_1));

  gzmsg << "[UGOceanCurrent] 插件加载完成" << std::endl;
}

  
// SDF <layer> 加载
  
void UGOceanCurrentPlugin::LoadLayersFromSDF(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("layer"))
    return;

  sdf::ElementPtr layerElem = _sdf->GetElement("layer");
  while (layerElem)
  {
    DepthLayer dl;
    dl.depth = layerElem->Get<double>("depth");
    double vx = layerElem->Get<double>("vx");
    double vy = layerElem->Get<double>("vy");
    double vz = layerElem->HasAttribute("vz") ? layerElem->Get<double>("vz") : 0.0;
    dl.velocity.Set(vx, vy, vz);  // 已经是 ENU
    layers_.push_back(dl);
    layerElem = layerElem->GetNextElement("layer");
  }
}

  
// CSV 加载（DAVE 兼容格式：north, east, depth）
  
bool UGOceanCurrentPlugin::LoadLayersFromCSV(const std::string &_path)
{
  std::ifstream file(_path);
  if (!file.is_open())
  {
    // 尝试 Gazebo 资源路径
    common::SystemPaths *paths = common::SystemPaths::Instance();
    std::string resolved = paths->FindFile(_path, true);
    file.open(resolved);
    if (!file.is_open())
    {
      gzerr << "[UGOceanCurrent] 无法打开 CSV: " << _path << std::endl;
      return false;
    }
  }

  // 跳过 3 行 header（DAVE 格式）
  std::string line;
  for (int i = 0; i < 3 && std::getline(file, line); ++i) {}

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    std::istringstream iss(line);
    std::string token;
    std::vector<double> row;
    while (std::getline(iss, token, ','))
    {
      try { row.push_back(std::stod(token)); }
      catch (...) { break; }
    }
    if (row.size() < 3)
      continue;

    // CSV 格式: north, east, depth(正下)
    // 转 ENU: X=east, Y=north, Z=0
    DepthLayer dl;
    dl.velocity.Set(row[1], row[0], 0.0);  // X=East(col2), Y=North(col1), Z=0
    dl.depth = row[2];                       // 正值深度
    layers_.push_back(dl);
  }

  file.close();
  return !layers_.empty();
}

  
// 深度线性插值
  
ignition::math::Vector3d UGOceanCurrentPlugin::InterpolateAtDepth(double _depth) const
{
  if (layers_.size() == 1 || _depth <= layers_.front().depth)
    return layers_.front().velocity;

  if (_depth >= layers_.back().depth)
    return layers_.back().velocity;

  // 找到插值区间
  for (size_t i = 1; i < layers_.size(); ++i)
  {
    if (_depth <= layers_[i].depth)
    {
      double rate = (_depth - layers_[i - 1].depth)
                  / (layers_[i].depth - layers_[i - 1].depth);
      return layers_[i - 1].velocity
           + (layers_[i].velocity - layers_[i - 1].velocity) * rate;
    }
  }

  return layers_.back().velocity;
}

  
// 物理步回调
  
void UGOceanCurrentPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // 当前深度（Gazebo Z 向上，取反得正值深度）
  double depth = -model_->WorldPose().Pos().Z();
  if (depth < 0.0)
    depth = 0.0;  // 水面以上按 0m 深度处理

  // 插值
  ignition::math::Vector3d currentVel = InterpolateAtDepth(depth);

  //    Gazebo transport 发布（每步，与物理同步）  
  msgs::Vector3d msg;
  msgs::Set(&msg, currentVel);
  currentPub_->Publish(msg);

  //    ROS 降频发布   
  if (rosNode_)
  {
    common::Time now = _info.simTime;

    // TwistStamped (20Hz)
    if ((now - lastRosPubTime_).Double() >= ROS_PUB_PERIOD)
    {
      lastRosPubTime_ = now;
      geometry_msgs::TwistStamped twistMsg;
      twistMsg.header.stamp = ros::Time::now();
      twistMsg.header.frame_id = "world";
      twistMsg.twist.linear.x = currentVel.X();
      twistMsg.twist.linear.y = currentVel.Y();
      twistMsg.twist.linear.z = currentVel.Z();
      rosPub_.publish(twistMsg);
    }

    // MarkerArray (2Hz)
    if ((now - lastMarkerPubTime_).Double() >= MARKER_PUB_PERIOD)
    {
      lastMarkerPubTime_ = now;
      PublishMarkers(model_->WorldPose(), currentVel, depth);
    }
  }
}

  
// RViz 可视化
  
void UGOceanCurrentPlugin::PublishMarkers(
    const ignition::math::Pose3d &_pose,
    const ignition::math::Vector3d &_currentVel,
    double _depth)
{
  if (markerPub_.getNumSubscribers() == 0)
    return;

  visualization_msgs::MarkerArray markerArray;
  ros::Time stamp = ros::Time::now();
  const double ARROW_SCALE = 10.0;  // 放大系数，方便观察

  //    Marker 0: 当前位置的洋流箭头   
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = stamp;
    arrow.ns = namespace_ + "/ocean_current";
    arrow.id = 0;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.lifetime = ros::Duration(1.0);

    // 起点：滑翔机位置
    geometry_msgs::Point start, end;
    start.x = _pose.Pos().X();
    start.y = _pose.Pos().Y();
    start.z = _pose.Pos().Z();

    // 终点：起点 + 洋流方向 × 放大系数
    end.x = start.x + _currentVel.X() * ARROW_SCALE;
    end.y = start.y + _currentVel.Y() * ARROW_SCALE;
    end.z = start.z + _currentVel.Z() * ARROW_SCALE;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    // 箭头粗细
    arrow.scale.x = 0.08;  // shaft diameter
    arrow.scale.y = 0.15;  // head diameter
    arrow.scale.z = 0.2;   // head length

    // 青色
    arrow.color.r = 0.0f;
    arrow.color.g = 0.8f;
    arrow.color.b = 1.0f;
    arrow.color.a = 0.9f;

    markerArray.markers.push_back(arrow);
  }

  //    Marker 1~N: 深度剖面箭头列   
  double profileX = _pose.Pos().X() + 3.0;  // 偏移 3m，不遮挡滑翔机
  double profileY = _pose.Pos().Y();

  for (size_t i = 0; i < layers_.size(); ++i)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = stamp;
    arrow.ns = namespace_ + "/ocean_profile";
    arrow.id = static_cast<int>(i);
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.lifetime = ros::Duration(1.0);

    double z = -layers_[i].depth;  // 转回 Gazebo Z (向上为正)
    const auto &vel = layers_[i].velocity;

    geometry_msgs::Point start, end;
    start.x = profileX;
    start.y = profileY;
    start.z = z;
    end.x = profileX + vel.X() * ARROW_SCALE;
    end.y = profileY + vel.Y() * ARROW_SCALE;
    end.z = z + vel.Z() * ARROW_SCALE;

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    arrow.scale.x = 0.06;
    arrow.scale.y = 0.12;
    arrow.scale.z = 0.15;

    // 颜色渐变：红(浅)到蓝(深)
    float ratio = (layers_.size() > 1)
        ? static_cast<float>(i) / (layers_.size() - 1)
        : 0.0f;
    arrow.color.r = 1.0f - ratio;
    arrow.color.g = 0.2f;
    arrow.color.b = ratio;
    arrow.color.a = 0.8f;

    // 当前深度对应的层高亮（加粗 + 高透明度）
    if (i + 1 < layers_.size() &&
        _depth >= layers_[i].depth && _depth < layers_[i + 1].depth)
    {
      arrow.scale.x = 0.1;
      arrow.scale.y = 0.18;
      arrow.color.a = 1.0f;
      arrow.color.g = 1.0f;  // 黄绿高亮
    }

    markerArray.markers.push_back(arrow);
  }

  markerPub_.publish(markerArray);
}

}  // namespace gazebo
