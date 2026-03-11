/**
 * 异常场报警，含目标NED坐标                  
 *
 * 在海底放置一个虚拟目标，监听所有 Glider 的 NED 位置，
 * 当某台 Glider 进入探测半径时，向该机定向发布一次报警（携带目标坐标）。
 * 目标的 RViz 可视化（红色圆柱 + 探测球）由 swarm_viz_node 负责。
 *
 * 参数/swarm/ 命名空间：
 *   anomaly_north          (double)    目标 north [m, NED]
 *   anomaly_east           (double)    目标 east  [m, NED]
 *   anomaly_depth          (double)    目标 depth [m, NED, 正值]
 *   anomaly_detect_radius  (double)    探测半径 [m]
 *   glider_namespaces      (string[])  滑翔机命名空间列表
 *
 * 订阅：
 *   /{ns}/glider_state     (GliderState)      每台 Glider 的 NED 状态
 *
 * 发布：
 *   /{ns}/sensor/alert     (PointStamped)     定向报警，point 为目标 NED 坐标，只发一次
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <ug_msgs/GliderState.h>
#include <cmath>
#include <vector>
#include <string>
#include <set>
#include <clocale>

class AnomalyFieldNode
{
public:
  AnomalyFieldNode()
  {
    ros::NodeHandle nh("/swarm");

    nh.param("anomaly_north", targetNorth_, 150.0);
    nh.param("anomaly_east", targetEast_, 12.5);
    nh.param("anomaly_depth", targetDepth_, 25.0);
    nh.param("anomaly_detect_radius", detectRadius_, 15.0);

    std::vector<std::string> namespaces;
    nh.getParam("glider_namespaces", namespaces);
    if (namespaces.empty())
    {
      ROS_ERROR("[AnomalyField] glider_namespaces 参数为空！");
      return;
    }

    ros::NodeHandle nhGlobal;
    for (int k = 0; k < (int)namespaces.size(); k++)
    {
      const auto &ns = namespaces[k];

      stateSubs_.push_back(
          nhGlobal.subscribe<ug_msgs::GliderState>(
              "/" + ns + "/glider_state", 1,
              [this, k, ns](const ug_msgs::GliderState::ConstPtr &msg)
              { onGliderState(k, ns, msg); }));

      alertPubs_.push_back(
          nhGlobal.advertise<geometry_msgs::PointStamped>("/" + ns + "/sensor/alert", 1, true));
    }

    ROS_INFO("[AnomalyField] 目标位置: (%.1f, %.1f, %.1f), Glider探测半径: %.1fm",
             targetNorth_, targetEast_, targetDepth_, detectRadius_);
  }

private:
  void onGliderState(int idx, const std::string &ns,
                     const ug_msgs::GliderState::ConstPtr &msg)
  {
    if (alerted_.count(idx))
      return;

    double dn = msg->north - targetNorth_;
    double de = msg->east - targetEast_;
    double dd = msg->depth - targetDepth_;
    double dist = std::sqrt(dn * dn + de * de + dd * dd);

    if (dist <= detectRadius_)
    {
      geometry_msgs::PointStamped alert;
      alert.header.stamp = ros::Time::now();
      alert.point.x = targetNorth_;
      alert.point.y = targetEast_;
      alert.point.z = targetDepth_;
      alertPubs_[idx].publish(alert);
      alerted_.insert(idx);

      ROS_WARN("[AnomalyField] %s 进入探测半径！距离: %.1fm", ns.c_str(), dist);
    }
  }

  double targetNorth_, targetEast_, targetDepth_;
  double detectRadius_;

  std::vector<ros::Subscriber> stateSubs_;
  std::vector<ros::Publisher> alertPubs_;
  std::set<int> alerted_;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "anomaly_field_node");
  AnomalyFieldNode node;
  ros::spin();
  return 0;
}
