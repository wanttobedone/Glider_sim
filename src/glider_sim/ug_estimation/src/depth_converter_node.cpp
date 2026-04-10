/**
 * depth_converter_node
 *
 * 将 UUV 压力传感器的 sensor_msgs/FluidPressure 转换为
 * geometry_msgs/PoseWithCovarianceStamped，供 robot_localization EKF使用
 *
 * 输入/{ns}/pressure  [sensor_msgs/FluidPressure]
 * 输出/{ns}/depth/pose [geometry_msgs/PoseWithCovarianceStamped]
 *
 * 深度计算depth = (pressure - P_atm) / (rho * g)
 * ENU 输出z = -depth (水下为负)
 */

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class DepthConverter
{
public:
  DepthConverter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param<double>("atmospheric_pressure", atmosPressure_, 101325.0);  // Pa
    pnh.param<double>("water_density", waterDensity_, 1028.0);            // kg/m^3
    pnh.param<double>("gravity", gravity_, 9.81);                         // m/s^2
    pnh.param<double>("depth_variance", depthVariance_, 0.01);            // m^2
    pnh.param<std::string>("output_frame", outputFrame_, "odom");

    depthPub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("depth/pose", 10);
    pressureSub_ = nh.subscribe("pressure", 10, &DepthConverter::pressureCb, this);

    ROS_INFO("[DepthConverter] P_atm=%.0f Pa, rho=%.1f kg/m3, frame=%s",
             atmosPressure_, waterDensity_, outputFrame_.c_str());
  }

private:
  void pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg)
  {
    double depth = (msg->fluid_pressure - atmosPressure_) / (waterDensity_ * gravity_);

    geometry_msgs::PoseWithCovarianceStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = outputFrame_;

    // ENU水下 z 为负
    out.pose.pose.position.x = 0.0;
    out.pose.pose.position.y = 0.0;
    out.pose.pose.position.z = -depth;
    out.pose.pose.orientation.w = 1.0;

    // 只有 z 的协方差有意义 (index 14 = [2][2])
    out.pose.covariance[14] = depthVariance_;

    depthPub_.publish(out);
  }

  ros::Publisher depthPub_;
  ros::Subscriber pressureSub_;
  double atmosPressure_;
  double waterDensity_;
  double gravity_;
  double depthVariance_;
  std::string outputFrame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_converter_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  DepthConverter converter(nh, pnh);
  ros::spin();
  return 0;
}
