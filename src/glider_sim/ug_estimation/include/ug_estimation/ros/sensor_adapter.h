// ug_estimation/ros/sensor_adapter.h
//
// ROS ↔ EKF core 的传感器适配层（仅 wrapper 编译，core 永不见此文件）。
// 职责：把 ROS/Gazebo body-FLU 原始量转成 EKF core 所需 body-FRD raw 量。
//
// 硬约束 (C2)
// 不读取 imu.orientation。Gazebo IMU 的姿态是真值
// 真实嵌入式 IMU 不提供，core 必须只吃 angular_velocity + linear_acceleration。
//
// 轴翻转 FLU → FRD
//   FLU: X前 Y左 Z上   FRD: X前 Y右 Z下
//   翻转矩阵 diag(1,-1,-1)：
//     gyro_FRD  = [ ωx, -ωy, -ωz]
//     accel_FRD = [ ax, -ay, -az]

#pragma once

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include "ug_estimation/core/types.h"

namespace ug_ekf {
namespace ros_adapter {

// FLU → FRD 轴翻转
inline Vec3 FluToFrd(double x, double y, double z) {
  return Vec3(static_cast<Scalar>(x),
              static_cast<Scalar>(-y),
              static_cast<Scalar>(-z));
}

// 从 sensor_msgs/Imu 取角速度，转 FRD。不读 orientation。
inline Vec3 GyroFrdFromImu(const sensor_msgs::Imu& msg) {
  return FluToFrd(msg.angular_velocity.x,
                  msg.angular_velocity.y,
                  msg.angular_velocity.z);
}

// 从 sensor_msgs/Imu 取比力（含重力），转 FRD。不读 orientation。
inline Vec3 AccelFrdFromImu(const sensor_msgs::Imu& msg) {
  return FluToFrd(msg.linear_acceleration.x,
                  msg.linear_acceleration.y,
                  msg.linear_acceleration.z);
}

// 压力 → 深度 (m)，水下为正。
//   depth = (P - P_atm) / (rho * g)
inline Scalar DepthFromPressure(const sensor_msgs::FluidPressure& msg,
                                Scalar p_atm, Scalar rho, Scalar g) {
  return static_cast<Scalar>(
      (msg.fluid_pressure - p_atm) / (rho * g));
}

}  // namespace ros_adapter
}  // namespace ug_ekf
