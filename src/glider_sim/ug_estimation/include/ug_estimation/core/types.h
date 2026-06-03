// ug_estimation/core/types.h
//
// 平台无关核心类型。允许：Eigen 定长矩阵、std::array、POD 结构。
// 禁用：动态容器、ROS 类型、iostream、异常、RTTI。
//
// 标量精度通过宏 UG_EKF_USE_FP32 切换：
//   - 默认 (sim)        : double
//   - -DUG_EKF_USE_FP32 : float (用于 STM32 移植阶段)

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace ug_ekf {

#ifdef UG_EKF_USE_FP32
using Scalar = float;
#else
using Scalar = double;
#endif

using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
using Vec15 = Eigen::Matrix<Scalar, 15, 1>;

using Mat2 = Eigen::Matrix<Scalar, 2, 2>;
using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using Mat15 = Eigen::Matrix<Scalar, 15, 15>;

using Quat = Eigen::Quaternion<Scalar>;

// Nominal state (维度 16，分量分别为 p, v, q, b_g, b_a)
struct State {
  Vec3 p_NED;       // [N, E, D] (m)
  Vec3 v_NED;       // [vN, vE, vD] (m/s)
  Quat q_NB;        // body→NED 单位四元数 (Eigen 顺序: x,y,z,w)
  Vec3 b_g;         // gyro bias (rad/s)
  Vec3 b_a;         // accel bias (m/s^2)

  State() : p_NED(Vec3::Zero()), v_NED(Vec3::Zero()),
            q_NB(Quat::Identity()),
            b_g(Vec3::Zero()), b_a(Vec3::Zero()) {}
};

// Error state δx 索引（Joseph form 用）
//   [δp(0..2), δv(3..5), δθ(6..8), δb_g(9..11), δb_a(12..14)]
constexpr int kIdxDeltaP  = 0;
constexpr int kIdxDeltaV  = 3;
constexpr int kIdxDeltaTh = 6;
constexpr int kIdxDeltaBg = 9;
constexpr int kIdxDeltaBa = 12;
constexpr int kErrDim     = 15;

// 噪声密度（连续时间）
struct NoiseParams {
  Scalar sigma_g;        // gyro 白噪声 (rad/s/sqrt(Hz))
  Scalar sigma_a;        // accel 白噪声 (m/s^2/sqrt(Hz))
  Scalar sigma_bg;       // gyro bias 随机游走 (rad/s^2/sqrt(Hz))
  Scalar sigma_ba;       // accel bias 随机游走 (m/s^3/sqrt(Hz))
};

struct InitParams {
  Scalar g;                       // 重力加速度模 (m/s^2)，NED 下 g_NED=[0,0,+g]
  Scalar mag_declination_rad;     // 磁偏角 (rad)，对齐 mag yaw 时使用
  Vec3   r_pressure_FRD;          // 压力计在 FRD body 坐标 (m)，含杠杆臂

  // 初始 NED yaw (rad)；Phase 1 无 mag/航向观测，由 wrapper 显式给定。
  // 严禁直接把 Gazebo ENU yaw 写入此处，必须先做 ENU→NED 转换。
  Scalar init_yaw_ned_rad;

  // dt 上限 (s)，IMU 跳帧/启动时超出则跳过该次 predict 并 reset_count++
  Scalar dt_max;

  // 深度 NIS 门限（默认 χ²(0.99,1)=6.635）
  Scalar nis_gate_depth;

  NoiseParams noise;

  // 初始协方差对角元
  Scalar p0_pos;    // m^2
  Scalar p0_vel;    // (m/s)^2
  Scalar p0_att;    // rad^2
  Scalar p0_bg;     // (rad/s)^2
  Scalar p0_ba;     // (m/s^2)^2
};

// EKF 内部诊断信息（wrapper 决定如何序列化、发布）
struct Diagnostics {
  Scalar last_nis_depth = 0;
  Scalar last_nis_mag   = 0;
  Scalar last_nis_gps   = 0;
  Scalar last_nis_dvl   = 0;

  uint32_t accept_depth = 0, reject_depth = 0;
  uint32_t accept_mag   = 0, reject_mag   = 0;
  uint32_t accept_gps   = 0, reject_gps   = 0;
  uint32_t accept_dvl   = 0, reject_dvl   = 0;

  uint32_t reset_count = 0;
  Scalar   dt_last = 0;
  Scalar   t_last  = 0;
};

}  // namespace ug_ekf
