// ug_estimation/core/eskf.h
//
// 平台无关 ESKF 主类，部分仅声明，实现分阶段填充
// API 不暴露 ROS 类型，不抛异常，不分配堆。

#pragma once

#include "ug_estimation/core/types.h"

namespace ug_ekf {

class Eskf {
 public:
  Eskf();

  // 一次性初始化 
  void initialize(const InitParams& params, const State& x0, const Mat15& P0);

  // IMU 推进
  // 输入FRD 系角速度、比力（含重力）；t 为传感器时间戳 (s)
  void predictImu(Scalar t, const Vec3& gyro_FRD, const Vec3& accel_FRD);

  // 测量更新，返回是否被接受 (false = NIS 拒绝)
  bool updateDepth(Scalar t, Scalar depth_m, Scalar R);
  // accel 找平：低加速度段用重力方向约束 roll/pitch（yaw 不可观）。
  // R_tilt 为每轴比力测量方差 (m/s^2)^2，需含残余线加速度的保守膨胀。
  // 调用方负责运动门控（|‖a‖-g|<ε_a 且 |gyro|<ε_ω）。
  bool updateAccelTilt(Scalar t, const Vec3& accel_FRD, Scalar R_tilt);
  bool updateMag  (Scalar t, const Vec3& mag_FRD, Scalar R_yaw);
  bool updateGps  (Scalar t, const Vec2& pNE, const Mat2& R);        // Phase 3
  bool updateDvl  (Scalar t, const Vec3& v_FRD, const Mat3& R);      // Phase 3 

  // 静态对齐，从静止段累积 accel/gyro/mag 估计初始 RPY 与 b_g
  // 内部不分配；wrapper 持有定长缓冲区指针传入
  // ROS里面用，下面的是嵌入式里面用的
  void staticAlign(const Vec3* acc_buf, const Vec3* gyro_buf,
                   const Vec3* mag_buf, std::size_t n);

  // 静态对齐（增量版，O(1) RAM）：直接传静止段均值。
  // mag_mean 为 nullptr 时 yaw 回退 init_yaw_ned_rad。
  // 嵌入式 wrapper 用累加和/计数即可，无需缓存全部样本。
  void staticAlignFromMeans(const Vec3& acc_mean, const Vec3& gyro_mean,
                            const Vec3* mag_mean);

  // 访问器
  const State&        state()       const { return x_; }
  const Mat15&        covariance()  const { return P_; }
  const Diagnostics&  diag()        const { return diag_; }
  bool                initialized() const { return initialized_; }

 private:
  bool injectErrorState(const Vec15& dx);
  void enforceSymmetry();

  State        x_;
  Mat15        P_ = Mat15::Identity();
  InitParams   params_{};
  Diagnostics  diag_{};
  bool         initialized_ = false;
  Scalar       t_prev_ = Scalar(-1);
};

}  // namespace ug_ekf
