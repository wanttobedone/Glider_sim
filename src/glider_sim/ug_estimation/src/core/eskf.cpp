// ug_estimation/src/core/eskf.cpp
//
// Phase 1 实现：IMU 推进 + 深度更新 + 静态对齐。Phase 2/3 接口仍占位。
// 平台无关；无 ROS；无堆分配；无异常；无 RTTI。

#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/quaternion_utils.h"
#include "ug_estimation/core/imu_propagator.h"
#include "ug_estimation/core/measurements/depth.h"

#include <cmath>

namespace ug_ekf {

Eskf::Eskf() = default;

void Eskf::initialize(const InitParams& params, const State& x0, const Mat15& P0) {
  params_ = params;
  x_ = x0;
  P_ = P0;
  diag_ = Diagnostics{};
  t_prev_ = Scalar(-1);
  initialized_ = true;
  enforceSymmetry();
}

void Eskf::predictImu(Scalar t, const Vec3& gyro_FRD, const Vec3& accel_FRD) {
  if (!initialized_) return;

  // 首帧：仅记录时间戳，不做传播
  if (t_prev_ < Scalar(0)) {
    t_prev_ = t;
    diag_.t_last = t;
    diag_.dt_last = Scalar(0);
    return;
  }

  Scalar dt = t - t_prev_;
  if (dt <= Scalar(0)) {
    // 时间戳倒退或重复，跳过
    return;
  }
  if (dt > params_.dt_max) {
    // 跳帧过大：重置 t_prev，不传播
    diag_.reset_count++;
    t_prev_ = t;
    diag_.t_last = t;
    diag_.dt_last = dt;
    return;
  }

  // 1) 用传播前的状态构造 F/Q（线性化点）
  Mat15 F;
  Mat15 Q;
  BuildErrorJacobians(x_, gyro_FRD, accel_FRD, params_.noise, dt, F, Q);

  // 2) 名义传播
  PropagateNominal(x_, gyro_FRD, accel_FRD, params_.g, dt);

  // 3) 协方差传播 P = F P F^T + Q
  P_ = F * P_ * F.transpose() + Q;
  enforceSymmetry();

  t_prev_ = t;
  diag_.t_last = t;
  diag_.dt_last = dt;
}

bool Eskf::updateDepth(Scalar t, Scalar depth_m, Scalar R) {
  if (!initialized_) return false;
  (void)t;  // 深度测量时间戳由 caller 用于诊断；预测协方差已由 predict 推到当前

  RowVec15 H;
  BuildDepthH(x_, params_.r_pressure_FRD, H);

  const Scalar z_pred = PredictDepth(x_, params_.r_pressure_FRD);
  const Scalar y = depth_m - z_pred;              // innovation

  // S = H P H^T + R (标量)
  const Scalar S = (H * P_ * H.transpose())(0, 0) + R;
  if (S <= Scalar(0)) {
    diag_.reject_depth++;
    return false;
  }

  const Scalar nis = y * y / S;
  diag_.last_nis_depth = nis;

  if (nis > params_.nis_gate_depth) {
    diag_.reject_depth++;
    return false;
  }

  // Kalman gain K = P H^T / S (15x1)
  const Eigen::Matrix<Scalar, 15, 1> K = (P_ * H.transpose()) / S;

  // 误差状态注入
  const Vec15 dx = K * y;
  injectErrorState(dx);

  // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
  const Mat15 I15 = Mat15::Identity();
  const Mat15 IKH = I15 - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  enforceSymmetry();

  diag_.accept_depth++;
  return true;
}

bool Eskf::updateMag(Scalar /*t*/, const Vec3& /*mag_FRD*/, Scalar /*R_yaw*/) {
  // TODO Phase 2
  return false;
}

bool Eskf::updateGps(Scalar /*t*/, const Vec2& /*pNE*/, const Mat2& /*R*/) {
  // TODO Phase 3
  return false;
}

bool Eskf::updateDvl(Scalar /*t*/, const Vec3& /*v_FRD*/, const Mat3& /*R*/) {
  // TODO Phase 3 (可选)
  return false;
}

// 静态对齐：
//   roll  = atan2(-a_y, -a_z)                       (FRD 体系，静止 a≈[0,0,-g])
//   pitch = atan2(a_x, sqrt(a_y^2 + a_z^2))
//   yaw   = params_.init_yaw_ned_rad                (无 mag 时直接用先验)
//   b_g   = mean(gyro)
// 准入条件（在 wrapper 端筛选样本，core 假设 caller 已确保静止）：
//   |gyro|<0.02 rad/s 且 ||a|-g|<0.2 m/s²
void Eskf::staticAlign(const Vec3* acc_buf, const Vec3* gyro_buf,
                       const Vec3* /*mag_buf*/, std::size_t n) {
  if (!initialized_ || n == 0) return;

  Vec3 a_sum = Vec3::Zero();
  Vec3 g_sum = Vec3::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    a_sum += acc_buf[i];
    g_sum += gyro_buf[i];
  }
  const Scalar inv_n = Scalar(1) / static_cast<Scalar>(n);
  const Vec3 a_mean = a_sum * inv_n;
  const Vec3 g_mean = g_sum * inv_n;

  // 从加速度恢复 roll/pitch（假设 a_mean ≈ [0,0,-g]_FRD）
  // 等价公式 (Tait-Bryan ZYX):
  //   pitch = atan2( a_x, sqrt(a_y^2 + a_z^2) )    (a_x 增大 ⇒ 抬头 ⇒ pitch+，沿 NED 约定)
  //   roll  = atan2(-a_y, -a_z)
  const Scalar ax = a_mean.x();
  const Scalar ay = a_mean.y();
  const Scalar az = a_mean.z();
  const Scalar pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));
  const Scalar roll  = std::atan2(-ay, -az);
  const Scalar yaw   = params_.init_yaw_ned_rad;

  // 用 ZYX (yaw-pitch-roll) 构造 q_NB
  const Eigen::AngleAxis<Scalar> Rz(yaw,   Vec3::UnitZ());
  const Eigen::AngleAxis<Scalar> Ry(pitch, Vec3::UnitY());
  const Eigen::AngleAxis<Scalar> Rx(roll,  Vec3::UnitX());
  x_.q_NB = (Rz * Ry * Rx);
  x_.q_NB.normalize();

  // b_g 直接用 gyro 均值（静止 ⇒ 真角速度=0）
  x_.b_g = g_mean;
}

bool Eskf::injectErrorState(const Vec15& dx) {
  x_.p_NED += dx.segment<3>(kIdxDeltaP);
  x_.v_NED += dx.segment<3>(kIdxDeltaV);
  x_.q_NB   = InjectDeltaTheta(x_.q_NB, dx.segment<3>(kIdxDeltaTh));
  x_.b_g   += dx.segment<3>(kIdxDeltaBg);
  x_.b_a   += dx.segment<3>(kIdxDeltaBa);
  return true;
}

void Eskf::enforceSymmetry() {
  P_ = Scalar(0.5) * (P_ + P_.transpose());
}

}  // namespace ug_ekf
