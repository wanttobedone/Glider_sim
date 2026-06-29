// ug_estimation/src/core/eskf.cpp
//
// 已经实现IMU 预测 + 深度更新 + 静态对齐。Phase 3 接口占位。
// 平台无关；无 ROS；无堆分配；无异常；无 RTTI。

#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/quaternion_utils.h"
#include "ug_estimation/core/imu_propagator.h"
#include "ug_estimation/core/measurements/depth.h"
#include "ug_estimation/core/measurements/accel_tilt.h"
#include "ug_estimation/core/measurements/mag.h"

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

  // 首帧，仅记录时间戳，不做传播
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

  // 3) 协方差传播 P = F P F^T + Q，动力学传播
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
    diag_.depth_reject_streak++;
    return false;
  }

  const Scalar nis = y * y / S;
  diag_.last_nis_depth = nis;

  if (nis > params_.nis_gate_depth) {
    diag_.reject_depth++;
    diag_.depth_reject_streak++;
    // 失锁恢复：连续拒绝次数达门限，判定垂直通道预测量发散，把深度对齐到测量、垂直速度清零、
    // 解相关并把深度/垂直速度方差恢复到初值（PX4 resetHeight 思路），使后续量测重新被信任
    if (params_.depth_reset_streak > Scalar(0) &&
        Scalar(diag_.depth_reject_streak) >= params_.depth_reset_streak) {
      x_.p_NED.z() = depth_m;
      x_.v_NED.z() = Scalar(0);
      P_.row(kIdxDeltaP + 2).setZero();  P_.col(kIdxDeltaP + 2).setZero();
      P_.row(kIdxDeltaV + 2).setZero();  P_.col(kIdxDeltaV + 2).setZero();
      P_(kIdxDeltaP + 2, kIdxDeltaP + 2) = params_.p0_pos;
      P_(kIdxDeltaV + 2, kIdxDeltaV + 2) = params_.p0_vel;
      enforceSymmetry();
      diag_.depth_reset_count++;
      diag_.depth_reject_streak = 0;
    }
    return false;
  }

  // Kalman gain K = P H^T / S (15x1)
  const Eigen::Matrix<Scalar, 15, 1> K = (P_ * H.transpose()) / S;

  // 误差状态注入
  const Vec15 dx = K * y;
  injectErrorState(dx);

  // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T，预测更新协方差
  const Mat15 I15 = Mat15::Identity();
  const Mat15 IKH = I15 - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  enforceSymmetry();

  diag_.accept_depth++;
  diag_.depth_reject_streak = 0;
  return true;
}
//加速度计作为观测量，在机体近似静止情况下修正delt_theta
bool Eskf::updateAccelTilt(Scalar /*t*/, const Vec3& accel_FRD, Scalar R_tilt) {
  if (!initialized_) return false;

  Mat3x15 H;
  BuildAccelTiltH(x_, params_.g, H);

  const Vec3 h = PredictAccelSpecificForce(x_, params_.g);
  const Vec3 y = accel_FRD - h;                       // innovation (3x1)

  // S = H P H^T + R·I3
  Mat3 S = H * P_ * H.transpose();
  S.diagonal().array() += R_tilt;

  // NIS = y^T S^-1 y
  const Mat3 S_inv = S.inverse();
  const Scalar nis = (y.transpose() * S_inv * y)(0, 0);
  diag_.last_nis_tilt = nis;

  if (!(nis == nis) || nis > params_.nis_gate_tilt) {  // NaN 或超门限
    diag_.reject_tilt++;
    return false;
  }

  // K = P H^T S^-1  (15x3)
  const Eigen::Matrix<Scalar, 15, 3> K = P_ * H.transpose() * S_inv;

  // 注入
  const Vec15 dx = K * y;
  injectErrorState(dx);

  // Joseph form: P = (I-KH)P(I-KH)^T + K R K^T，R = R_tilt·I3
  const Mat15 I15 = Mat15::Identity();
  const Mat15 IKH = I15 - K * H;
  P_ = IKH * P_ * IKH.transpose() + (K * R_tilt) * K.transpose();
  enforceSymmetry();

  diag_.accept_tilt++;
  return true;
}

bool Eskf::updateMag(Scalar /*t*/, const Vec3& mag_FRD, Scalar R_yaw) {
  if (!initialized_) return false;
  if (mag_FRD.norm() < Scalar(1e-12)) { diag_.reject_mag++; return false; }

  RowVec15 H;
  BuildMagYawH(x_, H);

  const Scalar y = MagYawInnovation(x_, mag_FRD, params_.mag_declination_rad);

  // S = H P H^T + R (标量)
  const Scalar S = (H * P_ * H.transpose())(0, 0) + R_yaw;
  if (S <= Scalar(0)) { diag_.reject_mag++; return false; }

  const Scalar nis = y * y / S;
  diag_.last_nis_mag = nis;
  if (nis > params_.nis_gate_mag) { diag_.reject_mag++; return false; }

  const Eigen::Matrix<Scalar, 15, 1> K = (P_ * H.transpose()) / S;
  injectErrorState(K * y);

  const Mat15 I15 = Mat15::Identity();
  const Mat15 IKH = I15 - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R_yaw * K.transpose();
  enforceSymmetry();

  diag_.accept_mag++;
  return true;
}

bool Eskf::updateGps(Scalar /*t*/, const Vec2& /*pNE*/, const Mat2& /*R*/) {
  // TODO Phase 3
  return false;
}

bool Eskf::updateDvl(Scalar /*t*/, const Vec3& /*v_FRD*/, const Mat3& /*R*/) {
  //  后续可补充DVL
  return false;
}

// 静态对齐，水平部署假设捕获 b_a
//
// 背景：MEMS 加速度计有 ~40mg 常值零偏，单测静止 accel 无法把"零偏"和"倾角"
//   分开。Phase 1 深度更新下水平 b_a 不可观，若按 accel→tilt 反推姿态会把
//   零偏误当成 ~2.3° 倾角， roll/pitch≤1° 阈值
//
// 真机标准做法（放平标定）：入水/部署时滑翔机近似水平，于是
//   - 姿态：roll=pitch=0，yaw=先验 init_yaw_ned_rad
//   - 加计零偏：静止时 a_NED 应=0 ⇒ R_NB·(a_mean - b_a) + g_NED = 0
//     水平(绕 Down 轴 yaw 旋转不改 [0,0,-g]) ⇒ a_mean - b_a = [0,0,-g]
//     ⇒ b_a = a_mean - [0,0,-g] = a_mean + [0,0,+g]
//   - 陀螺零偏：b_g = mean(gyro)（静止真角速度=0）
//
// 代价若部署时真有安装倾角，会被误记进 b_a；对水平部署的滑翔机可接受
//   （本仿真 ground_truth 静止姿态 roll=0/pitch=-0.48°，残差<0.5°）。
//   后续如需建模安装角，可加 init_roll/init_pitch 先验参数扣除。
//
// 准入条件由 wrapper 端筛样本，core 假设 caller 已确保静止。
void Eskf::staticAlign(const Vec3* acc_buf, const Vec3* gyro_buf,
                       const Vec3* mag_buf, std::size_t n) {
  if (!initialized_ || n == 0) return;

  Vec3 a_sum = Vec3::Zero();
  Vec3 g_sum = Vec3::Zero();
  Vec3 m_sum = Vec3::Zero();
  for (std::size_t i = 0; i < n; ++i) {
    a_sum += acc_buf[i];
    g_sum += gyro_buf[i];
    if (mag_buf) m_sum += mag_buf[i];
  }
  const Scalar inv_n = Scalar(1) / static_cast<Scalar>(n);
  const Vec3 a_mean = a_sum * inv_n;
  const Vec3 g_mean = g_sum * inv_n;
  const Vec3 m_mean = m_sum * inv_n;
  staticAlignFromMeans(a_mean, g_mean, mag_buf ? &m_mean : nullptr);
}

void Eskf::staticAlignFromMeans(const Vec3& acc_mean, const Vec3& gyro_mean,
                                const Vec3* mag_mean) {
  if (!initialized_) return;

  // yaw，有 mag 则用 level 假设从磁场恢复，否则回退先验 init_yaw_ned_rad
  Scalar yaw = params_.init_yaw_ned_rad;
  if (mag_mean && mag_mean->norm() > Scalar(1e-12)) {
    yaw = MagInitYaw(*mag_mean, params_.mag_declination_rad);
  }

  // 姿态：水平 + yaw（仅绕 Down 轴）
  x_.q_NB = Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitZ());
  x_.q_NB.normalize();

  // 陀螺零偏 = 静止角速度均值
  x_.b_g = gyro_mean;

  // 加计零偏 = a_mean - [0,0,-g]（水平静止下的期望比力为 [0,0,-g]_FRD）
  const Vec3 expected_specific_force(Scalar(0), Scalar(0), -params_.g);
  x_.b_a = acc_mean - expected_specific_force;
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
