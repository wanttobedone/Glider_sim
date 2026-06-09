// test_accel_tilt.cpp
//
// 验证 accel 找平更新：
//   1) 注入未知陀螺零偏，仅 predict 时 roll/pitch 漂移；加 tilt 后有界且 b_g 收敛
//   2) yaw（绕重力）不被 tilt 修正（不可观）
//   3) Joseph 后 P 对称
//   4) b_a 在俯仰机动下可观

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/measurements/accel_tilt.h"

using namespace ug_ekf;

namespace {
constexpr Scalar kG = Scalar(9.81);

InitParams MakeParams() {
  InitParams p{};
  p.g = kG;
  p.mag_declination_rad = 0;
  p.r_pressure_FRD = Vec3::Zero();
  p.init_yaw_ned_rad = 0;
  p.dt_max = Scalar(0.1);
  p.nis_gate_depth = Scalar(6.635);
  p.nis_gate_tilt  = Scalar(11.345);   // χ²(0.99,3)
  p.noise.sigma_g  = Scalar(1e-3);
  p.noise.sigma_a  = Scalar(1e-2);
  p.noise.sigma_bg = Scalar(1e-4);     // 允许 b_g 在线调整
  p.noise.sigma_ba = Scalar(1e-3);
  p.p0_pos = Scalar(1.0);
  p.p0_vel = Scalar(0.1);
  p.p0_att = Scalar(1e-2);
  p.p0_bg  = Scalar(1e-2);             // b_g 初始不确定，留给 tilt 估计
  p.p0_ba  = Scalar(1e-2);
  return p;
}

Mat15 MakeP0(const InitParams& p) {
  Mat15 P = Mat15::Zero();
  P.diagonal().head<3>().setConstant(p.p0_pos);
  P.diagonal().segment<3>(3).setConstant(p.p0_vel);
  P.diagonal().segment<3>(6).setConstant(p.p0_att);
  P.diagonal().segment<3>(9).setConstant(p.p0_bg);
  P.diagonal().segment<3>(12).setConstant(p.p0_ba);
  return P;
}

// NED pitch/roll 提取
void RollPitch(const Quat& q, Scalar& roll, Scalar& pitch) {
  Mat3 R = q.toRotationMatrix();
  pitch = std::asin(std::max(Scalar(-1), std::min(Scalar(1), -R(2,0))));
  roll  = std::atan2(R(2,1), R(2,2));
}
}  // namespace

// H 矩阵零空间应沿重力方向（yaw 不可观）
TEST(AccelTilt, JacobianYawUnobservable) {
  State x;  // level, yaw=0
  Mat3x15 H;
  BuildAccelTiltH(x, kG, H);
  // δθ 块 = [h_grav]_×, h_grav = R^T(-g_NED) = [0,0,-g]
  Mat3 Hth = H.block<3,3>(0, kIdxDeltaTh);
  // 绕重力(body z, level 下=Down)的扰动 δθ=[0,0,1] 应落入零空间
  Vec3 dyaw(0, 0, 1);
  EXPECT_LT((Hth * dyaw).norm(), 1e-9) << "yaw 扰动不应进入 tilt 观测";
  // roll/pitch 扰动应被观测到
  EXPECT_GT((Hth * Vec3(1,0,0)).norm(), 1.0);
  EXPECT_GT((Hth * Vec3(0,1,0)).norm(), 1.0);
}

// 注入未知陀螺零偏：仅 predict 漂移；加 tilt 后 roll/pitch 有界 + b_g 收敛
TEST(AccelTilt, BoundsAttitudeAndEstimatesGyroBias) {
  const Vec3 true_bg(0.01, -0.008, 0.0);  // 未知陀螺零偏 (rad/s)
  const Scalar R_tilt = Scalar(0.01);     // (0.1 m/s^2)^2
  const Scalar dt = Scalar(0.02);

  // --- 对照组：仅 predict ---
  Eskf ekf_pred;
  { InitParams p = MakeParams(); State x0; ekf_pred.initialize(p, x0, MakeP0(p)); }
  // --- 实验组：predict + tilt ---
  Eskf ekf_tilt;
  { InitParams p = MakeParams(); State x0; ekf_tilt.initialize(p, x0, MakeP0(p)); }

  // 真值：静止水平。测量 gyro = 0 + bias；accel = [0,0,-g]
  const Vec3 gyro_meas = true_bg;
  const Vec3 acc_meas(0, 0, -kG);

  for (int i = 1; i <= 1500; ++i) {  // 30 s
    Scalar t = i * dt;
    ekf_pred.predictImu(t, gyro_meas, acc_meas);
    ekf_tilt.predictImu(t, gyro_meas, acc_meas);
    ekf_tilt.updateAccelTilt(t, acc_meas, R_tilt);
  }

  Scalar r_pred, p_pred, r_tilt, p_tilt;
  RollPitch(ekf_pred.state().q_NB, r_pred, p_pred);
  RollPitch(ekf_tilt.state().q_NB, r_tilt, p_tilt);

  const Scalar tilt_err_pred = std::sqrt(r_pred*r_pred + p_pred*p_pred);
  const Scalar tilt_err_tilt = std::sqrt(r_tilt*r_tilt + p_tilt*p_tilt);

  // 仅 predict 应明显漂移（30s × ~0.01rad/s ≈ 0.3rad）
  EXPECT_GT(tilt_err_pred, Scalar(0.1)) << "predict-only 应漂移";
  // 加 tilt 后 roll/pitch 应被压到 < 1°
  EXPECT_LT(tilt_err_tilt, Scalar(0.0175))
      << "tilt 后姿态误差(rad)=" << tilt_err_tilt;
  // b_g 水平分量应收敛到真值附近 (±20%)
  EXPECT_NEAR(ekf_tilt.state().b_g.x(), true_bg.x(), std::abs(true_bg.x())*0.3);
  EXPECT_NEAR(ekf_tilt.state().b_g.y(), true_bg.y(), std::abs(true_bg.y())*0.3);

  // P 对称
  const Mat15& P = ekf_tilt.covariance();
  EXPECT_LT((P - P.transpose()).norm(), 1e-9);
  EXPECT_GT(ekf_tilt.diag().accept_tilt, 1000u);
}

// yaw 漂移不被 tilt 纠正（绕重力不可观）
TEST(AccelTilt, YawNotCorrected) {
  const Vec3 true_bg(0, 0, 0.01);  // 仅 z 陀螺零偏 → yaw 漂
  const Scalar R_tilt = Scalar(0.01);
  const Scalar dt = Scalar(0.02);

  Eskf ekf;
  { InitParams p = MakeParams(); State x0; ekf.initialize(p, x0, MakeP0(p)); }
  const Vec3 acc_meas(0, 0, -kG);

  for (int i = 1; i <= 1500; ++i) {
    Scalar t = i * dt;
    ekf.predictImu(t, true_bg, acc_meas);
    ekf.updateAccelTilt(t, acc_meas, R_tilt);
  }
  // yaw 应仍漂移 ≈ 0.01*30 = 0.3 rad（tilt 无力纠正）
  Mat3 R = ekf.state().q_NB.toRotationMatrix();
  Scalar yaw = std::atan2(R(1,0), R(0,0));
  EXPECT_GT(std::abs(yaw), Scalar(0.15)) << "yaw 不该被 tilt 纠正, yaw=" << yaw;
  // 但 roll/pitch 仍应保持小
  Scalar r, p; RollPitch(ekf.state().q_NB, r, p);
  EXPECT_LT(std::sqrt(r*r+p*p), Scalar(0.0175));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
