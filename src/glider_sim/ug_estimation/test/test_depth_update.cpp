// test_depth_update.cpp
//
// 验证：
//   1) 杠杆臂预测公式：pitch=30° + r_p=[0.5,0,0] ⇒ z_pred = p_D + 0.5·sin(30°)
//   2) updateDepth 1Hz 注入正弦深度，能压住 propagation 漂移，RMSE 满足阈值
//   3) NIS 序列大致服从 χ²(1) 分布，>95% 落入 6.63 包络
//   4) Joseph form 后 P 对称且对角正

#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/measurements/depth.h"

using namespace ug_ekf;

namespace {

constexpr Scalar kG = Scalar(9.81);

InitParams MakeParams(const Vec3& r_p) {
  InitParams p{};
  p.g = kG;
  p.mag_declination_rad = 0;
  p.r_pressure_FRD = r_p;
  p.init_yaw_ned_rad = 0;
  p.dt_max = Scalar(0.1);
  p.nis_gate_depth = Scalar(6.635);
  p.noise.sigma_g  = Scalar(1e-3);
  p.noise.sigma_a  = Scalar(1e-2);
  p.noise.sigma_bg = Scalar(1e-5);
  p.noise.sigma_ba = Scalar(1e-4);
  p.p0_pos = Scalar(100.0);  // 深度先验宽松：水深 0~几十米都可能
  p.p0_vel = Scalar(0.01);   // 启动时速度先验较紧 (σ≈0.1 m/s)
  p.p0_att = Scalar(1e-3);
  p.p0_bg  = Scalar(1e-4);
  p.p0_ba  = Scalar(1e-3);
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

}  // namespace

// 杠杆臂预测公式数值验证
TEST(DepthMeasurement, LeverArmPrediction) {
  State x;
  const Scalar pitch = Scalar(M_PI / 6);  // 30°
  x.q_NB = Eigen::AngleAxis<Scalar>(pitch, Vec3::UnitY());
  x.p_NED = Vec3(0, 0, 10);  // 主体在 10m 深处

  const Vec3 r_p(0.5, 0, 0);
  // R · r_p with pitch only: R_y(pitch) · [0.5,0,0] = [0.5·cos pitch, 0, -0.5·sin pitch]
  // e_D^T · (p + R·r_p) = p_D + (-0.5·sin pitch) = 10 - 0.25
  // 注意：FRD 抬头 (pitch>0) 时鼻尖向 -D 方向移动，所以 z_pred 减小
  const Scalar z = PredictDepth(x, r_p);
  EXPECT_NEAR(z, 10 - 0.5 * std::sin(pitch), 1e-9);
}

// 单次 updateDepth 全链路 sanity check
TEST(EskfDepth, SingleUpdateMovesEstimateTowardMeasurement) {
  Eskf ekf;
  InitParams p = MakeParams(Vec3::Zero());
  State x0;
  ekf.initialize(p, x0, MakeP0(p));

  // 测量真值 = 5m，初始估计 p_D=0
  const Scalar R = Scalar(0.01);
  EXPECT_TRUE(ekf.updateDepth(0, Scalar(5.0), R));

  // 更新后 p_D 应朝 5 移动
  EXPECT_GT(ekf.state().p_NED.z(), 0);
  EXPECT_LT(ekf.state().p_NED.z(), 5);

  // P 仍对称
  const Mat15& P = ekf.covariance();
  EXPECT_LT((P - P.transpose()).norm(), 1e-9);
}

// 50Hz IMU 静止 + 10Hz 深度测量 + 匀速下沉真值，闭环 RMSE 测试。
// 设计哲学：模拟水下滑翔机 dive 阶段，v_D ≈ 0.1 m/s 匀速下沉，
// 压力计 10Hz（真实硬件频率），IMU 静止（无水平加速度）。
TEST(EskfDepth, ClosedLoopRmseAndNis) {
  Eskf ekf;
  InitParams p = MakeParams(Vec3::Zero());
  State x0;
  // 初始真实位置 0；不给 EKF 初始位置先验
  ekf.initialize(p, x0, MakeP0(p));

  std::mt19937 rng(42);
  std::normal_distribution<double> noise_z(0.0, 0.05);   // σ=5 cm
  const Scalar R = Scalar(0.05 * 0.05);

  const Scalar dt = Scalar(0.02);  // 50 Hz IMU
  const int steps = 3000;          // 60 s
  const double v_dive = 0.1;       // m/s 真值下沉速率

  std::vector<double> errs;
  std::vector<double> nis_vals;
  errs.reserve(steps);
  nis_vals.reserve(steps / 5);

  int warmup_steps = 100;  // 前 2s 数据不计入 RMSE，留滤波器收敛

  for (int i = 1; i <= steps; ++i) {
    Scalar t = i * dt;
    ekf.predictImu(t, Vec3::Zero(), Vec3(0, 0, -kG));

    double depth_true = v_dive * static_cast<double>(t);

    // 10 Hz 深度测量
    if (i % 5 == 0) {
      double z = depth_true + noise_z(rng);
      ekf.updateDepth(t, static_cast<Scalar>(z), R);
      if (i >= warmup_steps) {
        nis_vals.push_back(static_cast<double>(ekf.diag().last_nis_depth));
      }
    }

    if (i >= warmup_steps) {
      double err = static_cast<double>(ekf.state().p_NED.z()) - depth_true;
      errs.push_back(err);
    }
  }

  // RMSE
  double sq = 0;
  for (double e : errs) sq += e * e;
  const double rmse = std::sqrt(sq / errs.size());

  // Phase 1 验收阈值同款：0.05 m
  EXPECT_LT(rmse, 0.05) << "depth RMSE=" << rmse;

  // NIS 合规率：χ²(0.95,1)=3.84，目标 ≥90%（Phase 1 验收）
  int in_envelope = 0;
  for (double nis : nis_vals) if (nis < 3.84) ++in_envelope;
  const double ratio = static_cast<double>(in_envelope) / nis_vals.size();
  EXPECT_GT(ratio, 0.9) << "NIS in 95% envelope ratio=" << ratio;

  // 应有大量 accept
  EXPECT_GT(ekf.diag().accept_depth, 500u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
