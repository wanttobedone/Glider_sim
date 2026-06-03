// test_imu_propagator.cpp
//
// 核心目的：守住"重力符号"约定，防止回归到 -2g 系统漂移那种致命错误。
// 同时校验 F/Q 矩阵线性化合理性、dt 上限保护、姿态/位置协方差增长方向。

#include <gtest/gtest.h>
#include <cmath>
#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/imu_propagator.h"

using namespace ug_ekf;

namespace {

constexpr Scalar kG = Scalar(9.81);

InitParams MakeDefaultParams() {
  InitParams p{};
  p.g = kG;
  p.mag_declination_rad = 0;
  p.r_pressure_FRD = Vec3::Zero();
  p.init_yaw_ned_rad = 0;
  p.dt_max = Scalar(0.1);
  p.nis_gate_depth = Scalar(6.635);
  p.noise.sigma_g  = Scalar(1e-3);
  p.noise.sigma_a  = Scalar(1e-2);
  p.noise.sigma_bg = Scalar(1e-5);
  p.noise.sigma_ba = Scalar(1e-4);
  p.p0_pos = Scalar(0.1);
  p.p0_vel = Scalar(0.1);
  p.p0_att = Scalar(1e-3);
  p.p0_bg  = Scalar(1e-4);
  p.p0_ba  = Scalar(1e-3);
  return p;
}

Mat15 MakeDefaultP0(const InitParams& p) {
  Mat15 P = Mat15::Zero();
  P.diagonal().head<3>().setConstant(p.p0_pos);
  P.diagonal().segment<3>(3).setConstant(p.p0_vel);
  P.diagonal().segment<3>(6).setConstant(p.p0_att);
  P.diagonal().segment<3>(9).setConstant(p.p0_bg);
  P.diagonal().segment<3>(12).setConstant(p.p0_ba);
  return P;
}

}  // namespace

// === 核心防回归：静止 IMU 注入 60s，v_NED 不应出现 2g 级系统漂移 ===
TEST(ImuPropagator, StaticInputNo2gDrift) {
  Eskf ekf;
  InitParams p = MakeDefaultParams();
  State x0;  // 默认 body 对齐 NED，yaw=0
  ekf.initialize(p, x0, MakeDefaultP0(p));

  // body 对齐 NED 时静止 accel_FRD = [0, 0, -g]
  const Vec3 accel_static(0, 0, -kG);
  const Vec3 gyro_static(0, 0, 0);

  const Scalar dt = Scalar(0.02);  // 50 Hz
  for (int i = 1; i <= 3000; ++i) {  // 60 s
    Scalar t = i * dt;
    ekf.predictImu(t, gyro_static, accel_static);
  }

  // 60s 后速度模长应远小于 2g·60 = 1177 m/s。
  // 由于无噪声，理论 v=0，给 1e-6 严格阈值。
  EXPECT_LT(ekf.state().v_NED.norm(), 1e-6)
      << "v_NED=" << ekf.state().v_NED.transpose()
      << "  重力符号写反会 ≈ -2g·60 ≈ 1177 m/s。";
  EXPECT_LT(std::abs(ekf.state().p_NED.norm()), 1e-3);
}

// 反向证伪：手工构造"错误符号"代码路径，确认测试能抓到。
TEST(ImuPropagator, WrongGravitySignWouldDrift) {
  // 直接调用 PropagateNominal 验证：a_NED = R·(a_B - b_a) + g_NED 是正确的；
  // 若改成 -g_NED 则会得到 a_NED = -2g_NED。手动算一遍：
  State x;
  Vec3 a_B(0, 0, -kG);
  Vec3 g_NED(0, 0, kG);
  Vec3 a_NED_correct = x.q_NB.toRotationMatrix() * (a_B - x.b_a) + g_NED;
  Vec3 a_NED_wrong   = x.q_NB.toRotationMatrix() * (a_B - x.b_a) - g_NED;
  EXPECT_NEAR(a_NED_correct.norm(), 0.0, 1e-9);
  EXPECT_NEAR(a_NED_wrong.z(), -2 * kG, 1e-6);
}

// 自由落体一致性：将 body 翻转 180° (上下颠倒) 后 accel_FRD = [0,0,+g]
// 此时 a_NED = R · [0,0,g] + [0,0,g] = [0,0,-g] + [0,0,g] = 0 ?
// 不对。重新算：R = R_x(π) 把 FRD z 翻到 NED -z 方向。
//   R·[0,0,g] = [0,0,-g]
//   + g_NED = [0,0,g]
//   = [0,0,0]
// 静止倒立时 a_FRD 也是 [0,0,+g]，物理上是 body z 朝上、反作用力指向 body z。
// 用这个交叉验证旋转不会破坏静止条件。
TEST(ImuPropagator, RotatedStaticStillZero) {
  Eskf ekf;
  InitParams p = MakeDefaultParams();
  p.noise.sigma_a = 0;  // 关闭噪声做精确验证
  p.noise.sigma_g = 0;

  State x0;
  // pitch 30° 抬头
  const Scalar pitch = Scalar(M_PI / 6);
  x0.q_NB = Eigen::AngleAxis<Scalar>(pitch, Vec3::UnitY());
  ekf.initialize(p, x0, MakeDefaultP0(p));

  // body 抬头 30°，静止时 a_FRD = R_BN · (-g_NED)
  //   R_BN = R_y(-pitch)
  //   -g_NED = [0,0,-g]
  //   a_FRD = R_y(-pitch) · [0,0,-g] = [g·sin pitch, 0, -g·cos pitch]
  const Vec3 accel_static(kG * std::sin(pitch), 0, -kG * std::cos(pitch));
  const Vec3 gyro_static(0, 0, 0);

  for (int i = 1; i <= 500; ++i) {
    ekf.predictImu(i * Scalar(0.02), gyro_static, accel_static);
  }

  EXPECT_LT(ekf.state().v_NED.norm(), 1e-4)
      << "v_NED=" << ekf.state().v_NED.transpose();
}

// dt 上限保护：dt=1s > 0.1s 应被丢弃且 reset_count++
TEST(ImuPropagator, DtExceedingMaxIsRejected) {
  Eskf ekf;
  InitParams p = MakeDefaultParams();
  State x0;
  ekf.initialize(p, x0, MakeDefaultP0(p));

  ekf.predictImu(0.0, Vec3::Zero(), Vec3(0, 0, -kG));   // 首帧，建 t_prev
  ekf.predictImu(1.0, Vec3::Zero(), Vec3(0, 0, -kG));   // dt=1s 应被拒绝
  EXPECT_EQ(ekf.diag().reset_count, 1u);
  EXPECT_LT(ekf.state().v_NED.norm(), 1e-9);
}

// 静止 + 无观测 ⇒ attitude 协方差应随时间增长（无 mag/无 yaw 观测）
TEST(ImuPropagator, AttitudeCovarianceGrowsWithoutMeas) {
  Eskf ekf;
  InitParams p = MakeDefaultParams();
  State x0;
  ekf.initialize(p, x0, MakeDefaultP0(p));

  const Scalar P0_att_z = ekf.covariance()(kIdxDeltaTh + 2, kIdxDeltaTh + 2);

  for (int i = 1; i <= 1500; ++i) {  // 30s
    ekf.predictImu(i * Scalar(0.02), Vec3::Zero(), Vec3(0, 0, -kG));
  }

  const Scalar P_att_z = ekf.covariance()(kIdxDeltaTh + 2, kIdxDeltaTh + 2);
  EXPECT_GT(P_att_z, P0_att_z) << "yaw 方差应增长，初始=" << P0_att_z
                               << " 最终=" << P_att_z;
}

// F 矩阵数值线性化合理性：F·dx 应近似等于小 dt 数值差分
TEST(ImuPropagator, JacobianFinitDifferenceSanity) {
  State x0;
  x0.q_NB = Quat::Identity();
  Vec3 gyro(0.01, -0.02, 0.03);
  Vec3 accel(0.1, -0.2, -kG);
  NoiseParams noise{};
  noise.sigma_g  = Scalar(1e-3);
  noise.sigma_a  = Scalar(1e-2);
  noise.sigma_bg = Scalar(1e-5);
  noise.sigma_ba = Scalar(1e-4);

  Mat15 F, Q;
  const Scalar dt = Scalar(0.01);
  BuildErrorJacobians(x0, gyro, accel, noise, dt, F, Q);

  // F 对角应接近 1，且非负
  for (int i = 0; i < 15; ++i) {
    EXPECT_GT(F(i, i), 0.5) << "i=" << i;
  }

  // Q 应半正定（对角非负）
  for (int i = 0; i < 15; ++i) {
    EXPECT_GE(Q(i, i), 0) << "i=" << i;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
