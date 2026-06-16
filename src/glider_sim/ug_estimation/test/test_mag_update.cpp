// test_mag_update.cpp
//
// 验证磁力计 yaw 更新：
//   1) updateMag 只改 yaw，不动 roll/pitch（H 只观测 Down 轴）
//   2) 注入 z 陀螺零偏致 yaw 漂移；mag update 后 yaw 有界 + b_g.z 收敛
//   3) staticAlign 用 mag 恢复已知 yaw
//   4) Joseph 后 P 对称

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/measurements/mag.h"

using namespace ug_ekf;

namespace {
constexpr Scalar kG = Scalar(9.81);
constexpr Scalar kBh = Scalar(3.54e-5);   // 水平场强
constexpr Scalar kBd = Scalar(3.54e-5);   // 下向场强 (incl 45°)

// NED 参考磁场（decl=0）：北 + 下
Vec3 magNED() { return Vec3(kBh, 0, kBd); }

// 给定真实 yaw，生成 level body-FRD 磁场 = R_NB(yaw)^T · m_NED
Vec3 magFrdAtYaw(Scalar yaw) {
  Quat q(Eigen::AngleAxis<Scalar>(yaw, Vec3::UnitZ()));
  return q.toRotationMatrix().transpose() * magNED();
}

InitParams MakeParams() {
  InitParams p{};
  p.g = kG; p.mag_declination_rad = 0; p.r_pressure_FRD = Vec3::Zero();
  p.init_yaw_ned_rad = 0;
  p.dt_max = Scalar(0.1);
  p.nis_gate_depth = Scalar(6.635);
  p.nis_gate_tilt  = Scalar(11.345);
  p.nis_gate_mag   = Scalar(6.635);
  p.noise.sigma_g=Scalar(1e-3); p.noise.sigma_a=Scalar(1e-2);
  p.noise.sigma_bg=Scalar(1e-4); p.noise.sigma_ba=Scalar(1e-3);
  p.p0_pos=1; p.p0_vel=0.1; p.p0_att=Scalar(1e-2);
  p.p0_bg=Scalar(1e-2); p.p0_ba=Scalar(1e-2);
  return p;
}
Mat15 MakeP0(const InitParams& p){
  Mat15 P=Mat15::Zero();
  P.diagonal().head<3>().setConstant(p.p0_pos);
  P.diagonal().segment<3>(3).setConstant(p.p0_vel);
  P.diagonal().segment<3>(6).setConstant(p.p0_att);
  P.diagonal().segment<3>(9).setConstant(p.p0_bg);
  P.diagonal().segment<3>(12).setConstant(p.p0_ba);
  return P;
}
void RPY(const Quat& q, Scalar& r, Scalar& p, Scalar& y){
  Mat3 R=q.toRotationMatrix();
  p=std::asin(std::max(Scalar(-1),std::min(Scalar(1),-R(2,0))));
  r=std::atan2(R(2,1),R(2,2));
  y=std::atan2(R(1,0),R(0,0));
}
}  // namespace

// H 只观测 Down 轴（yaw），roll/pitch 扰动不进入
TEST(MagUpdate, JacobianYawOnly) {
  State x;  // level yaw=0, R_NB=I
  RowVec15 H; BuildMagYawH(x, H);
  Vec3 Hth = H.block<1,3>(0, kIdxDeltaTh).transpose();
  // level 下 e_D^T R_NB = [0,0,1] → 只 z(yaw)
  EXPECT_NEAR(Hth.x(), 0.0, 1e-9);
  EXPECT_NEAR(Hth.y(), 0.0, 1e-9);
  EXPECT_NEAR(Hth.z(), 1.0, 1e-9);
}

// staticAlign 用 mag 恢复已知 yaw
TEST(MagUpdate, StaticAlignYawFromMag) {
  for (Scalar yaw_true : {Scalar(0), Scalar(0.5), Scalar(1.5708), Scalar(-2.0)}) {
    Eskf ekf; InitParams p=MakeParams(); State x0;
    ekf.initialize(p,x0,MakeP0(p));
    std::vector<Vec3> acc(50, Vec3(0,0,-kG)), gyro(50, Vec3::Zero());
    std::vector<Vec3> mag(50, magFrdAtYaw(yaw_true));
    ekf.staticAlign(acc.data(),gyro.data(),mag.data(),50);
    Scalar r,p2,y; RPY(ekf.state().q_NB,r,p2,y);
    EXPECT_NEAR(WrapPi(y-yaw_true),0.0,1e-4) << "yaw_true="<<yaw_true;
  }
}

// updateMag 把 yaw 拉向真值，roll/pitch 不变。
// 用门控内的小 yaw 误差（运行期真实工况：staticAlign 已把 yaw 初始化到位，
// 只需修小漂移）。大初始误差由 staticAlign 负责，不靠运行期 update。
TEST(MagUpdate, CorrectsYawNotRollPitch) {
  Eskf ekf; InitParams p=MakeParams(); State x0;
  // 初始 yaw=0，真实 yaw=0.1 (≈5.7°，门控内)
  ekf.initialize(p,x0,MakeP0(p));
  Scalar r0,p0,y0; RPY(ekf.state().q_NB,r0,p0,y0);

  const Scalar yaw_true = Scalar(0.1);
  Vec3 mag = magFrdAtYaw(yaw_true);
  for (int i=0;i<50;i++) ekf.updateMag(0, mag, Scalar(1e-2));

  Scalar r1,p1,y1; RPY(ekf.state().q_NB,r1,p1,y1);
  EXPECT_NEAR(y1, yaw_true, 0.01) << "yaw 应收敛到真值";
  EXPECT_NEAR(r1, r0, 1e-3) << "roll 不应变";
  EXPECT_NEAR(p1, p0, 1e-3) << "pitch 不应变";
  const Mat15& P=ekf.covariance();
  EXPECT_LT((P-P.transpose()).norm(),1e-9);
  EXPECT_GT(ekf.diag().accept_mag, 10u);
}

// NIS 门控：过大 yaw 误差应被拒（防野值/初始未对齐时污染）
TEST(MagUpdate, GatesLargeInnovation) {
  Eskf ekf; InitParams p=MakeParams(); State x0;
  ekf.initialize(p,x0,MakeP0(p));
  Vec3 mag = magFrdAtYaw(Scalar(0.5));  // 0.5rad vs σ≈0.1 → NIS 超门
  ekf.updateMag(0, mag, Scalar(1e-2));
  EXPECT_EQ(ekf.diag().accept_mag, 0u);
  EXPECT_EQ(ekf.diag().reject_mag, 1u);
}

// z 陀螺零偏致 yaw 漂；mag 更新后有界 + b_g.z 收敛
TEST(MagUpdate, BoundsYawDriftEstimatesBiasZ) {
  const Vec3 true_bg(0,0,0.01);  // z 陀螺零偏
  const Scalar dt=Scalar(0.02);
  Eskf ekf; InitParams p=MakeParams(); State x0;
  ekf.initialize(p,x0,MakeP0(p));
  // 对齐到 yaw=0
  std::vector<Vec3> acc(50,Vec3(0,0,-kG)),gyro(50,Vec3::Zero()),mag(50,magFrdAtYaw(0));
  ekf.staticAlign(acc.data(),gyro.data(),mag.data(),50);

  const Vec3 acc_meas(0,0,-kG);
  for (int i=1;i<=1500;i++){  // 30s 静止，真实 yaw 恒=0
    Scalar t=i*dt;
    ekf.predictImu(t, true_bg, acc_meas);     // 测量含 z 零偏
    ekf.updateMag(t, magFrdAtYaw(0), Scalar(1e-2));  // 真实 yaw=0
  }
  Scalar r,pp,y; RPY(ekf.state().q_NB,r,pp,y);
  EXPECT_LT(std::abs(y), Scalar(0.0175)) << "yaw 应被 mag 约束在 <1°, y="<<y;
  EXPECT_NEAR(ekf.state().b_g.z(), true_bg.z(), std::abs(true_bg.z())*0.4)
      << "b_g.z 应收敛, bgz="<<ekf.state().b_g.z();
}

int main(int argc,char** argv){
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
