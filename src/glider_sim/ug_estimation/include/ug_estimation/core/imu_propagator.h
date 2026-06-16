// ug_estimation/core/imu_propagator.h
//
// ESKF IMU 名义传播 + 误差状态离散 F/Q 矩阵。
// 平台无关；Eigen 定长；无堆分配；无 ROS；无异常。
//
//   重力符号约定（禁止修改）  
// NED 下 g_NED = [0,0,+g]^T。加速度计 (FRD 系) 测的是 specific force，
// 静止 + body 对齐 NED 时输出 a_B ≈ [0,0,-g]^T （指向 -Down = Up）。
// 因此 v_NED 传播：
//     a_NED   = R_NB * (a_B - b_a)            (含重力的世界系比力)
//     dv/dt   = a_NED + g_NED                 (加号！)
//
// 反向核对：静止时 a_NED = R_NB * [0,0,-g] = [0,0,-g]；
//          dv/dt = [0,0,-g] + [0,0,+g] = 0 ✓
//
// 若误写减号，静止时 dv/dt = -2g，速度会以 ≈ -2g 系统漂移；
// test_imu_propagator.cpp 必须覆盖此场景。
//
//   误差状态约定  
//   δx = [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]   维度 15
//   δθ 表示 q_true = q_nom ⊗ ExpQ(δθ)，注入时 InjectDeltaTheta
//
//   离散 F 矩阵（一阶 Euler，足够 50Hz IMU）  
//   I3 = 3x3 单位阵; R = R_NB(q_nom); â = a_B - b_a; ω̂ = ω_B - b_g
//   分块（行 = 状态导数，列 = 误差源）:
//     δp:   δp' = δp + δv·dt
//     δv:   δv' = δv - R·[â]_×·δθ·dt - R·δb_a·dt
//     δθ:   δθ' = (I - [ω̂]_×·dt)·δθ - δb_g·dt
//     δb_g: δb_g' = δb_g
//     δb_a: δb_a' = δb_a
//
//   Q 矩阵（连续噪声密度离散化，对角块）  
//   Q_v (=R·diag(σ_a²)·R^T · dt)
//   Q_θ (= diag(σ_g²) · dt)
//   Q_bg(= diag(σ_bg²)· dt)
//   Q_ba(= diag(σ_ba²)· dt)

#pragma once

#include "ug_estimation/core/types.h"
#include "ug_estimation/core/quaternion_utils.h"

namespace ug_ekf {

// 名义状态传播（一阶 Euler）。
// 输入：
//   x      — 当前名义状态（in/out）
//   gyro_B — body-FRD 角速度 (rad/s)
//   acc_B  — body-FRD 比力含重力 (m/s^2)
//   g_mag  — 重力加速度模（NED 下 g_NED = [0,0,+g_mag]）
//   dt     — 步长 (s)，调用方保证 0 < dt ≤ dt_max
inline void PropagateNominal(State& x,
                             const Vec3& gyro_B,
                             const Vec3& acc_B,
                             Scalar g_mag,
                             Scalar dt) {
  const Vec3 omega = gyro_B - x.b_g;
  const Vec3 accel = acc_B  - x.b_a;

  const Mat3 R_NB = x.q_NB.toRotationMatrix();
  const Vec3 g_NED(Scalar(0), Scalar(0), g_mag);

  // 注意符号: dv/dt = R·a_B + g_NED
  const Vec3 a_NED = R_NB * accel + g_NED;

  x.p_NED += x.v_NED * dt + Scalar(0.5) * a_NED * dt * dt;
  x.v_NED += a_NED * dt;
  x.q_NB   = InjectDeltaTheta(x.q_NB, omega * dt);  // q_{k+1} = q_k ⊗ Exp(ω·dt)
  // b_g, b_a 随机游走 — 名义传播不变
}

// 构造离散 F 矩阵（15x15）和 Q 矩阵（15x15）。
// 输入
//   x_pre  — 传播前名义状态（用其 q、b_g、b_a 计算雅可比线性化点）
//   gyro_B, acc_B — 原始 IMU 输入（未减 bias）
//   noise  — 连续时间噪声密度
//   dt     — 步长 (s)
//   F_out, Q_out — 输出（in/out by reference）
inline void BuildErrorJacobians(const State& x_pre,
                                const Vec3& gyro_B,
                                const Vec3& acc_B,
                                const NoiseParams& noise,
                                Scalar dt,
                                Mat15& F_out,
                                Mat15& Q_out) {
  const Vec3 omega_hat = gyro_B - x_pre.b_g;
  const Vec3 accel_hat = acc_B  - x_pre.b_a;
  const Mat3 R_NB      = x_pre.q_NB.toRotationMatrix();

  F_out.setIdentity();

  const Mat3 I3 = Mat3::Identity();

  // δp 行: ∂δp'/∂δv = I·dt
  F_out.block<3,3>(kIdxDeltaP, kIdxDeltaV) = I3 * dt;

  // δv 行:
  //   ∂δv'/∂δθ  = -R·[â]_×·dt
  //   ∂δv'/∂δb_a = -R·dt
  F_out.block<3,3>(kIdxDeltaV, kIdxDeltaTh) = -R_NB * Hat(accel_hat) * dt;
  F_out.block<3,3>(kIdxDeltaV, kIdxDeltaBa) = -R_NB * dt;

  // δθ 行:
  //   ∂δθ'/∂δθ   = I - [ω̂]_×·dt
  //   ∂δθ'/∂δb_g = -I·dt
  F_out.block<3,3>(kIdxDeltaTh, kIdxDeltaTh) = I3 - Hat(omega_hat) * dt;
  F_out.block<3,3>(kIdxDeltaTh, kIdxDeltaBg) = -I3 * dt;

  // 离散过程噪声 Q（对角块；连续密度 × dt）
  Q_out.setZero();
  const Scalar sg2  = noise.sigma_g  * noise.sigma_g;
  const Scalar sa2  = noise.sigma_a  * noise.sigma_a;
  const Scalar sbg2 = noise.sigma_bg * noise.sigma_bg;
  const Scalar sba2 = noise.sigma_ba * noise.sigma_ba;

  // δv 噪声: R·Q_a·R^T·dt
  Q_out.block<3,3>(kIdxDeltaV,  kIdxDeltaV)  = R_NB * (sa2 * dt) * R_NB.transpose();
  Q_out.block<3,3>(kIdxDeltaTh, kIdxDeltaTh) = I3 * (sg2  * dt);
  Q_out.block<3,3>(kIdxDeltaBg, kIdxDeltaBg) = I3 * (sbg2 * dt);
  Q_out.block<3,3>(kIdxDeltaBa, kIdxDeltaBa) = I3 * (sba2 * dt);
}

}  // namespace ug_ekf
