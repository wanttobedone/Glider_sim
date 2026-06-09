// ug_estimation/core/measurements/accel_tilt.h
//
// 加速度计找平（重力方向）测量模型 + 雅可比。
// 平台无关；纯 Eigen；无堆分配；无 ROS。
//
// === 适用条件 ===
// 仅在"低线加速度"段使用（由 wrapper 门控：|‖a‖-g|<ε_a 且 |gyro|<ε_ω）。
// 此时加速度计测的比力 ≈ 仅含重力反作用，方向即体系下的 -g_NED 方向，
// 可观测 roll/pitch（绕重力的 yaw 不可观）。
//
// === 测量模型（FRD 体系 raw 比力）===
//   z = a_B = R_NB^T·(-g_NED) + b_a + n
//   g_NED = [0,0,+g]，故 -g_NED = [0,0,-g]
//   h(x) = R_NB^T·(-g_NED) + b_a
//
// === 雅可比（误差态约定 q_true = q_nom ⊗ Exp(δθ)）===
//   R_NB = R_nom·Exp(δθ) ⇒ R_NB^T ≈ (I-[δθ]_×)·R_nom^T
//   h ≈ h_nom + [h_grav]_×·δθ + δb_a,  h_grav = R_nom^T·(-g_NED)
//   ∂h/∂δθ   = [h_grav]_×   (3x3，秩 2，零空间沿重力 ⇒ 不观测 yaw)
//   ∂h/∂δb_a = I_3
//   其余块 = 0

#pragma once

#include "ug_estimation/core/types.h"
#include "ug_estimation/core/quaternion_utils.h"

namespace ug_ekf {

// 3x15 测量矩阵类型
using Mat3x15 = Eigen::Matrix<Scalar, 3, 15>;

// 期望 raw 比力（FRD）：重力方向 + accel 零偏
inline Vec3 PredictAccelSpecificForce(const State& x, Scalar g) {
  const Vec3 neg_g_NED(Scalar(0), Scalar(0), -g);          // -g_NED
  return x.q_NB.toRotationMatrix().transpose() * neg_g_NED + x.b_a;
}

// 构造 accel 找平的 H 矩阵 (3x15)
inline void BuildAccelTiltH(const State& x, Scalar g, Mat3x15& H_out) {
  H_out.setZero();
  const Vec3 neg_g_NED(Scalar(0), Scalar(0), -g);
  const Vec3 h_grav = x.q_NB.toRotationMatrix().transpose() * neg_g_NED;
  H_out.block<3,3>(0, kIdxDeltaTh) = Hat(h_grav);  // ∂h/∂δθ
  H_out.block<3,3>(0, kIdxDeltaBa) = Mat3::Identity();  // ∂h/∂δb_a
}

}  // namespace ug_ekf
