// ug_estimation/core/measurements/mag.h
//
// 磁力计偏航（yaw-only）测量模型 + 雅可比。
// 平台无关；纯 Eigen；无堆分配；无 ROS。
//
//   设计目标  
// 仅约束 yaw（绕 NED-Down 的航向），不触碰 roll/pitch（那由 accel 找平负责）。
// core 收到的是已做 FLU→FRD + 硬铁/软铁补偿的磁场向量 mag_FRD（单位任意，取方向）。
//
//   测量模型（标量 yaw 误差） 
// 用当前姿态把测量磁场旋到世界系：m_world = R_NB · mag_FRD。
// 若 yaw 估计正确，m_world 的水平分量应指向磁北 + 磁偏角 decl：
//     atan2(m_world.E, m_world.N) == decl
// 否则其偏差即 yaw 误差：
//     y(innovation) = wrap( decl - atan2(m_world.E, m_world.N) )
//
//   雅可比  
// 世界系 yaw 误差 δψ 与体系误差态 δθ 关系：δψ = e_D^T · R_NB · δθ
//   ∂y/∂δθ = e_D^T · R_NB = R_NB 的第 3 行   (1x3，仅观测 Down 轴旋转 = yaw)
// 其余块 = 0。roll/pitch（与 Down 正交的旋转）不被观测，故不破坏 tilt 约束。

#pragma once

#include "ug_estimation/core/types.h"
#include "ug_estimation/core/quaternion_utils.h"
#include <cmath>

namespace ug_ekf {

// 把角度 wrap 到 [-pi, pi]
inline Scalar WrapPi(Scalar a) {
  while (a >  Scalar(M_PI)) a -= Scalar(2 * M_PI);
  while (a < -Scalar(M_PI)) a += Scalar(2 * M_PI);
  return a;
}

// 计算 mag yaw 的 innovation（标量，rad）。
//   decl = 磁偏角 (rad)
inline Scalar MagYawInnovation(const State& x, const Vec3& mag_FRD, Scalar decl) {
  const Vec3 m_world = x.q_NB.toRotationMatrix() * mag_FRD;
  const Scalar field_heading = std::atan2(m_world.y(), m_world.x());  // atan2(E, N)
  return WrapPi(decl - field_heading);
}

// 构造 mag yaw 的 H 行向量 (1x15)。
inline void BuildMagYawH(const State& x, RowVec15& H_out) {
  H_out.setZero();
  const Mat3 R_NB = x.q_NB.toRotationMatrix();
  // e_D^T · R_NB = R_NB 第 3 行
  H_out(0, kIdxDeltaTh + 0) = R_NB(2, 0);
  H_out(0, kIdxDeltaTh + 1) = R_NB(2, 1);
  H_out(0, kIdxDeltaTh + 2) = R_NB(2, 2);
}

// 从静止水平段磁场恢复初始 yaw（level 假设 roll=pitch=0）。
//   level 下 mag_FRD 水平分量 (mx,my) 满足 mx=Bh·cos(ψ-decl), my=-Bh·sin(ψ-decl)
//   ⇒ ψ = decl + atan2(-my, mx)
inline Scalar MagInitYaw(const Vec3& mag_FRD_mean, Scalar decl) {
  return WrapPi(decl + std::atan2(-mag_FRD_mean.y(), mag_FRD_mean.x()));
}

}  // namespace ug_ekf
