// ug_estimation/core/measurements/depth.h
//
// 深度测量（压力计 → 标量 depth_m）测量模型 + 雅可比，含杠杆臂。
// 平台无关；纯 Eigen；无堆分配；无 ROS。
//
// === 测量模型 ===
//   z = e_D^T (p_NED + R_NB · r_pressure_FRD) + n,   n ~ N(0, R)
//   e_D = [0, 0, 1]^T  (NED 第三轴, Down 为正)
//
// === 雅可比 ===
//   ∂z/∂δp   = e_D^T                              (1x3)
//   ∂z/∂δθ   = -e_D^T · R_NB · [r_pressure_FRD]_× (1x3)
//             推导：δ(R·r) = -R·[r]_× δθ
//   其余分块为 0。

#pragma once

#include "ug_estimation/core/types.h"
#include "ug_estimation/core/quaternion_utils.h"

namespace ug_ekf {

// 行向量类型 (1x15)，与误差状态对齐
using RowVec15 = Eigen::Matrix<Scalar, 1, 15>;

// 计算预测深度 z_pred = e_D^T · (p_NED + R_NB · r_p)
inline Scalar PredictDepth(const State& x, const Vec3& r_pressure_FRD) {
  const Vec3 p_sensor_NED = x.p_NED + x.q_NB.toRotationMatrix() * r_pressure_FRD;
  return p_sensor_NED.z();
}

// 构造深度更新的 H 行向量。
inline void BuildDepthH(const State& x,
                        const Vec3& r_pressure_FRD,
                        RowVec15& H_out) {
  H_out.setZero();
  // ∂z/∂δp = e_D^T = [0,0,1]
  H_out(0, kIdxDeltaP + 2) = Scalar(1);

  // ∂z/∂δθ = -e_D^T · R_NB · [r_p]_×
  const Mat3 R_NB = x.q_NB.toRotationMatrix();
  const Mat3 R_hat_r = R_NB * Hat(r_pressure_FRD);
  // 取 R_hat_r 的第 3 行（对应 e_D^T 投影），并取反
  H_out(0, kIdxDeltaTh + 0) = -R_hat_r(2, 0);
  H_out(0, kIdxDeltaTh + 1) = -R_hat_r(2, 1);
  H_out(0, kIdxDeltaTh + 2) = -R_hat_r(2, 2);
}

}  // namespace ug_ekf
