// ug_estimation/src/core/eskf.cpp
//
// Phase 0：仅落空骨架，让库可链接 + 单测可运行。Phase 1 填实现。

#include "ug_estimation/core/eskf.h"
#include "ug_estimation/core/quaternion_utils.h"

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

void Eskf::predictImu(Scalar /*t*/, const Vec3& /*gyro_FRD*/, const Vec3& /*accel_FRD*/) {
  // TODO Phase 1: 名义传播 + ESKF F/Q
}

bool Eskf::updateDepth(Scalar /*t*/, Scalar /*depth_m*/, Scalar /*R*/) {
  // TODO Phase 1
  return false;
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

void Eskf::staticAlign(const Vec3* /*acc*/, const Vec3* /*gyro*/,
                       const Vec3* /*mag*/, std::size_t /*n*/) {
  // TODO Phase 2
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
