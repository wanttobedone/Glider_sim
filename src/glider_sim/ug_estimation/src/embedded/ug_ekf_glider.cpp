// ug_estimation/embedded/ug_ekf_glider.cpp
//
// ug_ekf_glider.h 的实现：C++ 封装 ug_ekf::Eskf，导出 C 接口。
// 无堆分配（静态实例 + 固定对齐缓冲）、无异常、无 iostream。

#include "ug_estimation/embedded/ug_ekf_glider.h"
#include "ug_estimation/core/eskf.h"

#include <cmath>
#include <cstring>

using namespace ug_ekf;

namespace {

// ---- 静态状态（零堆分配）----
Eskf       g_ekf;
UgEkfConfig g_cfg;
bool       g_initialized = false;
bool       g_aligned     = false;
double     g_t0_s        = -1.0;
Vec3       g_last_gyro_frd = Vec3::Zero();
Mat3       g_mountR        = Mat3::Identity();

// 增量对齐：累加和 + 计数（O(1) RAM，任意采样率/时长不溢出）
Vec3 g_acc_sum = Vec3::Zero(), g_gyro_sum = Vec3::Zero(), g_mag_sum = Vec3::Zero();
int  g_n_imu = 0, g_n_mag = 0;

inline double us_to_s(uint64_t t_us) { return double(t_us) * 1e-6; }

inline Vec3 toFrd(float x, float y, float z) {
  return g_mountR * Vec3(Scalar(x), Scalar(y), Scalar(z));
}

// 取默认或配置值（cfg 字段为 0 时回退默认）
inline Scalar pick(float v, double def) {
  return Scalar(v != 0.0f ? double(v) : def);
}

InitParams buildParams() {
  InitParams p{};
  p.g = pick(g_cfg.gravity, 9.81);
  p.mag_declination_rad = Scalar(g_cfg.mag_declination_rad);   // 0 合法
  p.init_yaw_ned_rad    = Scalar(g_cfg.init_yaw_ned_rad);      // 0 合法
  p.r_pressure_FRD = Vec3(Scalar(g_cfg.r_pressure_frd[0]),
                          Scalar(g_cfg.r_pressure_frd[1]),
                          Scalar(g_cfg.r_pressure_frd[2]));
  p.dt_max = Scalar(0.1);
  p.nis_gate_depth = pick(g_cfg.nis_gate_depth, 6.635);
  p.nis_gate_tilt  = pick(g_cfg.nis_gate_tilt, 11.345);
  p.nis_gate_mag   = pick(g_cfg.nis_gate_mag, 6.635);
  p.noise.sigma_g  = pick(g_cfg.sigma_g, 1e-3);
  p.noise.sigma_a  = pick(g_cfg.sigma_a, 1e-2);
  p.noise.sigma_bg = pick(g_cfg.sigma_bg, 1e-5);
  p.noise.sigma_ba = pick(g_cfg.sigma_ba, 1e-4);
  p.p0_pos = pick(g_cfg.p0_pos, 100.0);
  p.p0_vel = pick(g_cfg.p0_vel, 0.01);
  p.p0_att = pick(g_cfg.p0_att, 0.01);
  p.p0_bg  = pick(g_cfg.p0_bg, 1e-4);
  p.p0_ba  = pick(g_cfg.p0_ba, 1e-3);
  return p;
}

Mat15 buildP0(const InitParams& p) {
  Mat15 P = Mat15::Zero();
  P.diagonal().head<3>().setConstant(p.p0_pos);
  P.diagonal().segment<3>(3).setConstant(p.p0_vel);
  P.diagonal().segment<3>(6).setConstant(p.p0_att);
  P.diagonal().segment<3>(9).setConstant(p.p0_bg);
  P.diagonal().segment<3>(12).setConstant(p.p0_ba);
  return P;
}

void finishAlign() {
  InitParams p = buildParams();
  g_ekf.initialize(p, State{}, buildP0(p));
  if (g_n_imu >= 10) {
    const Vec3 acc_mean  = g_acc_sum  / Scalar(g_n_imu);
    const Vec3 gyro_mean = g_gyro_sum / Scalar(g_n_imu);
    const Vec3 mag_mean  = (g_n_mag >= 10) ? Vec3(g_mag_sum / Scalar(g_n_mag)) : Vec3::Zero();
    g_ekf.staticAlignFromMeans(acc_mean, gyro_mean,
                               (g_n_mag >= 10) ? &mag_mean : nullptr);
  }
  g_aligned = true;
}

}  // namespace

extern "C" {

void UgEkf_DefaultConfig(UgEkfConfig* cfg) {
  if (!cfg) return;
  std::memset(cfg, 0, sizeof(*cfg));
  cfg->gravity = 9.81f;
  cfg->p_atm_pa = 101325.0f;
  cfg->water_rho = 1028.0f;
  cfg->mount_R[0] = 1; cfg->mount_R[4] = 1; cfg->mount_R[8] = 1;  // 单位阵
  cfg->r_pressure_frd[2] = -0.1f;
  cfg->static_align_seconds = 8.0f;
  cfg->depth_var = 0.01f;
  cfg->tilt_var = 0.25f;
  cfg->mag_yaw_var = 0.01f;
  cfg->tilt_gate_acc = 0.5f;
  cfg->tilt_gate_gyro = 0.05f;
}

void UgEkf_Init(const UgEkfConfig* cfg) {
  if (cfg) g_cfg = *cfg;
  else UgEkf_DefaultConfig(&g_cfg);

  // mount_R 全 0 → 单位阵
  bool allzero = true;
  for (int i = 0; i < 9; ++i) if (g_cfg.mount_R[i] != 0.0f) { allzero = false; break; }
  if (allzero) g_mountR = Mat3::Identity();
  else {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        g_mountR(r, c) = Scalar(g_cfg.mount_R[r * 3 + c]);
  }

  g_n_imu = 0; g_n_mag = 0;
  g_aligned = false; g_t0_s = -1.0;
  g_last_gyro_frd = Vec3::Zero();

  InitParams p = buildParams();
  g_ekf.initialize(p, State{}, buildP0(p));
  g_initialized = true;
}

void UgEkf_PushImu(float gx, float gy, float gz,
                   float ax, float ay, float az, uint64_t t_us) {
  if (!g_initialized) return;
  const double t = us_to_s(t_us);
  const Vec3 gyro = toFrd(gx, gy, gz);
  const Vec3 acc  = toFrd(ax, ay, az);

  if (!g_aligned) {
    if (g_t0_s < 0) g_t0_s = t;
    g_acc_sum += acc; g_gyro_sum += gyro; ++g_n_imu;
    if ((t - g_t0_s) >= double(g_cfg.static_align_seconds) && g_n_imu >= 10) finishAlign();
    return;
  }

  g_ekf.predictImu(Scalar(t), gyro, acc);
  g_last_gyro_frd = gyro;

  const Scalar g = pick(g_cfg.gravity, 9.81);
  const Scalar gate_a = pick(g_cfg.tilt_gate_acc, 0.5);
  const Scalar gate_w = pick(g_cfg.tilt_gate_gyro, 0.05);
  if (std::abs(acc.norm() - g) < gate_a && gyro.norm() < gate_w) {
    g_ekf.updateAccelTilt(Scalar(t), acc, pick(g_cfg.tilt_var, 0.25));
  }
}

void UgEkf_PushMag(float mx, float my, float mz, uint64_t t_us) {
  if (!g_initialized) return;
  const Vec3 mag = toFrd(mx, my, mz);
  if (!g_aligned) {
    g_mag_sum += mag; ++g_n_mag;
    return;
  }
  g_ekf.updateMag(Scalar(us_to_s(t_us)), mag, pick(g_cfg.mag_yaw_var, 0.01));
}

void UgEkf_PushDepth(float depth_m, uint64_t t_us) {
  if (!g_initialized || !g_aligned) return;
  g_ekf.updateDepth(Scalar(us_to_s(t_us)), Scalar(depth_m), pick(g_cfg.depth_var, 0.01));
}

void UgEkf_PushPressureBar(float pressure_bar, uint64_t t_us) {
  const Scalar p_atm = pick(g_cfg.p_atm_pa, 101325.0);
  const Scalar rho   = pick(g_cfg.water_rho, 1028.0);
  const Scalar g     = pick(g_cfg.gravity, 9.81);
  const Scalar depth = (Scalar(pressure_bar) * Scalar(1e5) - p_atm) / (rho * g);
  UgEkf_PushDepth(float(depth), t_us);
}

void UgEkf_GetOutput(UgEkfOutput* out) {
  if (!out) return;
  std::memset(out, 0, sizeof(*out));
  const State& x = g_ekf.state();
  const Mat3 R = x.q_NB.toRotationMatrix();
  out->pitch = float(std::asin(std::max(Scalar(-1), std::min(Scalar(1), -R(2, 0)))));
  out->roll  = float(std::atan2(R(2, 1), R(2, 2)));
  out->yaw   = float(std::atan2(R(1, 0), R(0, 0)));
  out->depth_m = float(x.p_NED.z());
  out->vN = float(x.v_NED.x()); out->vE = float(x.v_NED.y()); out->vD = float(x.v_NED.z());
  for (int i = 0; i < 3; ++i) { out->b_g[i] = float(x.b_g[i]); out->b_a[i] = float(x.b_a[i]); }
  const Diagnostics& d = g_ekf.diag();
  out->nis_depth = float(d.last_nis_depth);
  out->nis_tilt  = float(d.last_nis_tilt);
  out->nis_mag   = float(d.last_nis_mag);
  out->accept_depth = d.accept_depth; out->reject_depth = d.reject_depth;
  out->accept_tilt  = d.accept_tilt;  out->reject_tilt  = d.reject_tilt;
  out->accept_mag   = d.accept_mag;   out->reject_mag   = d.reject_mag;
  out->reset_count  = d.reset_count;
  out->aligned = g_aligned ? 1 : 0;
}

}  // extern "C"
