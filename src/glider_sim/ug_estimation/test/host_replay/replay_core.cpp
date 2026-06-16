// replay_core.cpp
//
// 独立无 ROS 的 ug_ekf_core 回放器（host），fp64/fp32 双编译。
// 既用于 fp64↔fp32 数值对比，又是 STM32 wrapper 的 host 原型：
// 数据流与板载一致——只喂 raw FRD 量，core 自算 dt、做 align/predict/update。
//
// 编译:
//   fp64: g++ -std=c++17 -O2 -I <inc> -I /usr/include/eigen3 replay_core.cpp eskf.cpp -o replay64
//   fp32: 加 -DUG_EKF_USE_FP32
//
// 用法: ./replay <sensors.csv> [align_sec] > trace.csv
//   sensors.csv 含原始 body-FLU 量；本程序做 FLU→FRD（与 ROS wrapper 一致）。
//   stdout: t,roll,pitch,yaw,depth,bgx,bgy,bgz,bax,bay,baz

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <array>
#include "ug_estimation/core/eskf.h"

using namespace ug_ekf;

namespace {
constexpr double kPatm = 101325.0, kRho = 1028.0, kG = 9.81;

// FLU → FRD: diag(1,-1,-1)
inline Vec3 fluToFrd(double x, double y, double z) {
  return Vec3(Scalar(x), Scalar(-y), Scalar(-z));
}

InitParams makeParams() {
  InitParams p{};
  p.g = Scalar(kG); p.mag_declination_rad = Scalar(0);
  p.r_pressure_FRD = Vec3(Scalar(0), Scalar(0), Scalar(-0.1));
  p.init_yaw_ned_rad = Scalar(0);
  p.dt_max = Scalar(0.1);
  p.nis_gate_depth = Scalar(6.635);
  p.nis_gate_tilt  = Scalar(11.345);
  p.nis_gate_mag   = Scalar(6.635);
  p.noise.sigma_g=Scalar(1e-3); p.noise.sigma_a=Scalar(1e-2);
  p.noise.sigma_bg=Scalar(1e-5); p.noise.sigma_ba=Scalar(1e-4);
  p.p0_pos=Scalar(100); p.p0_vel=Scalar(0.01); p.p0_att=Scalar(0.01);
  p.p0_bg=Scalar(1e-4); p.p0_ba=Scalar(1e-3);
  return p;
}
Mat15 makeP0(const InitParams& p){
  Mat15 P = Mat15::Zero();
  P.diagonal().head<3>().setConstant(p.p0_pos);
  P.diagonal().segment<3>(3).setConstant(p.p0_vel);
  P.diagonal().segment<3>(6).setConstant(p.p0_att);
  P.diagonal().segment<3>(9).setConstant(p.p0_bg);
  P.diagonal().segment<3>(12).setConstant(p.p0_ba);
  return P;
}

// 固定容量对齐缓冲（嵌入式无堆；10s@50Hz=500，留 800 余量）
constexpr int kAlignCap = 800;
struct AlignBuf {
  std::array<Vec3,kAlignCap> acc, gyro, mag;
  int n_imu = 0, n_mag = 0;
  void pushImu(const Vec3& a, const Vec3& g){ if(n_imu<kAlignCap){acc[n_imu]=a;gyro[n_imu]=g;++n_imu;} }
  void pushMag(const Vec3& m){ if(n_mag<kAlignCap){mag[n_mag]=m;++n_mag;} }
};

// tilt 运动门控（与 ROS node 一致）
constexpr Scalar kTiltAcc = Scalar(0.5), kTiltGyro = Scalar(0.05), kTiltR = Scalar(0.25);
constexpr Scalar kDepthR = Scalar(0.01), kMagR = Scalar(0.01);
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr,"用法: %s sensors.csv [align_sec]\n",argv[0]); return 1; }
  const double align_sec = (argc>2)? std::atof(argv[2]) : 8.0;

  std::FILE* fp = std::fopen(argv[1],"r");
  if (!fp) { std::fprintf(stderr,"打不开 %s\n",argv[1]); return 1; }
  char line[256];
  std::fgets(line,sizeof(line),fp);  // header

  Eskf ekf; InitParams par = makeParams();
  ekf.initialize(par, State{}, makeP0(par));
  AlignBuf abuf;
  bool aligned = false;
  double t0 = -1;
  Vec3 last_gyro = Vec3::Zero();

  std::printf("t,roll,pitch,yaw,depth,bgx,bgy,bgz,bax,bay,baz\n");

  while (std::fgets(line,sizeof(line),fp)) {
    double t, v[6]; char kind;
    if (std::sscanf(line,"%lf,%c,%lf,%lf,%lf,%lf,%lf,%lf",
                    &t,&kind,&v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) < 3) continue;
    if (t0 < 0) t0 = t;

    if (!aligned) {
      if (kind=='I') abuf.pushImu(fluToFrd(v[3],v[4],v[5]), fluToFrd(v[0],v[1],v[2]));
      else if (kind=='M') abuf.pushMag(fluToFrd(v[0],v[1],v[2]));
      if (kind=='I' && (t-t0) >= align_sec && abuf.n_imu >= 10) {
        const Vec3* magp = (abuf.n_mag>=10)? abuf.mag.data() : nullptr;
        int n = abuf.n_imu;
        if (magp && abuf.n_mag < n) n = abuf.n_mag;
        ekf.staticAlign(abuf.acc.data(), abuf.gyro.data(), magp, std::size_t(n));
        aligned = true;
      }
      continue;
    }

    if (kind=='I') {
      Vec3 gyro = fluToFrd(v[0],v[1],v[2]);
      Vec3 acc  = fluToFrd(v[3],v[4],v[5]);
      ekf.predictImu(Scalar(t), gyro, acc);
      last_gyro = gyro;
      const bool quiet = std::abs(acc.norm()-Scalar(kG)) < kTiltAcc && gyro.norm() < kTiltGyro;
      if (quiet) ekf.updateAccelTilt(Scalar(t), acc, kTiltR);

      const State& x = ekf.state();
      Mat3 R = x.q_NB.toRotationMatrix();
      Scalar pitch = std::asin(std::max(Scalar(-1),std::min(Scalar(1),-R(2,0))));
      Scalar roll  = std::atan2(R(2,1),R(2,2));
      Scalar yaw   = std::atan2(R(1,0),R(0,0));
      std::printf("%.6f,%.8f,%.8f,%.8f,%.6f,%.8e,%.8e,%.8e,%.8e,%.8e,%.8e\n",
        t, double(roll),double(pitch),double(yaw), double(x.p_NED.z()),
        double(x.b_g.x()),double(x.b_g.y()),double(x.b_g.z()),
        double(x.b_a.x()),double(x.b_a.y()),double(x.b_a.z()));
    } else if (kind=='P') {
      Scalar depth = Scalar((v[0]-kPatm)/(kRho*kG));
      ekf.updateDepth(Scalar(t), depth, kDepthR);
    } else if (kind=='M') {
      ekf.updateMag(Scalar(t), fluToFrd(v[0],v[1],v[2]), kMagR);
    }
  }
  std::fclose(fp);
  return 0;
}
