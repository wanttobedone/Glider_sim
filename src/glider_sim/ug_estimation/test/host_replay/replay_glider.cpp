// replay_glider.cpp
//
// 用 STM32 C wrapper (ug_ekf_glider.h) 在 host 上回放 sensors.csv。
// 验证"将在 MCU 上运行的同一份 wrapper+core 代码"的数值正确性。fp64/fp32 双编译。
//
// CSV 存原始 body-FLU 量（Gazebo 仿真 IMU 帧）；这里 mount_R = diag(1,-1,-1)
// 复现 ROS wrapper 的 FLU→FRD。真实板载 IMU 模块帧不同，标定后改 mount_R 即可。
//
// 用法: ./replay_glider <sensors.csv> [align_sec] > trace.csv

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include "ug_estimation/embedded/ug_ekf_glider.h"

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "用法: %s sensors.csv [align_sec]\n", argv[0]); return 1; }
  const float align_sec = (argc > 2) ? float(std::atof(argv[2])) : 8.0f;

  UgEkfConfig cfg;
  UgEkf_DefaultConfig(&cfg);
  cfg.static_align_seconds = align_sec;
  // Gazebo FLU → FRD: diag(1,-1,-1)
  cfg.mount_R[0] = 1;  cfg.mount_R[4] = -1;  cfg.mount_R[8] = -1;
  UgEkf_Init(&cfg);

  std::FILE* fp = std::fopen(argv[1], "r");
  if (!fp) { std::fprintf(stderr, "打不开 %s\n", argv[1]); return 1; }
  char line[256];
  if (!std::fgets(line, sizeof(line), fp)) { std::fclose(fp); return 1; }  // header

  std::printf("t,roll,pitch,yaw,depth,bgx,bgy,bgz,bax,bay,baz\n");
  UgEkfOutput out;

  while (std::fgets(line, sizeof(line), fp)) {
    double t, v[6]; char kind;
    if (std::sscanf(line, "%lf,%c,%lf,%lf,%lf,%lf,%lf,%lf",
                    &t, &kind, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) < 3) continue;
    const uint64_t t_us = uint64_t(t * 1e6);

    if (kind == 'I') {
      UgEkf_PushImu(float(v[0]), float(v[1]), float(v[2]),
                    float(v[3]), float(v[4]), float(v[5]), t_us);
      UgEkf_GetOutput(&out);
      if (out.aligned) {
        std::printf("%.6f,%.8f,%.8f,%.8f,%.6f,%.8e,%.8e,%.8e,%.8e,%.8e,%.8e\n",
          t, out.roll, out.pitch, out.yaw, out.depth_m,
          out.b_g[0], out.b_g[1], out.b_g[2], out.b_a[0], out.b_a[1], out.b_a[2]);
      }
    } else if (kind == 'P') {
      // sensors.csv 存的是 Pa（仿真 FluidPressure），换算 bar 喂入
      UgEkf_PushPressureBar(float(v[0] / 1e5), t_us);
    } else if (kind == 'M') {
      UgEkf_PushMag(float(v[0]), float(v[1]), float(v[2]), t_us);
    }
  }
  std::fclose(fp);
  return 0;
}
