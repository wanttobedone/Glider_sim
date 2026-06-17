/* ug_estimation/embedded/ug_ekf_glider.h
 *
 * STM32 (及任意无 ROS 平台) 的 C 可调 ESKF wrapper 接口。
 * 纯 C 头：可被 C 固件 (FreeRTOS 任务) 直接 #include。实现为 C++ (ug_ekf_glider.cpp)，
 * 内部封装 ug_ekf::Eskf。无堆分配、无异常、无 iostream。
 *
 * 数据流（与仿真 ROS wrapper 等价，只喂 raw 量，core 自算 dt 做 align/predict/update）：
 *   板载 IMU_Core_GetRawData() → UgEkf_PushImu(gyro, accel, t_us)
 *   板载 mag (同 IMU 帧)       → UgEkf_PushMag(mag, t_us)
 *   板载 pressure_bar          → UgEkf_PushPressureBar(bar, t_us)
 *   周期读取                    → UgEkf_GetOutput(&out)
 *
 * 坐标系：core 内部 NED-FRD。板载 IMU 模块自身轴系经 cfg.mount_R 旋到 body-FRD：
 *   v_FRD = mount_R * v_module   (mount_R 行主序 3x3，由安装标定确定)
 * 单位：accel m/s^2，gyro rad/s，mag 任意(取方向)，pressure bar。
 *
 * 硬约束：不读取 IMU 模块的 orientation/quaternion/euler（C2）。
 */
#ifndef UG_EKF_GLIDER_H
#define UG_EKF_GLIDER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 配置，所有量纲见上。任意字段填 0 时用内部默认（见 .cpp）。 */
typedef struct {
  float gravity;            /* m/s^2，默认 9.81 */
  float p_atm_pa;           /* 大气压 Pa，默认 101325 */
  float water_rho;          /* 水密度 kg/m^3，默认 1028 */

  float mount_R[9];         /* IMU 模块帧 → body-FRD，行主序 3x3。全 0 视为单位阵 */
  float r_pressure_frd[3];  /* 压力计在 FRD 的杠杆臂 (m) */

  float static_align_seconds;   /* 静止对齐窗口 (s)，默认 8 */
  float init_yaw_ned_rad;       /* 无 mag 时的初始航向 (rad)，默认 0 */
  float mag_declination_rad;    /* 磁偏角 (rad)，默认 0 */

  /* 噪声密度与初始协方差；全 0 时用默认 */
  float sigma_g, sigma_a, sigma_bg, sigma_ba;
  float p0_pos, p0_vel, p0_att, p0_bg, p0_ba;

  /* 量测噪声与门控；全 0 时用默认 */
  float depth_var, tilt_var, mag_yaw_var;
  float nis_gate_depth, nis_gate_tilt, nis_gate_mag;
  float tilt_gate_acc, tilt_gate_gyro;
} UgEkfConfig;

/* 输出状态 (NED-FRD 约定) */
typedef struct {
  float roll, pitch, yaw;   /* rad */
  float depth_m;            /* 正=水下 */
  float vN, vE, vD;         /* NED 速度 m/s */
  float b_g[3], b_a[3];     /* 在线估计的零偏 (FRD) */

  float nis_depth, nis_tilt, nis_mag;
  uint32_t accept_depth, reject_depth;
  uint32_t accept_tilt,  reject_tilt;
  uint32_t accept_mag,   reject_mag;
  uint32_t reset_count;
  uint8_t  aligned;         /* 1=已完成静态对齐进入 RUNNING */
} UgEkfOutput;

/* 用默认值填充 cfg（调用方可随后覆盖个别字段）。 */
void UgEkf_DefaultConfig(UgEkfConfig* cfg);

/* 初始化/复位滤波器。cfg 为 NULL 时用全默认。 */
void UgEkf_Init(const UgEkfConfig* cfg);

/* 喂 IMU（模块帧 raw 量）。t_us = 设备时间戳 (微秒)。 */
void UgEkf_PushImu(float gx, float gy, float gz,
                   float ax, float ay, float az, uint64_t t_us);

/* 喂磁力计（模块帧 raw 量，单位任意）。 */
void UgEkf_PushMag(float mx, float my, float mz, uint64_t t_us);

/* 喂压力（bar，绝对压）。内部换算 depth=(bar*1e5 - p_atm)/(rho*g)。 */
void UgEkf_PushPressureBar(float pressure_bar, uint64_t t_us);

/* 直接喂深度 (m)，若上层已自行换算。 */
void UgEkf_PushDepth(float depth_m, uint64_t t_us);

/* 读取当前状态。 */
void UgEkf_GetOutput(UgEkfOutput* out);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* UG_EKF_GLIDER_H */
