/**
 * glider_ekf_node
 *
 * 自研 ESKF (ug_ekf_core) 的 ROS wrapper (Phase 1)。
 *
 * 订阅:
 *   {ns}/imu      [sensor_msgs/Imu]            50 Hz  — 仅用 ω、a，不读 orientation
 *   {ns}/pressure [sensor_msgs/FluidPressure]  10 Hz  — 换算为深度后做更新
 *
 * 发布:
 *   {ns}/odometry/filtered_eskf      [nav_msgs/Odometry]       30 Hz (ENU/FLU)
 *   {ns}/glider_ekf/diagnostics      [ug_msgs/EskfDiagnostics] 30 Hz
 *
 * 坐标系:
 *   core 内部 NED-FRD；输出转 ENU-FLU 供 state_converter_node 复用。
 *     R_WB'(FLU→ENU) = C · R_NB · D,  C=[[0,1,0],[1,0,0],[0,0,-1]], D=diag(1,-1,-1)
 *
 * 启动流程 (Phase 1):
 *   ALIGNING: 累积 static_align_seconds 秒静止样本 → staticAlign 初始化 roll/pitch/b_g
 *             静止样本不足则回退到 init_yaw_ned_rad + 零 bias。
 *   RUNNING : 正常 predict/update。
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <ug_msgs/EskfDiagnostics.h>

#include <vector>
#include <cmath>

#include "ug_estimation/core/eskf.h"
#include "ug_estimation/ros/sensor_adapter.h"

using ug_ekf::Scalar;
using ug_ekf::Vec3;
using ug_ekf::Mat3;
using ug_ekf::Mat15;
using ug_ekf::State;
using ug_ekf::InitParams;

class GliderEkfNode {
 public:
  GliderEkfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string ns = pnh.param<std::string>("namespace", "ug_glider");

    //   物理 / 噪声参数  
    pnh.param<double>("gravity", g_, 9.81);
    pnh.param<double>("atmospheric_pressure", p_atm_, 101325.0);
    pnh.param<double>("water_density", rho_, 1028.0);

    pnh.param<double>("init_yaw_ned_rad", init_yaw_ned_rad_, 0.0);
    pnh.param<double>("static_align_seconds", align_seconds_, 3.0);
    pnh.param<double>("dt_max", dt_max_, 0.1);
    pnh.param<double>("nis_gate_depth", nis_gate_depth_, 6.635);
    pnh.param<double>("depth_meas_variance", depth_R_, 0.01);

    // 压力计杠杆臂 (FRD)。base_link FLU 偏移 (0,0,0.1) → FRD (0,0,-0.1)
    std::vector<double> r_p;
    pnh.param<std::vector<double>>("r_pressure_frd", r_p, {0.0, 0.0, -0.1});
    r_pressure_frd_ = Vec3(static_cast<Scalar>(r_p[0]),
                           static_cast<Scalar>(r_p[1]),
                           static_cast<Scalar>(r_p[2]));

    // 噪声密度
    pnh.param<double>("sigma_g",  sigma_g_,  1e-3);
    pnh.param<double>("sigma_a",  sigma_a_,  1e-2);
    pnh.param<double>("sigma_bg", sigma_bg_, 1e-5);
    pnh.param<double>("sigma_ba", sigma_ba_, 1e-4);

    // 初始协方差对角
    pnh.param<double>("p0_pos", p0_pos_, 100.0);
    pnh.param<double>("p0_vel", p0_vel_, 0.01);
    pnh.param<double>("p0_att", p0_att_, 1e-2);
    pnh.param<double>("p0_bg",  p0_bg_,  1e-4);
    pnh.param<double>("p0_ba",  p0_ba_,  1e-3);

    // 静态对齐准入门槛
    pnh.param<double>("align_gyro_thresh", align_gyro_thresh_, 0.02);
    pnh.param<double>("align_acc_thresh",  align_acc_thresh_,  0.2);

    //   IO  
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odometry/filtered_eskf", 10);
    diag_pub_ = nh.advertise<ug_msgs::EskfDiagnostics>("glider_ekf/diagnostics", 10);
    imu_sub_  = nh.subscribe("imu", 100, &GliderEkfNode::onImu, this);
    pres_sub_ = nh.subscribe("pressure", 20, &GliderEkfNode::onPressure, this);

    odom_frame_ = pnh.param<std::string>("odom_frame", ns + "/odom");
    base_frame_ = pnh.param<std::string>("base_frame", ns + "/base_link");

    pub_timer_ = nh.createTimer(ros::Duration(1.0 / 30.0),
                                &GliderEkfNode::onPublishTimer, this);

    ROS_INFO("[glider_ekf] 启动: align=%.1fs init_yaw_ned=%.3f r_p_frd=[%.3f %.3f %.3f]",
             align_seconds_, init_yaw_ned_rad_,
             r_p[0], r_p[1], r_p[2]);
  }

 private:
  enum class Phase { ALIGNING, RUNNING };

  InitParams buildInitParams() const {
    InitParams p{};
    p.g = static_cast<Scalar>(g_);
    p.mag_declination_rad = 0;
    p.r_pressure_FRD = r_pressure_frd_;
    p.init_yaw_ned_rad = static_cast<Scalar>(init_yaw_ned_rad_);
    p.dt_max = static_cast<Scalar>(dt_max_);
    p.nis_gate_depth = static_cast<Scalar>(nis_gate_depth_);
    p.noise.sigma_g  = static_cast<Scalar>(sigma_g_);
    p.noise.sigma_a  = static_cast<Scalar>(sigma_a_);
    p.noise.sigma_bg = static_cast<Scalar>(sigma_bg_);
    p.noise.sigma_ba = static_cast<Scalar>(sigma_ba_);
    p.p0_pos = static_cast<Scalar>(p0_pos_);
    p.p0_vel = static_cast<Scalar>(p0_vel_);
    p.p0_att = static_cast<Scalar>(p0_att_);
    p.p0_bg  = static_cast<Scalar>(p0_bg_);
    p.p0_ba  = static_cast<Scalar>(p0_ba_);
    return p;
  }

  Mat15 buildP0(const InitParams& p) const {
    Mat15 P = Mat15::Zero();
    P.diagonal().head<3>().setConstant(p.p0_pos);
    P.diagonal().segment<3>(3).setConstant(p.p0_vel);
    P.diagonal().segment<3>(6).setConstant(p.p0_att);
    P.diagonal().segment<3>(9).setConstant(p.p0_bg);
    P.diagonal().segment<3>(12).setConstant(p.p0_ba);
    return P;
  }

  void finishAlignment() {
    InitParams p = buildInitParams();
    State x0;  // 位置/速度=0，yaw 由 staticAlign 用 init_yaw_ned_rad 设置
    ekf_.initialize(p, x0, buildP0(p));

    if (acc_buf_.size() >= 10) {
      ekf_.staticAlign(acc_buf_.data(), gyro_buf_.data(),
                       nullptr, acc_buf_.size());
      ROS_INFO("[glider_ekf] 静态对齐完成: 用 %zu 个静止样本, b_g=[%.4f %.4f %.4f]",
               acc_buf_.size(),
               ekf_.state().b_g.x(), ekf_.state().b_g.y(), ekf_.state().b_g.z());
    } else {
      ROS_WARN("[glider_ekf] 静止样本不足(%zu)，回退: yaw=%.3f 零 bias",
               acc_buf_.size(), init_yaw_ned_rad_);
    }
    acc_buf_.clear();
    gyro_buf_.clear();
    phase_ = Phase::RUNNING;
  }

  void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
    const Scalar t = static_cast<Scalar>(msg->header.stamp.toSec());
    const Vec3 gyro_FRD  = ug_ekf::ros_adapter::GyroFrdFromImu(*msg);
    const Vec3 accel_FRD = ug_ekf::ros_adapter::AccelFrdFromImu(*msg);

    if (phase_ == Phase::ALIGNING) {
      if (align_t0_ < 0) align_t0_ = t;

      // 仅纳入"近似静止"样本
      const bool gyro_ok = gyro_FRD.norm() < static_cast<Scalar>(align_gyro_thresh_);
      const bool acc_ok  = std::abs(accel_FRD.norm() - static_cast<Scalar>(g_))
                           < static_cast<Scalar>(align_acc_thresh_);
      if (gyro_ok && acc_ok) {
        acc_buf_.push_back(accel_FRD);
        gyro_buf_.push_back(gyro_FRD);
      }

      if (t - align_t0_ >= static_cast<Scalar>(align_seconds_)) {
        finishAlignment();
      }
      return;
    }

    ekf_.predictImu(t, gyro_FRD, accel_FRD);
    last_gyro_FRD_ = gyro_FRD;
    have_state_ = true;
  }

  void onPressure(const sensor_msgs::FluidPressure::ConstPtr& msg) {
    if (phase_ != Phase::RUNNING) return;
    const Scalar t = static_cast<Scalar>(msg->header.stamp.toSec());
    const Scalar depth = ug_ekf::ros_adapter::DepthFromPressure(
        *msg, static_cast<Scalar>(p_atm_), static_cast<Scalar>(rho_),
        static_cast<Scalar>(g_));
    ekf_.updateDepth(t, depth, static_cast<Scalar>(depth_R_));
  }

  void onPublishTimer(const ros::TimerEvent&) {
    if (!have_state_ || !ekf_.initialized()) return;
    publishOdometry();
    publishDiagnostics();
  }

  // C·R_NB·D : body-FLU → ENU world
  static Mat3 NedFrdToEnuFlu(const Mat3& R_NB) {
    Mat3 C;
    C << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    Mat3 D = Vec3(1, -1, -1).asDiagonal();
    return C * R_NB * D;
  }

  void publishOdometry() {
    const State& x = ekf_.state();
    const Mat3 R_NB = x.q_NB.toRotationMatrix();

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(static_cast<double>(ekf_.diag().t_last));
    if (odom.header.stamp.isZero()) odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // 位置 NED → ENU
    odom.pose.pose.position.x = x.p_NED.y();   // E
    odom.pose.pose.position.y = x.p_NED.x();   // N
    odom.pose.pose.position.z = -x.p_NED.z();  // U

    // 姿态 body-FLU → ENU
    Eigen::Quaternion<Scalar> q_enu(NedFrdToEnuFlu(R_NB));
    q_enu.normalize();
    odom.pose.pose.orientation.x = q_enu.x();
    odom.pose.pose.orientation.y = q_enu.y();
    odom.pose.pose.orientation.z = q_enu.z();
    odom.pose.pose.orientation.w = q_enu.w();

    // twist: body-FLU 线速度/角速度
    Mat3 Dm = Vec3(1, -1, -1).asDiagonal();
    Vec3 v_body_frd = R_NB.transpose() * x.v_NED;
    Vec3 v_flu = Dm * v_body_frd;
    Vec3 w_flu = Dm * (last_gyro_FRD_ - x.b_g);
    odom.twist.twist.linear.x = v_flu.x();
    odom.twist.twist.linear.y = v_flu.y();
    odom.twist.twist.linear.z = v_flu.z();
    odom.twist.twist.angular.x = w_flu.x();
    odom.twist.twist.angular.y = w_flu.y();
    odom.twist.twist.angular.z = w_flu.z();

    // 协方差 (Phase 1: 位置对角精确映射，姿态/速度保守常数)
    const Mat15& P = ekf_.covariance();
    for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = P(1, 1);   // var(E)
    odom.pose.covariance[7]  = P(0, 0);   // var(N)
    odom.pose.covariance[14] = P(2, 2);   // var(U)=var(D)
    odom.pose.covariance[21] = std::max<double>(P(6, 6), 1e-3);   // roll
    odom.pose.covariance[28] = std::max<double>(P(7, 7), 1e-3);   // pitch
    odom.pose.covariance[35] = std::max<double>(P(8, 8), 1e-2);   // yaw (无观测，保守)

    for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0]  = P(3, 3);
    odom.twist.covariance[7]  = P(4, 4);
    odom.twist.covariance[14] = P(5, 5);

    odom_pub_.publish(odom);
  }

  void publishDiagnostics() {
    const State& x = ekf_.state();
    const Mat15& P = ekf_.covariance();
    const ug_ekf::Diagnostics& d = ekf_.diag();

    ug_msgs::EskfDiagnostics msg;
    msg.header.stamp = ros::Time::now();
    msg.t_last = static_cast<double>(d.t_last);
    msg.dt_last = static_cast<double>(d.dt_last);
    msg.nis_depth = static_cast<double>(d.last_nis_depth);
    msg.accept_depth = d.accept_depth;
    msg.reject_depth = d.reject_depth;
    msg.reset_count = d.reset_count;
    msg.P_trace_pos = static_cast<double>(P.block<3,3>(0,0).trace());
    msg.P_trace_vel = static_cast<double>(P.block<3,3>(3,3).trace());
    msg.P_trace_att = static_cast<double>(P.block<3,3>(6,6).trace());
    msg.b_g = {static_cast<double>(x.b_g.x()),
               static_cast<double>(x.b_g.y()),
               static_cast<double>(x.b_g.z())};
    msg.b_a = {static_cast<double>(x.b_a.x()),
               static_cast<double>(x.b_a.y()),
               static_cast<double>(x.b_a.z())};
    diag_pub_.publish(msg);
  }

  // ROS
  ros::Subscriber imu_sub_, pres_sub_;
  ros::Publisher  odom_pub_, diag_pub_;
  ros::Timer      pub_timer_;
  std::string odom_frame_, base_frame_;

  // EKF
  ug_ekf::Eskf ekf_;
  Phase phase_ = Phase::ALIGNING;
  bool have_state_ = false;
  Scalar align_t0_ = Scalar(-1);
  Vec3 last_gyro_FRD_ = Vec3::Zero();

  std::vector<Vec3> acc_buf_;
  std::vector<Vec3> gyro_buf_;

  // params
  double g_, p_atm_, rho_;
  double init_yaw_ned_rad_, align_seconds_, dt_max_, nis_gate_depth_, depth_R_;
  Vec3 r_pressure_frd_;
  double sigma_g_, sigma_a_, sigma_bg_, sigma_ba_;
  double p0_pos_, p0_vel_, p0_att_, p0_bg_, p0_ba_;
  double align_gyro_thresh_, align_acc_thresh_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "glider_ekf_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  GliderEkfNode node(nh, pnh);
  ros::spin();
  return 0;
}
