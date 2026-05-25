/**
 * UG Magnetometer Plugin
 *
 * Gazebo ModelPlugin 模拟三轴磁力计：
 *   读取指定 link 的世界姿态 (Gazebo 世界系 = ENU)
 *   构造世界系磁场向量 m_W (按 declination/inclination/strength)
 *   投影到 body 系：m_b_FLU = R_BW^T * m_W   (Gazebo body = FLU)
 *   施加 soft-iron 矩阵 W、hard-iron 偏置 V 与高斯噪声
 *   发布 sensor_msgs/MagneticField (单位 Tesla)
 *
 * 注意：发布的 m_b 仍处于 ROS body-FLU 约定下；FLU→FRD 的轴翻转由EKF 的 ROS sensor_adapter 完成
 * SDF 参数：
 *   <robot_namespace>     ROS 命名空间
 *   <link_name>           磁力计绑定 link (建议 mag_link)
 *   <sensor_topic>        发布话题 (默认 "mag")
 *   <update_rate>         更新频率 Hz (默认 50)
 *   <reference_frame>     "ENU" | "NED"  默认 ENU (与 Gazebo 世界一致)
 *   <field_strength_T>    总场强 (默认 5.0e-5, 约南昌地区)
 *   <declination_deg>     磁偏角 (从地理北向东为正，默认 0)
 *   <inclination_deg>     磁倾角 (北半球向下为正，默认 45)
 *   <noise_sigma_T>       三轴噪声 stddev，T (默认 5e-7)
 *   <hard_iron_T>         "x y z" 硬铁偏置 (默认 "0 0 0")
 *   <soft_iron>           9 个数 row-major 软铁矩阵 (默认 单位阵)
 */

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <random>
#include <sstream>
#include <array>

namespace gazebo
{

class UGMagnetometerPlugin : public ModelPlugin
{
public:
  UGMagnetometerPlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    world_ = _model->GetWorld();

    std::string ns = "ug_glider";
    if (_sdf->HasElement("robot_namespace"))
      ns = _sdf->Get<std::string>("robot_namespace");

    std::string linkName;
    if (_sdf->HasElement("link_name"))
      linkName = _sdf->Get<std::string>("link_name");
    else
    {
      gzerr << "[UGMag] 缺少 <link_name>\n";
      return;
    }

    link_ = _model->GetLink(linkName);
    if (!link_)
    {
      gzerr << "[UGMag] 找不到 link: " << linkName << "\n";
      return;
    }

    std::string topic = "mag";
    if (_sdf->HasElement("sensor_topic"))
      topic = _sdf->Get<std::string>("sensor_topic");

    if (_sdf->HasElement("update_rate"))
      updateRate_ = _sdf->Get<double>("update_rate");

    std::string refFrame = "ENU";
    if (_sdf->HasElement("reference_frame"))
      refFrame = _sdf->Get<std::string>("reference_frame");

    double fieldT = 5.0e-5;
    if (_sdf->HasElement("field_strength_T"))
      fieldT = _sdf->Get<double>("field_strength_T");

    double declDeg = 0.0;
    if (_sdf->HasElement("declination_deg"))
      declDeg = _sdf->Get<double>("declination_deg");

    double incDeg = 45.0;
    if (_sdf->HasElement("inclination_deg"))
      incDeg = _sdf->Get<double>("inclination_deg");

    if (_sdf->HasElement("noise_sigma_T"))
      noiseSigma_ = _sdf->Get<double>("noise_sigma_T");

    if (_sdf->HasElement("hard_iron_T"))
    {
      std::istringstream iss(_sdf->Get<std::string>("hard_iron_T"));
      iss >> hardIron_[0] >> hardIron_[1] >> hardIron_[2];
    }

    softIron_ = {1, 0, 0,  0, 1, 0,  0, 0, 1};
    if (_sdf->HasElement("soft_iron"))
    {
      std::istringstream iss(_sdf->Get<std::string>("soft_iron"));
      for (int i = 0; i < 9; ++i) iss >> softIron_[i];
    }

    // 构造世界系参考磁场向量 m_W (单位 T)
    // NED 中：m_N_NED = field * [cos(inc)*cos(decl), cos(inc)*sin(decl), sin(inc)]
    //   x=北, y=东, z=下；inclination 向下为正。
    const double inc = incDeg * M_PI / 180.0;
    const double decl = declDeg * M_PI / 180.0;
    const double mN = fieldT * std::cos(inc) * std::cos(decl);
    const double mE = fieldT * std::cos(inc) * std::sin(decl);
    const double mD = fieldT * std::sin(inc);

    if (refFrame == "NED")
    {
      mWorld_ = ignition::math::Vector3d(mN, mE, mD);
    }
    else  // ENU (Gazebo world)
    {
      // ENU: x=East, y=North, z=Up；与 NED 互换
      mWorld_ = ignition::math::Vector3d(mE, mN, -mD);
    }

    noiseDist_ = std::normal_distribution<double>(0.0, noiseSigma_);

    rosNode_.reset(new ros::NodeHandle(ns));
    pub_ = rosNode_->advertise<sensor_msgs::MagneticField>(topic, 10);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&UGMagnetometerPlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[UGMag] link=" << linkName
          << " topic=/" << ns << "/" << topic
          << " rate=" << updateRate_ << "Hz"
          << " field=" << fieldT << "T"
          << " decl=" << declDeg << "deg inc=" << incDeg << "deg"
          << " ref=" << refFrame
          << " m_World=[" << mWorld_.X() << "," << mWorld_.Y() << "," << mWorld_.Z() << "]\n";
  }

private:
  void OnUpdate(const common::UpdateInfo& _info)
  {
    if (updateRate_ > 0.0)
    {
      double dt = (_info.simTime - lastUpdateTime_).Double();
      if (dt < 1.0 / updateRate_) return;
    }
    lastUpdateTime_ = _info.simTime;

    // body 姿态：q_BW (body→world，world=ENU)
    auto pose = link_->WorldPose();
    ignition::math::Quaterniond q = pose.Rot();

    // 世界向量旋到 body：m_b = R_BW^T * m_W
    ignition::math::Vector3d mBody = q.RotateVectorReverse(mWorld_);

    // soft-iron W * m_b + hard-iron V
    double mx = softIron_[0]*mBody.X() + softIron_[1]*mBody.Y() + softIron_[2]*mBody.Z() + hardIron_[0];
    double my = softIron_[3]*mBody.X() + softIron_[4]*mBody.Y() + softIron_[5]*mBody.Z() + hardIron_[1];
    double mz = softIron_[6]*mBody.X() + softIron_[7]*mBody.Y() + softIron_[8]*mBody.Z() + hardIron_[2];

    // 噪声
    mx += noiseDist_(rng_);
    my += noiseDist_(rng_);
    mz += noiseDist_(rng_);

    sensor_msgs::MagneticField msg;
    msg.header.stamp.sec = _info.simTime.sec;
    msg.header.stamp.nsec = _info.simTime.nsec;
    msg.header.frame_id = link_->GetName();
    msg.magnetic_field.x = mx;
    msg.magnetic_field.y = my;
    msg.magnetic_field.z = mz;
    const double var = noiseSigma_ * noiseSigma_;
    msg.magnetic_field_covariance[0] = var;
    msg.magnetic_field_covariance[4] = var;
    msg.magnetic_field_covariance[8] = var;

    pub_.publish(msg);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  boost::shared_ptr<ros::NodeHandle> rosNode_;
  ros::Publisher pub_;

  double updateRate_ = 50.0; //传感器消息更新频率
  double noiseSigma_ = 5.0e-7;       // T
  std::array<double, 3> hardIron_{0, 0, 0};
  std::array<double, 9> softIron_{1, 0, 0, 0, 1, 0, 0, 0, 1};
  ignition::math::Vector3d mWorld_;

  common::Time lastUpdateTime_;
  std::default_random_engine rng_{std::random_device{}()};
  std::normal_distribution<double> noiseDist_{0.0, 5.0e-7};
};

GZ_REGISTER_MODEL_PLUGIN(UGMagnetometerPlugin)

}  // namespace gazebo
