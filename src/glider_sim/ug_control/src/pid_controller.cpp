/**
 * 通用 PID 控制器实现
 */

#include <ug_control/pid_controller.h>

namespace ug_control
{

PIDController::PIDController()
    : kp_(0.0), ki_(0.0), kd_(0.0),
      outputMin_(-1e10), outputMax_(1e10),
      maxIntegral_(1e10),
      integral_(0.0), prevError_(0.0),
      firstRun_(true)
{
}

//增益参数设置
void PIDController::setGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
//限幅
void PIDController::setOutputLimits(double min, double max)
{
  outputMin_ = min;
  outputMax_ = max;
}

void PIDController::setIntegralLimits(double max_integral)
{
  maxIntegral_ = std::abs(max_integral);
}
//切换控制模式等时复位
void PIDController::reset()
{
  integral_ = 0.0;
  prevError_ = 0.0;
  firstRun_ = true;
}

double PIDController::compute(double error, double dt)
{
  if (dt <= 0.0)
    return 0.0;

  // P项
  double p_term = kp_ * error;

  // I，抗饱和
  integral_ += error * dt;
  // 限制积分累积
  integral_ = std::max(-maxIntegral_, std::min(maxIntegral_, integral_));
  double i_term = ki_ * integral_;

  // D项（首次运行不计算微分，避免跳变）
  double d_term = 0.0;
  if (!firstRun_)
  {
    double derivative = (error - prevError_) / dt;
    d_term = kd_ * derivative;
  }
  firstRun_ = false;
  prevError_ = error;

  // 输出限幅
  double output = p_term + i_term + d_term;
  output = std::max(outputMin_, std::min(outputMax_, output));

  return output;
}

}  // namespace ug_control
