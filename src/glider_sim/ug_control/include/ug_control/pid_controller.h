// PID 控制器类，带积分抗饱，后续实现微分滤波

#ifndef UG_CONTROL_PID_CONTROLLER_H_
#define UG_CONTROL_PID_CONTROLLER_H_

#include <algorithm>
#include <cmath>

namespace ug_control
{

class PIDController
{
public:
  PIDController();

  void setGains(double kp, double ki, double kd);
  void setOutputLimits(double min, double max);
  void setIntegralLimits(double max_integral);
  void reset();

  // 输入误差，dt，输出控制量，已限幅；计算PID输出
  double compute(double error, double dt);

private:
  double kp_, ki_, kd_;
  double outputMin_, outputMax_;
  double maxIntegral_;

  double integral_;
  double prevError_;
  bool firstRun_;
};

}  // namespace ug_control

#endif
