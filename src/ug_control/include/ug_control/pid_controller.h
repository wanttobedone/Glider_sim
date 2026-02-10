/**
 * pid_controller.h
 * 通用 PID 控制器类，带积分抗饱和和微分滤波
 */

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

  /**
   * @brief 计算 PID 输出
   * @param error 误差 = 目标值 - 实际值
   * @param dt 时间步长 [s]
   * @return 控制输出（已限幅）
   */
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

#endif  // UG_CONTROL_PID_CONTROLLER_H_
