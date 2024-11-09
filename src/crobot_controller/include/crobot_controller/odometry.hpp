#ifndef DIFF_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define DIFF_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace crobot_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool update(double back_left_pos, double back_right_pos, double front_left_pos, double front_right_pos, const rclcpp::Time & time);
  bool updateFromVelocity(double back_left_vel, double back_right_vel, double front_left_vel, double front_right_vel, const rclcpp::Time & time);
  void updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_separation, double back_left_wheel_radius, double back_right_wheel_radius, double front_left_wheel_radius, double front_right_wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
#else
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;
#endif

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_;
  double back_left_wheel_radius_;
  double back_right_wheel_radius_;
  double front_left_wheel_radius_;
  double front_right_wheel_radius_;

  // Previous wheel position/state [rad]:
  double back_left_wheel_old_pos_;
  double back_right_wheel_old_pos_;
  double front_left_wheel_old_pos_;
  double front_right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace diff_drive_controller

#endif  // DIFF_DRIVE_CONTROLLER__ODOMETRY_HPP_