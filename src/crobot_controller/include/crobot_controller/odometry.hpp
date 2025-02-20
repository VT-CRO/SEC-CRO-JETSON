#ifndef DIFF_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define DIFF_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

namespace crobot_controller
{
class Odometry
{
public:
  explicit Odometry();

  bool updatePos(long leftTicks, long rightTicks, long auxTicks long front_left_pos const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return _x; }
  double getY() const { return _y; }
  double getHeading() const { return _heading; }

private:
  // Current pos:
  double _x;        //   [cm]
  double _y;        //   [cm]
  double _heading;  //   [rad]

  float _PI; //pi

  float _R; // Odo Wheel Radius [cm]
  short _N; // Ticks per revolution Encoder Revolution (4096)
  float _B; // The distance between the midpoints of the left and right wheel and the aux wheel [cm]
  float _L; // The distance between the center of the left and right wheels [cm]
  float _CM_PER_TICK; // What the name says [cm]


  // The tick values of each encoder in the current iteration
  long _currentLeftPosition;
  long _currentRightPosition;
  long _currentAuxPosition;

  // The tick values of each encoder from the previous iteration
  long _prevLeftPosition;
  long _prevRightPosition;
  long _prevAuxPosition;
};

}  // namespace diff_drive_controller

#endif  // DIFF_DRIVE_CONTROLLER__ODOMETRY_HPP_