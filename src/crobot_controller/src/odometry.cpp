#include "crobot_controller/odometry.hpp"

namespace crobot_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
    x_(0.0),
    y_(0.0),
    heading_(0.0),
    linear_(0.0),
    angular_(0.0),
    wheel_separation_(0.0),
    back_left_wheel_radius_(0.0),
    back_right_wheel_radius_(0.0),
    front_left_wheel_radius_(0.0),
    front_right_wheel_radius_(0.0),
    back_left_wheel_old_pos_(0.0),
    back_right_wheel_old_pos_(0.0),
    front_left_wheel_old_pos_(0.0),
    front_right_wheel_old_pos_(0.0),
    velocity_rolling_window_size_(velocity_rolling_window_size),
    linear_accumulator_(velocity_rolling_window_size),
    angular_accumulator_(velocity_rolling_window_size)
{
    
}

void Odometry::init(const rclcpp::Time & time)
{
    resetAccumulators();
    timestamp_ = time;
}

bool Odometry::update(double back_left_pos, double back_right_pos, double front_left_pos, double front_right_pos, const rclcpp::Time & time)
{
    // TODO
    return false;
}

bool Odometry::updateFromVelocity(double back_left_vel, double back_right_vel, double front_left_vel, double front_right_vel, const rclcpp::Time & time)
{
    // TODO
    return false;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
    // TODO
    return;
}

void Odometry::resetOdometry()
{
    // TODO
    return;
}

void Odometry::setWheelParams(double wheel_separation, double back_left_wheel_radius, double back_right_wheel_radius, double front_left_wheel_radius, double front_right_wheel_radius)
{
    wheel_separation_ = wheel_separation;
    back_left_wheel_radius_ = back_left_wheel_radius;
    back_right_wheel_radius_ = back_right_wheel_radius;
    front_left_wheel_radius_ = front_left_wheel_radius;
    front_right_wheel_radius_ = front_right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(std::size_t velocity_rolling_window_size)
{
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
    // TODO
    return;
}

void Odometry::integrateExact(double linear, double angular)
{
    // TODO
    return;    
}

void Odometry::resetAccumulators()
{
    linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}
}