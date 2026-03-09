#ifndef CROBOT_CONTROLLER__CROBOT_DRIVE_CONTROLLER_HPP_
#define CROBOT_CONTROLLER__CROBOT_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace crobot_controller
{

class CrobotDriveController : public controller_interface::ControllerInterface
{
public:
    CrobotDriveController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    struct Params
    {
        std::vector<std::string> wheel_joints;  // [fl, fr, bl, br]
        std::vector<std::string> ankle_joints;  // [fl, fr, bl, br]

        std::string imu_joint;

        // Robot geometry (meters)
        double wheel_separation_width  = 0.150;   // left-right wheel spacing
        double wheel_separation_length = 0.230;   // front-back wheel spacing
        double wheel_radius            = 0.035;   // wheel radius

        // Estimated maximum servo slew rate (rad/s); tune to match physical servo speed
        double assumed_servo_speed_ = 1.57;

        // Swerve optimization
        double max_ankle_angle = M_PI / 2.0;      // hard limit from servo range

        // Velocity limits
        double max_linear_velocity  = 1.5;   // m/s
        double max_angular_velocity = 1.5;   // rad/s

        // Odometry
        bool        enable_odom_tf  = true;
        std::string odom_frame_id   = "odom";
        std::string base_frame_id   = "base_link";

        // Topics
        std::string cmd_vel_topic = "/cmd_vel";
        std::string odom_topic    = "~/odom";

        // Ankle angle limits (radians)
        std::vector<double> ankle_min_angles = {-1.885, -0.524, -0.436, -1.728};  // [FL, FR, BL, BR]
        std::vector<double> ankle_max_angles = { 0.471,  1.676,  1.920,  0.628};
    } params_;

    // Command velocity subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> received_cmd_vel_;

    // Odometry publisher
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Odometry state
    struct OdomState
    {
        double x = 0.0, y = 0.0, theta = 0.0;
        double linear_x = 0.0, linear_y = 0.0, angular_z = 0.0;
        rclcpp::Time timestamp;
    } odom_state_;

    // Per-wheel swerve module command
    struct WheelAnkleCommand
    {
        std::vector<double> ankle_angles;  // [fl, fr, bl, br] radians
        std::vector<double> wheel_vels;    // [fl, fr, bl, br] rad/s
    };

    WheelAnkleCommand computeSwerve(double linear_x, double linear_y, double angular_z);

    // Odometry
    void updateOdometry(const rclcpp::Time & time, const rclcpp::Duration & period);
    void resetOdometry();

    double normalizeAngle(double angle);

    // Open-loop ankle angle tracking (servos have no position feedback)
    std::vector<double> assumed_ankle_angles_ = {0.0, 0.0, 0.0, 0.0};
};

}  // namespace crobot_controller

#endif  // CROBOT_CONTROLLER__CROBOT_DRIVE_CONTROLLER_HPP_