#ifndef CROBOT_CONTROLLER__CROBOT_DRIVE_CONTROLLER_HPP_
#define CROBOT_CONTROLLER__CROBOT_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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
        
        // Robot geometry (meters)
        double wheel_separation_width = 0.150;   // 150mm left-right
        double wheel_separation_length = 0.230;  // 230mm front-back
        double wheel_radius = 0.035;             // 35mm radius
        
        double point_turn_speed_threshold = 0.05;  // m/s - below this, use point turn
        double strafe_angle_threshold = 0.1;       // rad - if motion is nearly perpendicular
        double max_ankle_angle = M_PI / 4.0;       // 45 degrees max steering
        
        // Velocity limits
        double max_linear_velocity = 1.0;   // m/s
        double max_angular_velocity = 2.0;  // rad/s
        
        // Odometry
        bool enable_odom_tf = true;
        std::string odom_frame_id = "odom";
        std::string base_frame_id = "base_link";
        
        // Topic names
        std::string cmd_vel_topic = "/cmd_vel";
        std::string odom_topic = "~/odom";
    } params_;

    // Command velocity subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> received_cmd_vel_;
    
    // Odometry publisher
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;
    
    // Odometry state
    struct OdomState
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double linear_x = 0.0;
        double linear_y = 0.0;
        double angular_z = 0.0;
        rclcpp::Time timestamp;
    } odom_state_;

    // Kinematics functions
    struct WheelAnkleCommand
    {
        std::vector<double> ankle_angles;  // [fl, fr, bl, br] in radians
        std::vector<double> wheel_vels;    // [fl, fr, bl, br] in rad/s
    };
    
    WheelAnkleCommand computePointTurn(double angular_z);
    WheelAnkleCommand computeStrafeMode(double linear_x, double linear_y);
    WheelAnkleCommand computeAckermannMode(double linear_x, double linear_y, double angular_z);
    WheelAnkleCommand blendKinematics(double linear_x, double linear_y, double angular_z);
    
    // Odometry computation
    void updateOdometry(const rclcpp::Time & time, const rclcpp::Duration & period);
    
    // Helper functions
    void resetOdometry();
    double normalizeAngle(double angle);

    // Track the estimated position of the servos since we can't read them
    std::vector<double> assumed_ankle_angles_ = {0.0, 0.0, 0.0, 0.0};
    
    // Estimate of how fast your servos can physically rotate in rad/s
    // (e.g., 5.0 rad/s is roughly 0.2 seconds per 60 degrees)
    double assumed_servo_speed_ = 4.0;
};

}  // namespace crobot_controller

#endif  // CROBOT_CONTROLLER__CROBOT_DRIVE_CONTROLLER_HPP_