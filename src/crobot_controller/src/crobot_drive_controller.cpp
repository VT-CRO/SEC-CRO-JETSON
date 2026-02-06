#include "crobot_controller/crobot_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace crobot_controller
{

CrobotDriveController::CrobotDriveController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn CrobotDriveController::on_init()
{
    try
    {
        // Declare parameters
        auto_declare<std::vector<std::string>>("wheel_joints", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("ankle_joints", std::vector<std::string>());
        
        auto_declare<double>("wheel_separation_width", params_.wheel_separation_width);
        auto_declare<double>("wheel_separation_length", params_.wheel_separation_length);
        auto_declare<double>("wheel_radius", params_.wheel_radius);
        
        auto_declare<double>("point_turn_speed_threshold", params_.point_turn_speed_threshold);
        auto_declare<double>("strafe_angle_threshold", params_.strafe_angle_threshold);
        auto_declare<double>("max_ankle_angle", params_.max_ankle_angle);
        
        auto_declare<double>("max_linear_velocity", params_.max_linear_velocity);
        auto_declare<double>("max_angular_velocity", params_.max_angular_velocity);
        
        auto_declare<bool>("enable_odom_tf", params_.enable_odom_tf);
        auto_declare<std::string>("odom_frame_id", params_.odom_frame_id);
        auto_declare<std::string>("base_frame_id", params_.base_frame_id);
        
        auto_declare<std::string>("cmd_vel_topic", params_.cmd_vel_topic);
        auto_declare<std::string>("odom_topic", params_.odom_topic);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CrobotDriveController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Get parameters
    params_.wheel_joints = get_node()->get_parameter("wheel_joints").as_string_array();
    params_.ankle_joints = get_node()->get_parameter("ankle_joints").as_string_array();
    
    if (params_.wheel_joints.size() != 4 || params_.ankle_joints.size() != 4)
    {
        RCLCPP_ERROR(get_node()->get_logger(), 
            "Expected 4 wheel and 4 ankle joints, got %zu wheels and %zu ankles",
            params_.wheel_joints.size(), params_.ankle_joints.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    
    params_.wheel_separation_width = get_node()->get_parameter("wheel_separation_width").as_double();
    params_.wheel_separation_length = get_node()->get_parameter("wheel_separation_length").as_double();
    params_.wheel_radius = get_node()->get_parameter("wheel_radius").as_double();
    
    params_.point_turn_speed_threshold = get_node()->get_parameter("point_turn_speed_threshold").as_double();
    params_.strafe_angle_threshold = get_node()->get_parameter("strafe_angle_threshold").as_double();
    params_.max_ankle_angle = get_node()->get_parameter("max_ankle_angle").as_double();
    
    params_.max_linear_velocity = get_node()->get_parameter("max_linear_velocity").as_double();
    params_.max_angular_velocity = get_node()->get_parameter("max_angular_velocity").as_double();
    
    params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();
    params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
    params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();
    
    params_.cmd_vel_topic = get_node()->get_parameter("cmd_vel_topic").as_string();
    params_.odom_topic = get_node()->get_parameter("odom_topic").as_string();

    // Create cmd_vel subscriber
    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        params_.cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg)
        {
            received_cmd_vel_.writeFromNonRT(msg);
        });

    // Create odometry publisher
    odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        get_node()->create_publisher<nav_msgs::msg::Odometry>(
            params_.odom_topic, rclcpp::SystemDefaultsQoS()));

    RCLCPP_INFO(get_node()->get_logger(), "Configured CrobotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
CrobotDriveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Claim all 4 ankle position command interfaces
    for (const auto & joint : params_.ankle_joints)
    {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }

    // Claim all 4 wheel velocity command interfaces
    for (const auto & joint : params_.wheel_joints)
    {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    return config;
}

controller_interface::InterfaceConfiguration 
CrobotDriveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Read all 4 ankle position state interfaces
    for (const auto & joint : params_.ankle_joints)
    {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }

    // Read all 4 wheel velocity state interfaces
    for (const auto & joint : params_.wheel_joints)
    {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    return config;
}

controller_interface::CallbackReturn CrobotDriveController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Reset odometry
    resetOdometry();
    
    // Initialize command to zero
    std::shared_ptr<geometry_msgs::msg::Twist> zero_cmd = 
        std::make_shared<geometry_msgs::msg::Twist>();
    received_cmd_vel_.writeFromNonRT(zero_cmd);

    RCLCPP_INFO(get_node()->get_logger(), "Activated CrobotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CrobotDriveController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Stop the robot
    for (size_t i = 0; i < 4; ++i)
    {
        command_interfaces_[i].set_value(0.0);      // Ankle angles to 0
        command_interfaces_[i + 4].set_value(0.0);  // Wheel velocities to 0
    }

    RCLCPP_INFO(get_node()->get_logger(), "Deactivated CrobotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CrobotDriveController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Get the latest command
    auto cmd_vel = received_cmd_vel_.readFromRT();
    if (!cmd_vel || !(*cmd_vel))
    {
        // No command received, stop
        for (size_t i = 0; i < 4; ++i)
        {
            command_interfaces_[i].set_value(0.0);      // Ankles
            command_interfaces_[i + 4].set_value(0.0);  // Wheels
        }
        return controller_interface::return_type::OK;
    }

    // Extract velocity commands and clamp
    double linear_x = std::clamp((*cmd_vel)->linear.x, 
        -params_.max_linear_velocity, params_.max_linear_velocity);
    double linear_y = std::clamp((*cmd_vel)->linear.y, 
        -params_.max_linear_velocity, params_.max_linear_velocity);
    double angular_z = std::clamp((*cmd_vel)->angular.z, 
        -params_.max_angular_velocity, params_.max_angular_velocity);

    // Compute kinematics
    auto commands = blendKinematics(linear_x, linear_y, angular_z);

    // Apply commands
    // Ankles: command_interfaces_[0..3]
    for (size_t i = 0; i < 4; ++i)
    {
        command_interfaces_[i].set_value(commands.ankle_angles[i]);
    }

    // Wheels: command_interfaces_[4..7]
    for (size_t i = 0; i < 4; ++i)
    {
        command_interfaces_[i + 4].set_value(commands.wheel_vels[i]);
    }

    // Update odometry
    updateOdometry(time, period);

    return controller_interface::return_type::OK;
}

// ============================================================================
// KINEMATICS IMPLEMENTATIONS
// ============================================================================

CrobotDriveController::WheelAnkleCommand 
CrobotDriveController::computePointTurn(double angular_z)
{
    WheelAnkleCommand cmd;
    cmd.ankle_angles.resize(4);
    cmd.wheel_vels.resize(4);

    // Point turn configuration: X pattern
    // FL: -45°, FR: +45°, BL: +45°, BR: -45°
    const double angle = M_PI / 4.0;  // 45 degrees
    
    cmd.ankle_angles[0] = -angle;  // FL
    cmd.ankle_angles[1] = angle;   // FR
    cmd.ankle_angles[2] = angle;   // BL
    cmd.ankle_angles[3] = -angle;  // BR

    // Convert angular velocity to wheel velocity
    // For point turn: v_wheel = ω * r_turn
    // where r_turn is the distance from robot center to wheel
    double r_turn = std::hypot(params_.wheel_separation_width / 2.0, 
                               params_.wheel_separation_length / 2.0);
    
    // Use ABSOLUTE value of angular_z to compute magnitude
    double wheel_linear_vel = std::abs(angular_z) * r_turn;
    double wheel_angular_vel = wheel_linear_vel / params_.wheel_radius;

    // Sign for direction: positive angular_z (CCW) vs negative (CW)
    double direction = (angular_z > 0) ? 1.0 : -1.0;
    
    // For CCW (positive angular_z):
    //   FL, FR go backward (negative), BL, BR go forward (positive)
    // For CW (negative angular_z): opposite
    cmd.wheel_vels[0] = -wheel_angular_vel * direction;  // FL
    cmd.wheel_vels[1] = -wheel_angular_vel * direction;  // FR
    cmd.wheel_vels[2] = wheel_angular_vel * direction;   // BL
    cmd.wheel_vels[3] = wheel_angular_vel * direction;   // BR

    RCLCPP_INFO(get_node()->get_logger(), 
        "Point Turn: ω=%.3f, r=%.3f, wheel_vel=%.3f, [FL=%.3f, FR=%.3f, BL=%.3f, BR=%.3f]",
        angular_z, r_turn, wheel_angular_vel,
        cmd.wheel_vels[0], cmd.wheel_vels[1], cmd.wheel_vels[2], cmd.wheel_vels[3]);

    return cmd;
}

CrobotDriveController::WheelAnkleCommand 
CrobotDriveController::computeStrafeMode(double linear_x, double linear_y)
{
    WheelAnkleCommand cmd;
    cmd.ankle_angles.resize(4);
    cmd.wheel_vels.resize(4);

    // Pure strafe: all wheels at 90 degrees
    const double angle = M_PI / 2.0;
    
    // If moving +Y (left), ankles point left
    // If moving -Y (right), ankles point right
    double sign = (linear_y > 0) ? 1.0 : -1.0;
    
    cmd.ankle_angles[0] = -angle * sign;  // FL
    cmd.ankle_angles[1] = angle * sign;   // FR
    cmd.ankle_angles[2] = angle * sign;   // BL
    cmd.ankle_angles[3] = -angle * sign;  // BR

    // All wheels same speed for lateral motion
    double wheel_angular_vel = std::abs(linear_y) / params_.wheel_radius;
    
    for (int i = 0; i < 4; ++i)
    {
        cmd.wheel_vels[i] = wheel_angular_vel * sign;
    }

    return cmd;
}

CrobotDriveController::WheelAnkleCommand 
CrobotDriveController::computeAckermannMode(double linear_x, double linear_y, double angular_z)
{
    WheelAnkleCommand cmd;
    cmd.ankle_angles.resize(4);
    cmd.wheel_vels.resize(4);

    // Compute desired heading from linear velocities
    double speed = std::hypot(linear_x, linear_y);
    double heading = std::atan2(linear_y, linear_x);
    
    // Compute steering angle from curvature
    // For Ackermann: ω = v / R, so R = v / ω
    // Steering angle ≈ L / R (for small angles)
    double steering_angle = 0.0;
    if (speed > 0.01)  // Avoid division by zero
    {
        double turn_radius = speed / std::abs(angular_z);
        // Use wheelbase as characteristic length
        steering_angle = std::atan2(params_.wheel_separation_length, turn_radius);
        steering_angle = std::clamp(steering_angle, -params_.max_ankle_angle, params_.max_ankle_angle);
        
        // Preserve sign from angular_z
        if (angular_z < 0)
        {
            steering_angle = -steering_angle;
        }
    }

    // Simple Ackermann: front wheels steer, back wheels follow
    // In practice, for better control, we'll use parallel steering (all same angle)
    cmd.ankle_angles[0] = steering_angle;   // FL
    cmd.ankle_angles[1] = steering_angle;   // FR
    cmd.ankle_angles[2] = -steering_angle;  // BL (opposite for rear)
    cmd.ankle_angles[3] = -steering_angle;  // BR (opposite for rear)

    // Convert linear velocity to wheel angular velocity
    // IMPORTANT: Use linear_x to preserve forward/backward direction
    double wheel_angular_vel = linear_x / params_.wheel_radius;
    
    // All wheels drive at same speed (simplified - could add differential)
    for (int i = 0; i < 4; ++i)
    {
        cmd.wheel_vels[i] = wheel_angular_vel;
    }

    return cmd;
}

CrobotDriveController::WheelAnkleCommand 
CrobotDriveController::blendKinematics(double linear_x, double linear_y, double angular_z)
{
    double speed = std::hypot(linear_x, linear_y);
    
    // MODE 1: Point turn (very low speed, significant rotation)
    if (speed < params_.point_turn_speed_threshold && std::abs(angular_z) > 0.01)
    {
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Mode: Point Turn (speed=%.3f, ω=%.3f)", speed, angular_z);
        return computePointTurn(angular_z);
    }
    
    // MODE 2: Pure strafe (moving perpendicular, minimal forward/backward)
    double motion_angle = std::atan2(linear_y, linear_x);
    bool is_sideways = std::abs(std::abs(motion_angle) - M_PI / 2.0) < params_.strafe_angle_threshold;
    
    if (is_sideways && std::abs(angular_z) < 0.1)
    {
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Mode: Strafe (vx=%.3f, vy=%.3f)", linear_x, linear_y);
        return computeStrafeMode(linear_x, linear_y);
    }
    
    // MODE 3: Ackermann-style steering (default for all other cases)
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Mode: Ackermann (vx=%.3f, vy=%.3f, ω=%.3f)", linear_x, linear_y, angular_z);
    return computeAckermannMode(linear_x, linear_y, angular_z);
}

// ============================================================================
// ODOMETRY
// ============================================================================

void CrobotDriveController::updateOdometry(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // Simple odometry based on commanded velocities
    // For better accuracy, you could use wheel encoder feedback from state_interfaces_
    
    double dt = period.seconds();
    
    // Get current commanded velocities from the last command
    auto cmd_vel = received_cmd_vel_.readFromRT();
    if (cmd_vel && *cmd_vel)
    {
        double vx = (*cmd_vel)->linear.x;
        double vy = (*cmd_vel)->linear.y;
        double omega = (*cmd_vel)->angular.z;
        
        // Update pose using simple integration
        // Transform velocities from body frame to world frame
        double delta_x = (vx * std::cos(odom_state_.theta) - vy * std::sin(odom_state_.theta)) * dt;
        double delta_y = (vx * std::sin(odom_state_.theta) + vy * std::cos(odom_state_.theta)) * dt;
        double delta_theta = omega * dt;
        
        odom_state_.x += delta_x;
        odom_state_.y += delta_y;
        odom_state_.theta += delta_theta;
        odom_state_.theta = normalizeAngle(odom_state_.theta);
        
        odom_state_.linear_x = vx;
        odom_state_.linear_y = vy;
        odom_state_.angular_z = omega;
        odom_state_.timestamp = time;
    }

    // Publish odometry
    if (odom_pub_ && odom_pub_->trylock())
    {
        auto & odom_msg = odom_pub_->msg_;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = params_.odom_frame_id;
        odom_msg.child_frame_id = params_.base_frame_id;
        
        odom_msg.pose.pose.position.x = odom_state_.x;
        odom_msg.pose.pose.position.y = odom_state_.y;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(odom_state_.theta / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(odom_state_.theta / 2.0);
        
        odom_msg.twist.twist.linear.x = odom_state_.linear_x;
        odom_msg.twist.twist.linear.y = odom_state_.linear_y;
        odom_msg.twist.twist.angular.z = odom_state_.angular_z;
        
        odom_pub_->unlockAndPublish();
    }
}

void CrobotDriveController::resetOdometry()
{
    odom_state_.x = 0.0;
    odom_state_.y = 0.0;
    odom_state_.theta = 0.0;
    odom_state_.linear_x = 0.0;
    odom_state_.linear_y = 0.0;
    odom_state_.angular_z = 0.0;
    odom_state_.timestamp = get_node()->now();
}

double CrobotDriveController::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace crobot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    crobot_controller::CrobotDriveController,
    controller_interface::ControllerInterface)