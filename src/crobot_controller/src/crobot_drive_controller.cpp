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

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

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
    
    double dt = period.seconds();

    // --- OPEN-LOOP SWERVE OPTIMIZATION & ALIGNMENT ---
    for (size_t i = 0; i < 4; ++i)
    {
        double current_assumed_angle = assumed_ankle_angles_[i];
        double target_angle = commands.ankle_angles[i];
        double target_vel = commands.wheel_vels[i];

        // Find the shortest angular distance
        double error = normalizeAngle(target_angle - current_assumed_angle);

        // 1. Swerve Optimization (Flip 180 degrees if turn is > 90 degrees)
        if (error > M_PI / 2.0)
        {
            target_angle -= M_PI;
            target_vel = -target_vel;
            error -= M_PI;
        }
        else if (error < -M_PI / 2.0)
        {
            target_angle += M_PI;
            target_vel = -target_vel;
            error += M_PI;
        }

        // 2. Advance our software's "assumed" position based on expected servo speed
        double max_step = assumed_servo_speed_ * dt;
        
        if (std::abs(error) <= max_step) {
            // The servo has arrived at the target
            assumed_ankle_angles_[i] = normalizeAngle(target_angle);
        } else {
            // Move our assumed position closer to the target at the max servo speed
            assumed_ankle_angles_[i] = normalizeAngle(current_assumed_angle + std::copysign(max_step, error));
        }

        // Recalculate error based on our new assumed position for velocity scaling
        double current_error = normalizeAngle(target_angle - assumed_ankle_angles_[i]);

        // 3. Wait for Ankles:
        // If the assumed position is still > ~25 degrees (0.45 rad) away, stop the wheel motor
        if (std::abs(current_error) > 0.45) 
        {
            target_vel = 0.0;
        }
        else 
        {
            // Smoothly ramp up drive speed as the assumed ankle finishes aligning
            target_vel *= std::cos(current_error);
        }

        // Output to hardware
        // We command the physical servo to the absolute target angle immediately
        commands.ankle_angles[i] = normalizeAngle(target_angle);
        commands.wheel_vels[i] = target_vel;
    }
    // -------------------------------------------------

    // Apply commands to hardware interfaces
    for (size_t i = 0; i < 4; ++i)
    {
        command_interfaces_[i].set_value(commands.ankle_angles[i]);      // Ankles
        command_interfaces_[i + 4].set_value(commands.wheel_vels[i]);     // Wheels
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

    // Convert angular velocity to wheel linear velocity
    // v_wheel = ω_robot × distance_from_center
    double r_turn = std::hypot(params_.wheel_separation_width / 2.0, 
                               params_.wheel_separation_length / 2.0);
    double wheel_linear_vel = std::abs(angular_z) /* * r_turn */;  // m/s

    // Direction: positive angular_z (CCW) vs negative (CW)
    double direction = (angular_z > 0) ? 1.0 : -1.0;
 
    cmd.wheel_vels[0] = -wheel_linear_vel * direction;  // FL
    cmd.wheel_vels[1] = wheel_linear_vel * direction;   // FR
    cmd.wheel_vels[2] = -wheel_linear_vel * direction;  // BL
    cmd.wheel_vels[3] = wheel_linear_vel * direction;   // BR

    RCLCPP_INFO(get_node()->get_logger(), 
        "Point Turn: ω=%.3f rad/s, r=%.3f m, v_wheel=%.3f m/s, [FL=%.3f, FR=%.3f, BL=%.3f, BR=%.3f] m/s",
        angular_z, r_turn, wheel_linear_vel,
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
    // const double angle = M_PI / 2.0;

    double sign = (linear_y > 0) ? 1.0 : -1.0;
    
    cmd.ankle_angles[0] = -160.0 * M_PI / 180.0;  // FL
    cmd.ankle_angles[1] = 160.0 * M_PI / 180.0;   // FR
    cmd.ankle_angles[2] = 120.0 * M_PI / 180.0;   // BL
    cmd.ankle_angles[3] = 13.333 * M_PI / 180.0;  // BR

    // All wheels same speed for lateral motion (in m/s)
    double wheel_linear_vel = std::abs(linear_y);
    
    cmd.wheel_vels[0] = wheel_linear_vel * -sign; // FL
    cmd.wheel_vels[1] = wheel_linear_vel * sign; // FR
    cmd.wheel_vels[2] = wheel_linear_vel * sign; // BL
    cmd.wheel_vels[3] = wheel_linear_vel * -sign; // BR

    return cmd;
}

CrobotDriveController::WheelAnkleCommand 
CrobotDriveController::computeAckermannMode(double linear_x, double linear_y, double angular_z)
{
    WheelAnkleCommand cmd;
    cmd.ankle_angles.resize(4);
    cmd.wheel_vels.resize(4);

    double max_steering_angle = M_PI / 8.0;
    
    // Scale ankle angle based on angular_z
    double steering_angle = 0.0;
    if (std::abs(angular_z) > 0.01)
    {
        // Proportional: more angular_z → more ankle angle
        steering_angle = (angular_z / params_.max_angular_velocity) * max_steering_angle;
        steering_angle = std::clamp(steering_angle, -max_steering_angle, max_steering_angle);
    }

    cmd.ankle_angles[0] = steering_angle;   // FL
    cmd.ankle_angles[1] = steering_angle;   // FR
    cmd.ankle_angles[2] = -steering_angle;  // BL
    cmd.ankle_angles[3] = -steering_angle;  // BR

    double wheel_linear_vel = linear_x;
    
    for (int i = 0; i < 4; ++i)
    {
        cmd.wheel_vels[i] = wheel_linear_vel;  // m/s
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
    
    double dt = period.seconds();

    // if (dt <= 0.0) {
    //     return;
    // }
    
    // Get current commanded velocities from the last command
    auto cmd_vel = received_cmd_vel_.readFromRT();
    if (cmd_vel && *cmd_vel)
    {
        // double vx = (*cmd_vel)->linear.x;
        // double vy = (*cmd_vel)->linear.y;

        double wheel_omega[4];
        for (int i = 0; i < 4; ++i) {
            wheel_omega[i] = state_interfaces_[4 + i].get_value();
        }

        double wheel_v[4];
        for (int i = 0; i < 4; ++i) {
            // tryna change this to (-) because velocities are backwards
            wheel_v[i] = -(params_.wheel_radius * wheel_omega[i]);
        }

        double vx = 0.0;
        double vy = 0.0;

        for (int i = 0; i < 4; ++i) {
            vx += wheel_v[i] * std::cos(assumed_ankle_angles_[i]);
            vy += wheel_v[i] * std::sin(assumed_ankle_angles_[i]);
        }

        vx /= 4.0;
        vy /= 4.0;

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

    if (params_.enable_odom_tf && tf_broadcaster_)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = time;
        tf_msg.header.frame_id = params_.odom_frame_id;
        tf_msg.child_frame_id = params_.base_frame_id;

        tf_msg.transform.translation.x = odom_state_.x;
        tf_msg.transform.translation.y = odom_state_.y;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = std::sin(odom_state_.theta / 2.0);
        tf_msg.transform.rotation.w = std::cos(odom_state_.theta / 2.0);

        tf_broadcaster_->sendTransform(tf_msg);
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