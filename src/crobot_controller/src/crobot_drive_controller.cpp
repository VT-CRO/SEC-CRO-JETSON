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

// ============================================================================
// LIFECYCLE
// ============================================================================

controller_interface::CallbackReturn CrobotDriveController::on_init()
{
    try
    {
        auto_declare<std::vector<std::string>>("wheel_joints", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("ankle_joints", std::vector<std::string>());

        auto_declare<std::string>("imu_joint", "base_to_imu_joint");

        auto_declare<double>("wheel_separation_width",  params_.wheel_separation_width);
        auto_declare<double>("wheel_separation_length", params_.wheel_separation_length);
        auto_declare<double>("wheel_radius",            params_.wheel_radius);

        auto_declare<double>("max_ankle_angle",      params_.max_ankle_angle);
        auto_declare<double>("max_linear_velocity",  params_.max_linear_velocity);
        auto_declare<double>("max_angular_velocity", params_.max_angular_velocity);

        auto_declare<bool>       ("enable_odom_tf",  params_.enable_odom_tf);
        auto_declare<std::string>("odom_frame_id",   params_.odom_frame_id);
        auto_declare<std::string>("base_frame_id",   params_.base_frame_id);

        auto_declare<std::string>("cmd_vel_topic", params_.cmd_vel_topic);
        auto_declare<std::string>("odom_topic",    params_.odom_topic);
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
    params_.wheel_joints = get_node()->get_parameter("wheel_joints").as_string_array();
    params_.ankle_joints = get_node()->get_parameter("ankle_joints").as_string_array();

    if (params_.wheel_joints.size() != 4 || params_.ankle_joints.size() != 4)
    {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Expected 4 wheel and 4 ankle joints, got %zu wheels and %zu ankles",
            params_.wheel_joints.size(), params_.ankle_joints.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    params_.wheel_separation_width  = get_node()->get_parameter("wheel_separation_width").as_double();
    params_.wheel_separation_length = get_node()->get_parameter("wheel_separation_length").as_double();
    params_.wheel_radius            = get_node()->get_parameter("wheel_radius").as_double();

    params_.max_ankle_angle      = get_node()->get_parameter("max_ankle_angle").as_double();
    params_.max_linear_velocity  = get_node()->get_parameter("max_linear_velocity").as_double();
    params_.max_angular_velocity = get_node()->get_parameter("max_angular_velocity").as_double();

    params_.enable_odom_tf  = get_node()->get_parameter("enable_odom_tf").as_bool();
    params_.odom_frame_id   = get_node()->get_parameter("odom_frame_id").as_string();
    params_.base_frame_id   = get_node()->get_parameter("base_frame_id").as_string();

    params_.cmd_vel_topic = get_node()->get_parameter("cmd_vel_topic").as_string();
    params_.odom_topic    = get_node()->get_parameter("odom_topic").as_string();

    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        params_.cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg)
        {
            received_cmd_vel_.writeFromNonRT(msg);
        });

    odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        get_node()->create_publisher<nav_msgs::msg::Odometry>(
            params_.odom_topic, rclcpp::SystemDefaultsQoS()));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

    RCLCPP_INFO(get_node()->get_logger(), "Configured CrobotDriveController (true swerve)");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CrobotDriveController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint : params_.ankle_joints)
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);

    for (const auto & joint : params_.wheel_joints)
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);

    return config;
}

controller_interface::InterfaceConfiguration
CrobotDriveController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // for (const auto& interface : state_interfaces_) {
    //     // Access details like name and interface name
    //     std::string joint_name = interface.get_name();
    //     std::string interface_name = interface.get_interface_name();
    //     // Log or use the names as needed
    //     RCLCPP_INFO(get_node()->get_logger(), "State Interface: %s, %s", joint_name.c_str(), interface_name.c_str());
    // }

    // for (const auto & joint : params_.ankle_joints)
    //     config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);

    for (const auto & joint : params_.wheel_joints)
        if (joint.find("front") != std::string::npos)  // Only front wheels have state interfaces
        {
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
            // config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        }

    config.names.push_back(params_.imu_joint + "/" + hardware_interface::HW_IF_VELOCITY);

    return config;
}

controller_interface::CallbackReturn CrobotDriveController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    resetOdometry();
    received_cmd_vel_.writeFromNonRT(std::make_shared<geometry_msgs::msg::Twist>());

    for (auto & a : assumed_ankle_angles_)
        a = 0.0;

    RCLCPP_INFO(get_node()->get_logger(), "Activated CrobotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CrobotDriveController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    for (size_t i = 0; i < 4; ++i)
    {
        command_interfaces_[i].set_value(0.0);      // ankle angles → 0
        command_interfaces_[i + 4].set_value(0.0);  // wheel velocities → 0
    }
    RCLCPP_INFO(get_node()->get_logger(), "Deactivated CrobotDriveController");
    return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// UPDATE LOOP
// ============================================================================

controller_interface::return_type CrobotDriveController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    auto cmd_vel = received_cmd_vel_.readFromRT();
    if (!cmd_vel || !(*cmd_vel))
    {
        for (size_t i = 0; i < 4; ++i)
        {
            command_interfaces_[i].set_value(0.0);
            command_interfaces_[i + 4].set_value(0.0);
        }
        return controller_interface::return_type::OK;
    }

    double linear_x = std::clamp((*cmd_vel)->linear.x,
        -params_.max_linear_velocity, params_.max_linear_velocity);
    double linear_y = std::clamp((*cmd_vel)->linear.y,
        -params_.max_linear_velocity, params_.max_linear_velocity);
    double angular_z = std::clamp((*cmd_vel)->angular.z,
        -params_.max_angular_velocity, params_.max_angular_velocity);

    auto commands = computeSwerve(linear_x, linear_y, angular_z);

    double dt = period.seconds();

    // -------------------------------------------------------------------------
    // Open-loop swerve optimization and ankle-alignment speed scaling
    //
    // Because the 270° servos have no position feedback, we maintain a software
    // model of where each ankle is assumed to be (assumed_ankle_angles_), and
    // use it to:
    //   1. Flip the wheel 180° + negate speed when that's a shorter turn.
    //   2. Hold wheel speed at zero while the ankle is still rotating into place.
    //   3. Ramp speed up smoothly as the ankle finishes aligning.
    // -------------------------------------------------------------------------
    // for (size_t i = 0; i < 4; ++i)
    // {
    //     double current_assumed = assumed_ankle_angles_[i];
    //     double target_angle    = commands.ankle_angles[i];
    //     double target_vel      = commands.wheel_vels[i];

    //     // Shortest-path angular error
    //     double error = normalizeAngle(target_angle - current_assumed);

    //     // Swerve optimization: if turning > 90°, flip 180° and negate velocity
    //     if (error > M_PI / 2.0)
    //     {
    //         target_angle = normalizeAngle(target_angle - M_PI);
    //         target_vel   = -target_vel;
    //         error       -= M_PI;
    //     }
    //     else if (error < -M_PI / 2.0)
    //     {
    //         target_angle = normalizeAngle(target_angle + M_PI);
    //         target_vel   = -target_vel;
    //         error       += M_PI;
    //     }

    //     // Advance the assumed ankle position at the physical servo slew rate
    //     double max_step = params_.assumed_servo_speed_ * dt;
    //     if (std::abs(error) <= max_step)
    //         assumed_ankle_angles_[i] = normalizeAngle(target_angle);
    //     else
    //         assumed_ankle_angles_[i] = normalizeAngle(
    //             current_assumed + std::copysign(max_step, error));

    //     // Remaining error after the model step
    //     double remaining_error = normalizeAngle(target_angle - assumed_ankle_angles_[i]);

    //     // Hold wheels while ankle is more than ~25° out of position,
    //     // then smoothly ramp up as it finishes aligning
    //     if (std::abs(remaining_error) > 0.45)
    //         target_vel = 0.0;
    //     else
    //         target_vel *= std::cos(remaining_error);

    //     commands.ankle_angles[i] = target_angle;
    //     commands.wheel_vels[i]   = target_vel;
    // }

    // Write to hardware interfaces
    for (size_t i = 0; i < 4; ++i)
    {
        command_interfaces_[i].set_value(commands.ankle_angles[i]);     // position (rad)
        command_interfaces_[i + 4].set_value(commands.wheel_vels[i]);   // velocity (rad/s)
    }

    updateOdometry(time, period);
    return controller_interface::return_type::OK;
}

// ============================================================================
// TRUE SWERVE KINEMATICS
// ============================================================================
//
// Each wheel module independently resolves its steering angle and drive speed
// from the robot body-frame velocity command (vx, vy, ω).
//
// Wheel positions relative to base_link (center of robot, all four corners):
//
//          +x (front)
//          |
//  FL -----+------ FR
//  |       |       |
//  |       +-------+---> +y (left)
//  |               |
//  BL -------------BR
//
//   FL: ( +L/2,  +W/2 )     index 0
//   FR: ( +L/2,  -W/2 )     index 1
//   BL: ( -L/2,  +W/2 )     index 2
//   BR: ( -L/2,  -W/2 )     index 3
//
// The velocity vector at wheel i due to rigid-body motion is:
//
//   vx_i = vx  -  ω * py_i
//   vy_i = vy  +  ω * px_i
//
// where (px_i, py_i) is the wheel's position in the robot frame.
//
// Steering angle and wheel speed then follow directly:
//
//   angle_i = atan2(vy_i, vx_i)
//   speed_i = hypot(vx_i, vy_i) / wheel_radius      [rad/s]
//
// All four wheels are solved simultaneously — no mode switching required.
// ============================================================================

CrobotDriveController::WheelAnkleCommand
CrobotDriveController::computeSwerve(double linear_x, double linear_y, double angular_z)
{
    WheelAnkleCommand cmd;
    cmd.ankle_angles.resize(4);
    cmd.wheel_vels.resize(4);

    const double half_L = params_.wheel_separation_length / 2.0;
    const double half_W = params_.wheel_separation_width  / 2.0;

    // Wheel positions in robot frame: [FL, FR, BL, BR]
    const double px[4] = { +half_L, +half_L, -half_L, -half_L };
    const double py[4] = { +half_W, -half_W, +half_W, -half_W };

    double max_speed = 0.0;  // used for normalization if any module overshoots

    for (int i = 0; i < 4; ++i)
    {
        // Body-frame velocity at this wheel's contact point
        double vx_i = linear_x - angular_z * py[i];
        double vy_i = linear_y + angular_z * px[i];

        double linear_speed = std::hypot(vx_i, vy_i);

        cmd.ankle_angles[i] = (linear_speed > 1e-6)
            ? std::atan2(vy_i, vx_i)
            : 0.0;                          // indeterminate angle → keep current

        cmd.wheel_vels[i] = linear_speed / params_.wheel_radius;   // rad/s

        if (linear_speed > max_speed)
            max_speed = linear_speed;
    }

    // Proportionally scale all wheel speeds down so no module exceeds the
    // physical max_linear_velocity limit, preserving the motion direction.
    if (max_speed > params_.max_linear_velocity)
    {
        double scale = params_.max_linear_velocity / max_speed;
        for (int i = 0; i < 4; ++i)
            cmd.wheel_vels[i] *= scale;
    }

    RCLCPP_DEBUG(get_node()->get_logger(),
        "Swerve: vx=%.3f vy=%.3f ω=%.3f → "
        "angles=[%.2f, %.2f, %.2f, %.2f] rad  "
        "vels=[%.2f, %.2f, %.2f, %.2f] rad/s",
        linear_x, linear_y, angular_z,
        cmd.ankle_angles[0], cmd.ankle_angles[1],
        cmd.ankle_angles[2], cmd.ankle_angles[3],
        cmd.wheel_vels[0], cmd.wheel_vels[1],
        cmd.wheel_vels[2], cmd.wheel_vels[3]);

    return cmd;
}

// ============================================================================
// ODOMETRY
// ============================================================================

void CrobotDriveController::updateOdometry(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    double dt = period.seconds();

    // Reconstruct body-frame velocity from encoder readings and assumed angles.
    // Each wheel contributes one estimate of (vx, vy); we average them.
    double vx = 0.0;
    double vy = 0.0;

    for (int i = 0; i < 2; ++i)
    {
        // Wheel velocity from encoder (state interface 4+i), sign-corrected
        double wheel_omega = state_interfaces_[i].get_value();
        double wheel_v     = -(params_.wheel_radius * wheel_omega);   // m/s

        vx += wheel_v * std::cos(assumed_ankle_angles_[i]);
        vy += wheel_v * std::sin(assumed_ankle_angles_[i]);
    }

    vx /= 2.0;
    vy /= 2.0;

    // Angular velocity from command
    double omega = 0.0;

    // for (int i = 0; i < 2; ++i) {
    //     // Wheel velocity from encoder (state interface 4+i), sign-corrected
    //     double wheel_omega = state_interfaces_[i].get_value();

    //     // Contribution to angular velocity from this wheel's tangential speed
    //     omega += -wheel_omega *
    //              std::sin(assumed_ankle_angles_[i]) *  // sin(steering angle)
    //              (params_.wheel_separation_length / 2.0);  // distance from center
    // }

    // omega /= 2.0;

    omega = state_interfaces_[2].get_value() * M_PI / 180.0;  // Use IMU angular velocity

    // Integrate pose in world frame
    odom_state_.x     += (vx * std::cos(odom_state_.theta) - vy * std::sin(odom_state_.theta)) * dt;
    odom_state_.y     += (vx * std::sin(odom_state_.theta) + vy * std::cos(odom_state_.theta)) * dt;
    odom_state_.theta  = normalizeAngle(odom_state_.theta + omega * dt);

    odom_state_.linear_x  = vx;
    odom_state_.linear_y  = vy;
    odom_state_.angular_z = omega;
    odom_state_.timestamp = time;

    // Publish nav_msgs/Odometry
    if (odom_pub_ && odom_pub_->trylock())
    {
        auto & msg = odom_pub_->msg_;
        msg.header.stamp     = time;
        msg.header.frame_id  = params_.odom_frame_id;
        msg.child_frame_id   = params_.base_frame_id;

        msg.pose.pose.position.x    = odom_state_.x;
        msg.pose.pose.position.y    = odom_state_.y;
        msg.pose.pose.position.z    = 0.0;
        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = std::sin(odom_state_.theta / 2.0);
        msg.pose.pose.orientation.w = std::cos(odom_state_.theta / 2.0);

        msg.twist.twist.linear.x  = odom_state_.linear_x;
        msg.twist.twist.linear.y  = odom_state_.linear_y;
        msg.twist.twist.angular.z = odom_state_.angular_z;

        odom_pub_->unlockAndPublish();
    }

    // Broadcast odom → base_link TF
    if (params_.enable_odom_tf && tf_broadcaster_)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp         = time;
        tf_msg.header.frame_id      = params_.odom_frame_id;
        tf_msg.child_frame_id       = params_.base_frame_id;
        tf_msg.transform.translation.x = odom_state_.x;
        tf_msg.transform.translation.y = odom_state_.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x    = 0.0;
        tf_msg.transform.rotation.y    = 0.0;
        tf_msg.transform.rotation.z    = std::sin(odom_state_.theta / 2.0);
        tf_msg.transform.rotation.w    = std::cos(odom_state_.theta / 2.0);
        tf_broadcaster_->sendTransform(tf_msg);
    }
}

void CrobotDriveController::resetOdometry()
{
    odom_state_ = {};
    odom_state_.timestamp = get_node()->now();
}

double CrobotDriveController::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace crobot_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    crobot_controller::CrobotDriveController,
    controller_interface::ControllerInterface)