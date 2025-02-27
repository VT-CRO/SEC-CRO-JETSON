/**
 * @file
 * @author Jayson De La Vega
 * @date 8/10/24
 * @brief This file contains the crobot controller declaration based on the ROS2 control framework. 
            Adapted from https://github.com/ros-controls/ros2_controllers/tree/master/mecanum_drive_controller
 */

#ifndef CROBOT_CONTROL__CROBOT_DRIVE_CONTROLLER_HPP_
#define CROBOT_CONTROL__CROBOT_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "crobot_controller/visibility_control.h"
#include "crobot_controller/odometry.hpp"
#include <crobot_controller/crobot_drive_controller_parameters.hpp>

namespace crobot_controller
{

    static constexpr size_t NUM_STATE_INTERFACES = 4;
    static constexpr size_t NUM_CMD_INTERFACES = 4;

    class CrobotDriveController : public controller_interface::ControllerInterface
    {
        using Twist = geometry_msgs::msg::TwistStamped;

    public:
        CROBOT_CONTROL_PUBLIC
        CrobotDriveController();

        CROBOT_CONTROL_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
        using OdomStateMsg = nav_msgs::msg::Odometry;
        using TFStateMsg = tf2_msgs::msg::TFMessage;

    protected:
    
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        enum WheelIndex : std::size_t
        {
            FRONT_LEFT = 0,
            FRONT_RIGHT = 1,
            REAR_RIGHT = 2,
            REAR_LEFT = 3
        };

        enum DeadWheelIndex : std::size_t
        {
            DEADWHEEL_X = 0,
            DEADWHEEL_Y = 1,
            DEADWHEEL_HEADING = 2
        };

        std::vector<std::string> command_joint_names_;
        std::vector<std::string> state_joint_names_;

        rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_cmd_subscriber_ = nullptr;
        realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;
        rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);

        using OdomStatePublisher = realtime_tools::RealtimePublisher<OdomStateMsg>;
        rclcpp::Publisher<OdomStateMsg>::SharedPtr odom_state_publisher_;
        std::unique_ptr<OdomStatePublisher> rt_odom_state_publisher_;

        using TfStatePublisher = realtime_tools::RealtimePublisher<TFStateMsg>;
        rclcpp::Publisher<TFStateMsg>::SharedPtr tf_odom_state_publisher_;
        std::unique_ptr<TfStatePublisher> rt_tf_odom_state_publisher_;

        // Odometry odometry_;

        void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

        double velocity_in_center_frame_linear_x_;
        double velocity_in_center_frame_linear_y_;
        double velocity_in_center_frame_angular_z_;
    };
}

#endif