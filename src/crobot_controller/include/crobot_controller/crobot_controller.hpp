/**
 * @file
 * @author Jayson De La Vega
 * @date 8/10/24
 * @brief This file contains the crobot controller declaration based on the ROS2 control framework
 */

#ifndef CROBOT_CONTROL__CROBOT_CONTROL_HPP_
#define CROBOT_CONTROL__CROBOT_CONTROL_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "crobot_controller/visibility_control.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "crobot_controller/odometry.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "crobot_controller_parameters.hpp"

namespace crobot_controller
{
    class CrobotController : public controller_interface::ControllerInterface
    {
        using Twist = geometry_msgs::msg::TwistStamped;

    public:
        CROBOT_CONTROL_PUBLIC
        CrobotController();

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

        CROBOT_CONTROL_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        CROBOT_CONTROL_PUBLIC
        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

    protected:
        struct WheelHandle
        {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
        };

        const char * feedback_type() const;
        controller_interface::CallbackReturn configure_side(
            const std::string & side, const std::vector<std::string> & wheel_name,
            WheelHandle registered_handle
        );

        WheelHandle* registered_back_left_handle;
        WheelHandle* registered_back_right_handle;
        WheelHandle* registered_front_left_handle;
        WheelHandle* registered_front_right_handle;
        
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        // Odometry odometry_;

        rclcpp::Time previous_update_timestamp_{0};

        std::chrono::milliseconds cmd_vel_timeout_{500};

        realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};
        std::queue<Twist> previous_commands_;
        bool subscriber_is_active_ = false;
        rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;

        bool reset();
        void halt();
        
        bool is_halted = false;
    };
}

#endif