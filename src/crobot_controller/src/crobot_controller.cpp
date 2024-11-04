#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "crobot_controller/crobot_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}

namespace crobot_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;

    CrobotController::CrobotController() : controller_interface::ControllerInterface() {}

    const char * CrobotController::feedback_type() const
    {
        return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
    }

    controller_interface::CallbackReturn CrobotController::on_init()
    {
        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration CrobotController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        conf_names.push_back(params_.back_left_wheel_name + "/" + HW_IF_VELOCITY);
        conf_names.push_back(params_.front_left_wheel_name + "/" + HW_IF_VELOCITY);
        conf_names.push_back(params_.back_right_wheel_name + "/" + HW_IF_VELOCITY);
        conf_names.push_back(params_.front_right_wheel_name + "/" + HW_IF_VELOCITY);

        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration CrobotController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;

        conf_names.push_back(params_.back_left_wheel_name + "/" + feedback_type());
        conf_names.push_back(params_.front_left_wheel_name + "/" + feedback_type());
        conf_names.push_back(params_.back_right_wheel_name + "/" + feedback_type());
        conf_names.push_back(params_.front_right_wheel_name + "/" + feedback_type());

        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn CrobotController::on_configure(
        const rclcpp_lifecycle::State &
    )
    {
        auto logger = get_node()->get_logger();

        if (param_listener_->is_old(params_))
        {
            params_ = param_listener_->get_params();
            RCLCPP_INFO(logger, "Parameters were updated");
        }

        // set up odometry

        // cmd_vel_timeout_ = std::chrono::milliseconds{static_case<int>(params_.cmd_vel_timeout * 1000.0)};
        // publish_limited_velocity_ = params._publish_limited_velocity;

        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // setup publish limited velocity

        const Twist empty_twist;
        received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));
        previous_commands_.emplace(empty_twist);
        previous_commands_.emplace(empty_twist);

        velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
            DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<Twist> msg) -> void
            {
                if (!subscriber_is_active_)
                {
                    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
                    return;
                }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                {
                    RCLCPP_WARN_ONCE(
                        get_node()->get_logger(),
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once"
                    );
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                received_velocity_msg_ptr_.set(std::move(msg));
            }
        );

        // set up odometry publisher

        // setup odometry and transform publishers

        previous_update_timestamp_ = get_node()->get_clock()->now();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CrobotController::update(
        const rclcpp::Time & time, const rclcpp::Duration &period
    )
    {
        auto logger = get_node()->get_logger();

        if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
        {
            if (!is_halted)
            {
                halt();
                is_halted = true;
            }
            return controller_interface::return_type::OK;
        }

        std::shared_ptr<Twist> last_command_msg;
        received_velocity_msg_ptr_.get(last_command_msg);

        if (last_command_msg == nullptr)
        {
            RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
            return controller_interface::return_type::ERROR;
        }

        const auto age_of_last_command = time - last_command_msg->header.stamp;
        if (age_of_last_command > cmd_vel_timeout_)
        {
            last_command_msg->twist.linear.x = 0.0;
            last_command_msg->twist.angular.z = 0.0;
        }

        Twist command = *last_command_msg;
        double & linear_command_x = command.twist.linear.x;
        double & linear_command_y = command.twist.linear.y;
        double & angular_command = command.twist.angular.z;

        previous_update_timestamp_ = time;

        const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
        const double back_left_wheel_radius = params_.back_left_wheel_radius_multiplier * params_.wheel_radius;
        const double back_right_wheel_radius = params_.back_right_wheel_radius_multiplier * params_.wheel_radius;
        const double front_left_wheel_radius = params_.front_left_wheel_radius_multiplier * params_.wheel_radius;
        const double front_right_wheel_radius = params_.front_right_wheel_radius_multiplier * params_.wheel_radius;

        if (params_.open_loop)
        {
            // odometry_.updateOpenLoop()
        } else {
            // get feedback

            // check if feedback is valid

            // update odometry
        }

        tf2::Quaternion orientation;
        // orientation.setRPY(0.0, 0.0, odometry)

        bool should_publish = false;
        
        // publish updated odometry if should publish

        auto & last_command = previous_commands_.back().twist;
        auto & second_to_last_command = previous_commands_.front().twist;

        // apply linear limits on linear and angular commands

        previous_commands_.pop();
        previous_commands_.emplace(command);

        // publish limited velocity
        // if (publish_limited_velocity_ && reality)

        const double back_left_wheel_velocity = (linear_command_y - linear_command_x - 12 * (angular_command)) / back_left_wheel_radius;
        const double back_right_wheel_velocity = (linear_command_y - linear_command_x + 12 * (angular_command)) / back_right_wheel_radius;
        const double front_left_wheel_velocity = (linear_command_y + linear_command_x - 12 * (angular_command)) / front_left_wheel_radius;
        const double front_right_wheel_velocity = (linear_command_y + linear_command_x + 12 * (angular_command)) / front_right_wheel_radius;

        // registered_back_left_handle.velocity.get().set_value(back_left_wheel_velocity);
        // registered_back_right_handle.velocity.get().set_value(back_right_wheel_velocity);
        // registered_front_left_handle.velocity.get().set_value(front_left_wheel_velocity);
        // registered_front_right_handle.velocity.get().set_value(front_right_wheel_velocity);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn CrobotController::on_activate(
        const rclcpp_lifecycle::State &
    )
    {

        // configure wheels

        is_halted - false;
        subscriber_is_active_ = true;

        RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CrobotController::on_deactivate(
        const rclcpp_lifecycle::State &
    )
    {
        subscriber_is_active_ = false;
        if (!is_halted)
        {
            halt();
            is_halted = true;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CrobotController::on_cleanup(
        const rclcpp_lifecycle::State &
    )
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }
        received_velocity_msg_ptr_.set(std::make_shared<Twist>());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CrobotController::on_error(
        const rclcpp_lifecycle::State &
    )
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CrobotController::on_shutdown(
        const rclcpp_lifecycle::State &
    )
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool CrobotController::reset()
    {
        // reset odometry

        std::queue<Twist> empty;
        std::swap(previous_commands_, empty);

        subscriber_is_active_ = false;
        velocity_command_subscriber_.reset();

        received_velocity_msg_ptr_.set(nullptr);
        is_halted = false;
        return true;
    }

    void CrobotController::halt()
    {
        
    }
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  crobot_controller::CrobotController, controller_interface::ControllerInterface)