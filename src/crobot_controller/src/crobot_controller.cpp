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

        const double wheel_separation = params_.wheel_separation_multiplier * params_.wheel_separation;
        const double back_left_wheel_radius = params_.back_left_wheel_radius_multiplier * params_.wheel_radius;
        const double back_right_wheel_radius = params_.back_right_wheel_radius_multiplier * params_.wheel_radius;
        const double front_left_wheel_radius = params_.front_left_wheel_radius_multiplier * params_.wheel_radius;
        const double front_right_wheel_radius = params_.front_right_wheel_radius_multiplier * params_.wheel_radius;

        odometry_.setWheelParams(wheel_separation, back_left_wheel_radius, back_right_wheel_radius, front_left_wheel_radius, front_right_wheel_radius);
        odometry_.setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

        cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
        publish_limited_velocity_ = params_.publish_limited_velocity;

        // setup linear and angular limiters

        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // setup publish limited velocity
        if (publish_limited_velocity_)
        {
            limited_velocity_publisher_ = get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
            realtime_limited_velocity_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
        }

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

        odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
        realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

        std::string tf_prefix = "";
        if (params_.tf_frame_prefix_enable)
        {
            if (params_.tf_frame_prefix != "")
            {
                tf_prefix = params_.tf_frame_prefix;
            } else {
                tf_prefix = std::string(get_node()->get_namespace());
            }

            if (tf_prefix == "/")
            {
                tf_prefix = "";
            } else {
                tf_prefix = tf_prefix + "/";
            }
        }

        const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
        const auto base_frame_id = tf_prefix + params_.base_frame_id;

        auto & odometry_message = realtime_odometry_publisher_->msg_;
        odometry_message.header.frame_id = odom_frame_id;
        odometry_message.child_frame_id = base_frame_id;

        publish_rate_ = params_.publish_rate;
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

        odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

        constexpr size_t NUM_DIMENSIONS = 0;
        for (size_t index = 0; index < 6; ++index)
        {
            const size_t diagonal_index = NUM_DIMENSIONS * index + index;
            odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
            odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
        }

        odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
            DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS()
        );
        realtime_odometry_transform_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
            odometry_transform_publisher_
        );

        auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
        odometry_transform_message.transforms.resize(1);
        odometry_transform_message.transforms.front().header.frame_id = odom_frame_id;
        odometry_transform_message.transforms.front().child_frame_id = base_frame_id;

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
            odometry_.updateOpenLoop(linear_command_x, linear_command_y, angular_command, time);
        } else {
            double back_left_feedback = registered_back_left_handle->feedback.get().get_value();
            double back_right_feedback = registered_back_right_handle->feedback.get().get_value();
            double front_left_feedback  = registered_front_left_handle->feedback.get().get_value();
            double front_right_feedback  = registered_front_right_handle->feedback.get().get_value();

            if (
                std::isnan(back_left_feedback) || 
                std::isnan(back_right_feedback) ||
                std::isnan(front_left_feedback) ||
                std::isnan(front_right_feedback)
            )
            {
                RCLCPP_ERROR(
                    logger, "One wheel %s is invalid", feedback_type()
                );
                return controller_interface::return_type::ERROR;
            }

            if (params_.position_feedback)
            {
                odometry_.update(back_left_feedback, back_right_feedback, front_left_feedback, front_right_feedback, time);
            } else {
                odometry_.updateFromVelocity(
                    back_left_feedback * back_left_wheel_radius * period.seconds(), 
                    back_right_feedback * back_right_wheel_radius * period.seconds(), 
                    front_left_feedback * front_left_wheel_radius * period.seconds(), 
                    front_right_feedback * front_left_wheel_radius * period.seconds(), 
                    time
                );
            }
        }

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, odometry_.getHeading());

        bool should_publish = false;
        try
        {
            if (previous_publish_timestamp_ + publish_period_ < time)
            {
                previous_publish_timestamp_ += publish_period_;
                should_publish = true;
            }
        }
        catch (const std::runtime_error &)
        {
            previous_publish_timestamp_ = time;
            should_publish = true;
        }

        if (should_publish)
        {
            if (realtime_odometry_publisher_->trylock())
            {
                auto & odometry_message = realtime_odometry_publisher_->msg_;
                odometry_message.header.stamp = time;
                odometry_message.pose.pose.position.x = odometry_.getX();
                odometry_message.pose.pose.position.y = odometry_.getY();
                odometry_message.pose.pose.orientation.x = orientation.x();
                odometry_message.pose.pose.orientation.y = orientation.y();
                odometry_message.pose.pose.orientation.z = orientation.z();
                odometry_message.pose.pose.orientation.w = orientation.w();
                odometry_message.twist.twist.linear.x = odometry_.getLinear();
                odometry_message.twist.twist.angular.z = odometry_.getAngular();
                realtime_odometry_publisher_->unlockAndPublish();
            }

            if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
            {
                auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
                transform.header.stamp = time;
                transform.transform.translation.x = odometry_.getX();
                transform.transform.translation.y = odometry_.getY();
                transform.transform.rotation.x = orientation.x();
                transform.transform.rotation.y = orientation.y();
                transform.transform.rotation.z = orientation.z();
                transform.transform.rotation.w = orientation.w();
                realtime_odometry_transform_publisher_->unlockAndPublish();
            }
        }

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

        registered_back_left_handle->velocity.get().set_value(back_left_wheel_velocity);
        registered_back_right_handle->velocity.get().set_value(back_right_wheel_velocity);
        registered_front_left_handle->velocity.get().set_value(front_left_wheel_velocity);
        registered_front_right_handle->velocity.get().set_value(front_right_wheel_velocity);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn CrobotController::on_activate(
        const rclcpp_lifecycle::State &
    )
    {
        const auto bl_result = configure_wheel(params_.back_left_wheel_name, registered_back_left_handle);
        const auto br_result = configure_wheel(params_.back_right_wheel_name, registered_back_right_handle);
        const auto fl_result = configure_wheel(params_.front_left_wheel_name, registered_front_left_handle);
        const auto fr_result = configure_wheel(params_.front_right_wheel_name, registered_front_right_handle);

        if (
            bl_result == controller_interface::CallbackReturn::ERROR || 
            br_result == controller_interface::CallbackReturn::ERROR || 
            fl_result == controller_interface::CallbackReturn::ERROR || 
            fr_result == controller_interface::CallbackReturn::ERROR
        )
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        is_halted = false;
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

        registered_back_left_handle.reset(nullptr);
        registered_back_right_handle.reset(nullptr);
        registered_front_left_handle.reset(nullptr);
        registered_front_right_handle.reset(nullptr);

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

    controller_interface::CallbackReturn CrobotController::configure_wheel(
        const std::string wheel_name, std::unique_ptr<WheelHandle> & registered_handle
    )
    {
        auto logger = get_node()->get_logger();

        const auto interface_name = feedback_type();
        const auto state_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&wheel_name, &interface_name](const auto & interface)
            {
                return interface.get_prefix_name() == wheel_name && 
                interface.get_interface_name() == interface_name;
            }
        );

        if (state_handle == state_interfaces_.cend())
        {
            RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        const auto command_handle = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [&wheel_name](const auto & interface)
            {
                return interface.get_prefix_name() == wheel_name && 
                    interface.get_interface_name() == HW_IF_VELOCITY;
            }
        );

        if (command_handle == command_interfaces_.end())
        {
            RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        registered_handle = std::make_unique<CrobotController::WheelHandle>(WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool CrobotController::reset()
    {
        odometry_.resetOdometry();

        std::queue<Twist> empty;
        std::swap(previous_commands_, empty);

        registered_back_left_handle.reset(nullptr);
        registered_back_right_handle.reset(nullptr);
        registered_front_left_handle.reset(nullptr);
        registered_front_right_handle.reset(nullptr);
        

        subscriber_is_active_ = false;
        velocity_command_subscriber_.reset();

        received_velocity_msg_ptr_.set(nullptr);
        is_halted = false;
        return true;
    }

    void CrobotController::halt()
    {
        const auto halt_wheels = [](auto & wheel_handle)
        {
            wheel_handle->velocity.get().set_value(0.0);
        };

        halt_wheels(registered_back_left_handle);
        halt_wheels(registered_back_right_handle);
        halt_wheels(registered_front_left_handle);
        halt_wheels(registered_front_right_handle);
    }
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  crobot_controller::CrobotController, controller_interface::ControllerInterface)