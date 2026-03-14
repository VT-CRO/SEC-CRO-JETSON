#include "crobot_behavior/action_nodes/flag_dropper_control.hpp"
#include <string>

FlagDropperControl::FlagDropperControl(const std::string &name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_ptr_(node_ptr) {
    publisher_ = node_ptr_->create_publisher<std_msgs::msg::Float64>(
        "/flagdropper_controller/commands", 10);
}

BT::PortsList FlagDropperControl::providedPorts() {
    return { BT::InputPort<std::string>("flag_command") };
}

BT::NodeStatus FlagDropperControl::onStart() {
    auto result = getInput("flag_command", flag_string_);
    if (!result) {
        throw BT::RuntimeError("Missing required input [flag_command]");
    }

    msg_.data = std::stod(flag_string_);
    start_time_ = node_ptr_->now();

    publisher_->publish(msg_);
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] Sending flag dropper command %.3f rad (subs: %zu)",
        name().c_str(), msg_.data, publisher_->get_subscription_count());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FlagDropperControl::onRunning() {
    double time_elapsed = (node_ptr_->now() - start_time_).seconds();

    if (time_elapsed > TIMEOUT_SEC) {
        RCLCPP_ERROR(node_ptr_->get_logger(),
            "[%s] Timed out waiting for flag controller subscriber",
            name().c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (publisher_->get_subscription_count() == 0) {
        RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(),
            *node_ptr_->get_clock(), 500,
            "[%s] Waiting for flag controller subscriber...",
            name().c_str());
        return BT::NodeStatus::RUNNING;
    }

    publisher_->publish(msg_);
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] Flag dop command delivered: %.3f rad",
        name().c_str(), msg_.data);

    return BT::NodeStatus::SUCCESS;
}

void FlagDropperControl::onHalted() {
    RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Halted", name().c_str());
}