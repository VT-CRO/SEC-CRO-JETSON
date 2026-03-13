#include "crobot_behavior/action_nodes/embedded_mode.hpp"

EmbeddedModeControl::EmbeddedModeControl(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    publisher_ = node_ptr_->create_publisher<std_msgs::msg::String>("/crobot_embedded_mode", 10);
}

BT::PortsList EmbeddedModeControl::providedPorts()
{
    return { BT::InputPort<std::string>("mode") };
}

BT::NodeStatus EmbeddedModeControl::onStart()
{
    auto mode_input = getInput("mode", mode_string_);
    if (!mode_input)
    {
        throw BT::RuntimeError("Missing required input [mode]");
    }

    msg_.data = mode_string_;
    start_time_ = node_ptr_->now();

    publisher_->publish(msg_);

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[%s] Sending embedded mode command: %s (subscribers: %zu)",
        name().c_str(),
        msg_.data.c_str(),
        publisher_->get_subscription_count());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus EmbeddedModeControl::onRunning()
{
    double elapsed = (node_ptr_->now() - start_time_).seconds();

    if (elapsed > TIMEOUT_SEC)
    {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "[%s] Timed out waiting for embedded mode subscriber after %.1fs",
            name().c_str(),
            elapsed);
        return BT::NodeStatus::FAILURE;
    }

    size_t subs = publisher_->get_subscription_count();
    if (subs == 0)
    {
        RCLCPP_WARN_THROTTLE(
            node_ptr_->get_logger(),
            *node_ptr_->get_clock(),
            500,
            "[%s] Waiting for embedded mode subscriber... (%.1fs elapsed)",
            name().c_str(),
            elapsed);
        return BT::NodeStatus::RUNNING;
    }

    publisher_->publish(msg_);

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "[%s] Embedded mode command delivered: %s",
        name().c_str(),
        msg_.data.c_str());

    return BT::NodeStatus::SUCCESS;
}

void EmbeddedModeControl::onHalted()
{
    RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Halted", name().c_str());
}