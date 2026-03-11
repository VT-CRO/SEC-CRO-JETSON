#include "crobot_behavior/action_nodes/sweeper_controls.hpp"
#include <string>

SweeperControl::SweeperControl(const std::string &name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node_ptr)
: BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    publisher_ = node_ptr->create_publisher<std_msgs::msg::Float64>(
        "/sweeper_position_controller/commands", 10);
}

BT::PortsList SweeperControl::providedPorts()
{
    return { BT::InputPort<std::string>("sweeper_command") };
}

BT::NodeStatus SweeperControl::onStart()
{
    auto sweeperCommand = getInput("sweeper_command", sweeperString);
    if (!sweeperCommand)
        throw BT::RuntimeError("Missing required input [sweeper_command]");

    msg.data = std::stod(std::string(sweeperString));
    start_time_ = node_ptr_->now();

    // Publish immediately — don't wait for onRunning().
    // The controller subscriber may not exist yet, but we try here AND keep
    // retrying in onRunning() until it lands.
    publisher_->publish(msg);
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] Sending sweeper to %.3f rad (subscribers: %zu)",
        name().c_str(), msg.data, publisher_->get_subscription_count());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SweeperControl::onRunning()
{
    double elapsed = (node_ptr_->now() - start_time_).seconds();

    if (elapsed > TIMEOUT_SEC) {
        RCLCPP_ERROR(node_ptr_->get_logger(),
            "[%s] Timed out waiting for controller subscriber after %.1fs",
            name().c_str(), elapsed);
        return BT::NodeStatus::FAILURE;
    }

    size_t subs = publisher_->get_subscription_count();
    if (subs == 0) {
        // Controller not subscribed yet — keep retrying every tick (20 Hz)
        RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(),
            *node_ptr_->get_clock(), 500,
            "[%s] Waiting for controller subscriber... (%.1fs elapsed)",
            name().c_str(), elapsed);
        return BT::NodeStatus::RUNNING;
    }

    // Controller is subscribed — publish and confirm delivery.
    // Publish twice in case the first lands mid-update-cycle.
    publisher_->publish(msg);
    RCLCPP_INFO(node_ptr_->get_logger(),
        "[%s] Sweeper command delivered (%.3f rad)", name().c_str(), msg.data);
    return BT::NodeStatus::SUCCESS;
}

void SweeperControl::onHalted()
{
    RCLCPP_WARN(node_ptr_->get_logger(), "[%s] Halted", name().c_str());
}