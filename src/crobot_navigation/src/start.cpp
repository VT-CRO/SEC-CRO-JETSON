#include <chrono>

#include "crobot_navigation/behaviors/start.hpp"

StartBehavior::StartBehavior(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config),
      node_ptr_(node_ptr)
{
    RCLCPP_INFO(node_ptr_->get_logger(), "Creating start");

    node_ptr_->create_subscription<BoolMsg>(
        "/start_led_bool", 10, std::bind(&StartBehavior::start_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_ptr_->get_logger(), "Created subscriber.");

    _shouldStart = false;
}

BT::NodeStatus StartBehavior::onStart()
{
     const auto timer_period = std::chrono::seconds(5);

    timer_ = node_ptr_->create_wall_timer(
        timer_period,
        std::bind(&StartBehavior::timer_callback, this)
    );

    RCLCPP_INFO(node_ptr_->get_logger(), "Created timer.");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StartBehavior::onRunning()
{
    if (_shouldStart) {
        timer_->cancel();
        RCLCPP_INFO(node_ptr_->get_logger(), "Starting routine...");
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

void StartBehavior::onHalted()
{

}

void StartBehavior::start_callback(const std_msgs::msg::Bool & msg)
{
    if (msg.data) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Observed the start LED.");
        _shouldStart = true;
    }
}

void StartBehavior::timer_callback()
{
    RCLCPP_INFO(node_ptr_->get_logger(), "Timer ran out.");
    _shouldStart = true;
}