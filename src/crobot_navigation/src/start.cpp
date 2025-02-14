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

    publisher_ = node_ptr_->create_publisher<DynamicInterface>("/gpio_controller/commands", 10);

    _shouldStart = false;
}

BT::NodeStatus StartBehavior::onStart()
{
     const auto timer_period = std::chrono::seconds(10);

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

        auto msg = DynamicInterface();
        msg.interface_groups = {"crobot_systems"};

        auto interface = InterfaceValue();
        interface.interface_names = {"start_robot"};

        interface.values = {1.0};
        msg.interface_values = {interface};

        RCLCPP_INFO(node_ptr_->get_logger(), "Publishing: Group: %s, Interface: %s, Value: %f",
            msg.interface_groups[0].c_str(),
            interface.interface_names[0].c_str(),
            interface.values[0]);

        publisher_-> publish(msg);

        RCLCPP_INFO(node_ptr_->get_logger(), "Starting routine...");
        rclcpp::sleep_for(std::chrono::milliseconds(50)); // Give ROS time to process
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