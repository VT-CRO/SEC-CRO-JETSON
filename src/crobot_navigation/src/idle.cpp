#include <chrono>

#include "crobot_navigation/behaviors/idle.hpp"

StartBehavior::StartBehavior(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config),
    node_ptr_(node_ptr)
{
    // create publisher, topic /run
	node_ptr->create_publisher<BoolMsg>("/run", 10);
}

IdleBehavior::tick()
{
    // publish false
	auto msg = DynamicInterface();
    msg.interface_groups = {"crobot_systems"};

    auto interface = InterfaceValue();
    interface.interface_names = {"robot_start"};

	interface.values = {0};
    msg.interface_values = {interface};

    publisher_-> publish(msg);

    return BT::NodeStatus::SUCCESS;
}