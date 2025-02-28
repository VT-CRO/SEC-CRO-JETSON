#include <chrono>

#include "crobot_navigation/behaviors/idle.hpp"

IdleBehavior::IdleBehavior(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr)
    : BT::SyncActionNode(name, config),
    node_ptr_(node_ptr)
{
    // create publisher, topic /run
	node_ptr->create_publisher<BoolMsg>("/run", 10);
}

BT::NodeStatus IdleBehavior::tick()
{
    // publish false
	auto msg = DynamicInterface();
    msg.interface_groups = {"crobot_systems"};

    auto interface = InterfaceValue();
    interface.interface_names = {"start_robot"};

	interface.values = {0};
    msg.interface_values = {interface};

    publisher_-> publish(msg);

    return BT::NodeStatus::SUCCESS;
}