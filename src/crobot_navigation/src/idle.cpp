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
	BoolMsg msg;
	msg.data = false;
	publisher_->publish(msg);
	return BT::NodeStatus::RUNNING;
}