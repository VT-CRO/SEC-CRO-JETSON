#include "crobot_behavior/action_nodes/wait_for_light.hpp"

WaitForLight::WaitForLight(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp::Node::SharedPtr node_ptr
) : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
    sub_ = node_ptr->create_subscription<std_msgs::msg::Int32>(
        "/photoresistor", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            last_value_ = msg->data;
        }
    );
}

BT::PortsList WaitForLight::providedPorts()
{
    return {
        BT::InputPort<int>("threshold")
    };
}

BT::NodeStatus WaitForLight::onStart()
{
    getInput("threshold", threshold_);
    if (last_value_ < threshold_) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForLight::onRunning()
{
    if (last_value_ < threshold_) {
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

void WaitForLight::onHalted()
{

}