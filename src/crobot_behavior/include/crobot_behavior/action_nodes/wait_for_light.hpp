#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"

class WaitForLight : public BT::StatefulActionNode
{
public:
    WaitForLight(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node_ptr
    );

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted();

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    int last_value_ = 1000000;
    int threshold_ = -9999;
};