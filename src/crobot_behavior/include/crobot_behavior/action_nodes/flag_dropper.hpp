#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

class FlagDropper : public BT::StatefulActionNode
{
public:
    FlagDropper(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node_ptr
    );

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    std_msgs::msg::Float64 msg_;
    std::string flag_string_;
    rclcpp::Time start_time_;
    static constexpr double TIMEOUT_SEC = 30.0;
};