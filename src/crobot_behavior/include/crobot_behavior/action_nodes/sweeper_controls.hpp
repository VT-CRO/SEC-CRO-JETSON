#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/float64.hpp"


class SweeperControl : public BT::StatefulActionNode
{
public:
    SweeperControl(
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
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    std_msgs::msg::Float64 msg;
    std::string sweeperString;
    bool done_ = false;
};