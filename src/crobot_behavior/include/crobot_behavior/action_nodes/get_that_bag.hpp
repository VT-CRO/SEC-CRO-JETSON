#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

class GetThatBag : public BT::SyncActionNode
{
public:
    GetThatBag(
        const std::string &name,
        const BT::NodeConfiguration &config
    );

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};