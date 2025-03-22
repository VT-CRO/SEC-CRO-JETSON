#ifndef IDLE_HPP_
#define IDLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"

class IdleBehavior : public BT::SyncActionNode
{
    public:
        IdleBehavior(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

        BT::NodeStatus tick() override;

    private:
        using DynamicInterface = control_msgs::msg::DynamicInterfaceGroupValues;
        using InterfaceValue = control_msgs::msg::InterfaceValue;

        rclcpp::Publisher<DynamicInterface>::SharedPtr publisher_;
        using BoolMsg = std_msgs::msg::Bool;
         
        rclcpp::Node::SharedPtr node_ptr_;

        // bool val;
};

#endif