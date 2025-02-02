#ifndef IDLE_HPP_
#define IDLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp/behavior_tree.h"

class IdleBehavior : public BT::SyncActionNode
{
    public:
        IdleBehavior(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

        BT::NodeStatus tick() override;

    private:
        using BoolMsg = std_msgs::msg::Bool;
         
        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool _isIdle;
};

#endif