#ifndef START_HPP_
#define START_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"

#include "behaviortree_cpp/behavior_tree.h"

class StartBehavior : public BT::StatefulActionNode
{
    public:
        StartBehavior(const std::string &name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr);

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;

        void onHalted();

    private:
        using BoolMsg = std_msgs::msg::Bool;

        using DynamicInterface = control_msgs::msg::DynamicInterfaceGroupValues;
        using InterfaceValue = control_msgs::msg::InterfaceValue;

        rclcpp::Publisher<DynamicInterface>::SharedPtr publisher_;

        rclcpp::Node::SharedPtr node_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;

        bool _shouldStart;

        void start_callback(const BoolMsg & msg);

        void timer_callback();
};

#endif