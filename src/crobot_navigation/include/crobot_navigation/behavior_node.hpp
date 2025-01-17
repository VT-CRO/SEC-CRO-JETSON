#ifndef BEHAVIOR_NODE_HPP_
#define BEHAVIOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"


class BehaviorNode : public rclcpp::Node
{
    public:
        explicit BehaviorNode(const std::string &node_name);
        void setup();
        void create_behavior_tree();
        void update_behavior_tree();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        BT::Tree tree_;
};

#endif