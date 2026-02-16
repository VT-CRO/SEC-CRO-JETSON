#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GoToPose : public BT::StatefulActionNode
{
public:
    GoToPose(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node_ptr
    );

    using NavPoints = nav2_msgs::action::NavigateThroughPoses;
    using NavGoal = nav2_msgs::action::NavigateThroughPoses::Goal;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavPoints>;

    static BT::PortsList providedPorts();  

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted();
    void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);

private:
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavPoints>::SharedPtr action_client_ptr_;
    NavGoal goal;
    std::string pose_strings;
    bool done_ = false;
};