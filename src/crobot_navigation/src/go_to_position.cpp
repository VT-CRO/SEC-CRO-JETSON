#include "crobot_navigation/behaviors/go_to_position.hpp"

GoToPose::GoToPose(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config),
    node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavPose>(node_ptr_, "navigate_to_pose");
    done_flag_ = false;
    success_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return { BT::InputPort<NavGoal>("goalPose") };
}

BT::NodeStatus GoToPose::onStart() 
{
    done_flag_ = false;
    success_flag_ = false;

    auto navGoal  = getInput<NavGoal>("goalPose", _goal);
    if (!navGoal) {
        throw BT::RuntimeError("Missing required input [goalPose]");
        std::stringstream ss;
        ss << "Sending goal: " << _goal.pose.pose.position.x << " " << _goal.pose.pose.position.y << " " << _goal.pose.pose.orientation.z;
        RCLCPP_INFO(node_ptr_->get_logger(), ss.str().c_str());
    }

    if (!action_client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "navigate_to_pose action server not available");
        return BT::NodeStatus::FAILURE;
    }

    auto send_goal_options = rclcpp_action::Client<NavPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

    action_client_ptr_->async_send_goal(_goal, send_goal_options);

    RCLCPP_INFO(node_ptr_->get_logger(), "Sent goal: (%.3f, %.3f)", _goal.pose.pose.position.x, _goal.pose.pose.position.y);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
    if (!done_flag_) {
        return BT::NodeStatus::RUNNING;
    }

    return success_flag_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void GoToPose::onHalted()
{

}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    done_flag_ = true;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            success_flag_ = true;
            RCLCPP_INFO(node_ptr_->get_logger(), "navigate_to_pose SUCCEEDED");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            success_flag_ = false;
            RCLCPP_WARN(node_ptr_->get_logger(), "navigate_to_pose ABORTED");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            success_flag_ = false;
            RCLCPP_WARN(node_ptr_->get_logger(), "navigate_to_pose CANCELED");
            break;
        default:
            success_flag_ = false;
            RCLCPP_WARN(node_ptr_->get_logger(), "navigate_to_pose result unknown");
            break;
    }
}