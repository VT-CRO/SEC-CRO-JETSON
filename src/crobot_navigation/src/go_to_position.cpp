#include "crobot_navigation/behaviors/go_to_position.hpp"

GoToPose::GoToPose(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr)
    : StatefulActionNode(name, config),
      node_ptr_(node_ptr)
{
    action_client_ptr_ = rclcpp_action::create_client<NavPose>(
        node_ptr_,
        "crobot_navigation"
    );
    done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return { BT::InputPort<NavGoal>("goalPose")};
}

BT::NodeStatus GoToPose::onStart()
{
    auto navGoal = getInput<NavGoal>("goalPose", _goal);

    if (!navGoal)
    {
        throw BT::RuntimeError("Missing required input [goal]");
    }

    std::stringstream ss;
    ss << "Sending goal: " << _goal.pose.pose.position.x << " " << _goal.pose.pose.position.y;
    RCLCPP_INFO(node_ptr_->get_logger(), ss.str().c_str());

    auto send_goal_options = rclcpp_action::Client<NavPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

    if (!this->action_client_ptr_->wait_for_action_server())
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Action server not available after waiting");
    }

    action_client_ptr_->async_send_goal(_goal, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
    if (done_flag_)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name());
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPose::onHalted()
{

}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result)
    {
        done_flag_ = true;
    }
}