#include "crobot_navigation/behaviors/odom_go_to_position.hpp"


GoToPoseWithOdometry::GoToPoseWithOdometry(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_ptr)
  : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<NavPoints>(
    node_ptr_,
    "crobot_navigation"
  );
  done_flag_ = false;
}

BT::PortsList GoToPoseWithOdometry::providedPorts()
{
    return {BT::InputPort<NavPoints::Goal>("goalPoints")};
}

BT::NodeStatus GoToPoseWithOdometry::onStart()
{
    auto navGoal = getInput<NavPoints::Goal>("goalPoints", _action_msg);

    if (!navGoal)
    {
        throw BT::RuntimeError("Missing required input [goalPoints]");
    }
    auto send_goal_options = rclcpp_action::Client<NavPoints>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPoseWithOdometry::nav_to_pose_callback, this, std::placeholders::_1);

    if (!this->action_client_ptr_->wait_for_action_server())
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Action server not available after waiting");
    }

    action_client_ptr_->async_send_goal(_action_msg, send_goal_options);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPoseWithOdometry::onRunning()
{
    if (done_flag_)
    {
        RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPoseWithOdometry::onHalted()
{

}

void GoToPoseWithOdometry::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result)
    {
        done_flag_ = true;
    }
}