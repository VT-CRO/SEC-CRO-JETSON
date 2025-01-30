#include "crobot_navigation/behaviors/go_to_position.hpp"

GoToPosition::GoToPosition(
    const std::string &name,
    const BT::NodeConfig &config,
    const BT::RosNodeParams& params
    )
    : RosActionNode<NavigateToPose>(name, config, params)
{};

bool GoToPosition::setGoal(BT::RosActionNode<NavigateToPose>::Goal &goal)
{
    auto navGoal = getInput<nav2_msgs::action::NavigateToPose_Goal>("goal_pose", goal);

    if (!navGoal)
    {
        throw BT::RuntimeError("missing required input [goal_pose]: ", navGoal.error());
    }

    return true;
}

BT::NodeStatus GoToPosition::onResultReceived(const WrappedResult & /*wr*/)
{
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GoToPosition::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GoToPosition::onFeedback(const std::shared_ptr<const Feedback> /* feedback */)
{
    return BT::NodeStatus::RUNNING;
}