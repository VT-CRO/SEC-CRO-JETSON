#include "crobot_navigation/crobot_navigation_server.hpp"

namespace crobot_navigation
{
    CrobotNavigationActionServer::CrobotNavigationActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("crobot_navigation_server", options)
    {
        using namespace std::placeholders;
        
        RCLCPP_INFO(this->get_logger(), "Starting navigation server!");

        this->action_server_ = rclcpp_action::create_server<NavPose>(
            this,
            "crobot_navigation",
            std::bind(&CrobotNavigationActionServer::handle_goal, this, _1, _2),
            std::bind(&CrobotNavigationActionServer::handle_cancel, this, _1),
            std::bind(&CrobotNavigationActionServer::handle_accepted, this, _1)
        );
    }

    rclcpp_action::GoalResponse CrobotNavigationActionServer::handle_goal(const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request!");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse CrobotNavigationActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CrobotNavigationActionServer::handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&CrobotNavigationActionServer::execute, this, _1), goal_handle}.detach();
    }

    void CrobotNavigationActionServer::execute(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        // auto feedback = std::make_shared<NavPose::Feedback>();
        auto result = std::make_shared<NavPose::Result>();

        // TODO: setup path

        // while (running) {
        //      t = closestT
        //      desired_pos = pathbezier(t)
        //      TODO: p(id) current_pos with desired_pos
        //      publish command velocity
        //      stop running if t=1 and we're within threshold for a certain amount of time
        // }

        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(crobot_navigation::CrobotNavigationActionServer)