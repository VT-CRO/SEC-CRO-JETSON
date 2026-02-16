#pragma once
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <functional>
//placeholder values for params
using std::placeholders::_1;
using std::placeholders::_2;

class NavigationServer : public rclcpp::Node 
{
        
    public:
        using NavPoints = nav2_msgs::action::NavigateThroughPoses;
        using NavGoal = nav2_msgs::action::NavigateThroughPoses::Goal;
        using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavPoints>;

        NavigationServer(const rclcpp::NodeOptions & options);

        rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const NavGoal> goal);

        rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleNav> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle);

        void execute(const std::shared_ptr<GoalHandleNav> goal_handle);
    private:
        rclcpp_action::Server<NavPoints>::SharedPtr action_server_;
        double goal_x_;
        double goal_y_;

            


};