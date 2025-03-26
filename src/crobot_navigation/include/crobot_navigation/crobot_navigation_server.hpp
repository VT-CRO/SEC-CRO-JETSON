#ifndef CROBOT_NAVIGATION_SERVER_HPP
#define CROBOT_NAVIGATION_SERVER_HPP

#include <functional>
#include <memory>
#include <thread>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "crobot_msgs/action/navigation_points.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "crobot_navigation/BezierPath.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>

namespace crobot_navigation
{
    class CrobotNavigationActionServer : public rclcpp::Node
    {
        public:
            using NavigationGoalPoints = crobot_msgs::action::NavigationPoints;
            using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigationGoalPoints>;

            explicit CrobotNavigationActionServer(const rclcpp::NodeOptions & options);

        private:
            rclcpp_action::Server<NavigationGoalPoints>::SharedPtr action_server_;
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
            Pose2D currentPos;

            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const NavigationGoalPoints::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleNav> goal_handle);

            void odom_cb(const nav_msgs::msg::Odometry msg);

            void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle);

            void execute(const std::shared_ptr<GoalHandleNav> goal_handle);
    };
}

#endif